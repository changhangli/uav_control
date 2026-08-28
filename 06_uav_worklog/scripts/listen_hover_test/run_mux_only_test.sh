#!/usr/bin/env bash
# 纯测 vision_cmd_mux：不启视觉、不启降落节点
set -e
RUN="$HOME/uav_worklog/run/mux_only_$(date +%Y%m%d_%H%M%S)"
mkdir -p "$RUN"
ln -sfn "$RUN" "$HOME/uav_worklog/run/mux_only_latest"

source /opt/ros/noetic/setup.bash
source /home/uav/catkin_ws/devel/setup.bash
source /home/uav/land_ws/devel/setup.bash
export ROS_PACKAGE_PATH="/home/uav/land_ws/src:/home/uav/catkin_ws/src:${ROS_PACKAGE_PATH}"
export ROS_MASTER_URI=http://127.0.0.1:11311
export ROS_IP=127.0.0.1
export PYTHONUNBUFFERED=1

echo "[MUXTEST] RUN=$RUN"

# 停掉可能抢总线的节点，保留 roscore
pkill -9 -f 'fake_demo_cmd.py' 2>/dev/null || true
pkill -9 -f 'fake_tag.py' 2>/dev/null || true
pkill -9 -f 'watch_interrupt.py' 2>/dev/null || true
pkill -9 -f 'auto_listen_hover' 2>/dev/null || true
pkill -9 -f 'auto_land_by_pose' 2>/dev/null || true
pkill -9 -f 'vision_cmd_mux' 2>/dev/null || true
pkill -9 -f 'mux_probe' 2>/dev/null || true
sleep 1

if ! rostopic list >/dev/null 2>&1; then
  echo "[MUXTEST] start roscore"
  nohup roscore >"$RUN/roscore.log" 2>&1 &
  for i in $(seq 1 30); do
    rostopic list >/dev/null 2>&1 && break
    sleep 0.3
  done
fi

echo "[MUXTEST] start vision_cmd_mux only"
nohup roslaunch sunray_tutorial vision_cmd_mux.launch \
  >"$RUN/mux.log" 2>&1 &
echo $! >"$RUN/mux.pid"
sleep 2

python3 - <<'PY'
import time
import rospy
from std_msgs.msg import Bool
from swarm_msgs.msg import UAVControlCMD

rospy.init_node("mux_probe")

pub_demo = rospy.Publisher("/uav1/demo_control_cmd", UAVControlCMD, queue_size=1)
pub_vis = rospy.Publisher("/uav1/vision_control_cmd", UAVControlCMD, queue_size=1)
pub_pause = rospy.Publisher("/uav1/pause_demo", Bool, queue_size=1, latch=True)
out_holder = {"msg": None, "n": 0}

def on_out(msg):
    out_holder["msg"] = msg
    out_holder["n"] += 1

rospy.Subscriber("/uav1/uav_control_cmd", UAVControlCMD, on_out, queue_size=10)
time.sleep(1.0)

def mk(cmd, x, y, z, yaw, frame):
    m = UAVControlCMD()
    m.header.stamp = rospy.Time.now()
    m.header.frame_id = frame
    m.cmd = cmd
    m.desired_pos = [x, y, z]
    m.desired_yaw = yaw
    return m

def set_pause(v):
    pub_pause.publish(Bool(data=v))
    time.sleep(0.3)

def blast(which, seconds=1.0):
    """持续发 demo 或 vision，便于 mux 20Hz 采样到。"""
    t0 = time.time()
    n = 0
    while time.time() - t0 < seconds and not rospy.is_shutdown():
        if which == "both":
            pub_demo.publish(mk(4, 9.0, 9.0, 1.5, 0.77, "DEMO_FINGERPRINT"))
            pub_vis.publish(mk(14, 1.0, 2.0, -0.05, 0.11, "VISION_FINGERPRINT"))
        elif which == "demo":
            pub_demo.publish(mk(4, 9.0, 9.0, 1.5, 0.77, "DEMO_FINGERPRINT"))
        else:
            pub_vis.publish(mk(14, 1.0, 2.0, -0.05, 0.11, "VISION_FINGERPRINT"))
        n += 1
        time.sleep(0.05)
    return n

def sample_out(timeout=2.0):
    out_holder["msg"] = None
    t0 = time.time()
    while time.time() - t0 < timeout and out_holder["msg"] is None:
        time.sleep(0.05)
        rospy.sleep(0.01)
    return out_holder["msg"]

results = []

def check(name, pause, expect_frame, expect_cmd):
    set_pause(pause)
    # 两边都发，看 mux 选谁
    blast("both", 1.2)
    msg = sample_out(2.0)
    if msg is None:
        results.append((name, False, "NO_OUTPUT"))
        print("[FAIL] %s : no /uav1/uav_control_cmd" % name)
        return
    ok = (msg.header.frame_id == expect_frame) and (msg.cmd == expect_cmd)
    # 额外看位置指纹
    if expect_frame == "DEMO_FINGERPRINT":
        ok = ok and abs(msg.desired_pos[0] - 9.0) < 1e-3
    else:
        ok = ok and abs(msg.desired_pos[0] - 1.0) < 1e-3
    results.append((name, ok, "frame=%s cmd=%s pos=[%.2f %.2f %.2f] yaw=%.3f" % (
        msg.header.frame_id, msg.cmd,
        msg.desired_pos[0], msg.desired_pos[1], msg.desired_pos[2],
        msg.desired_yaw)))
    print("[%s] %s : %s" % ("PASS" if ok else "FAIL", name, results[-1][2]))

print("==== CASE1 pause=false 应出口=DEMO ====")
check("pause_false_select_demo", False, "DEMO_FINGERPRINT", 4)

print("==== CASE2 pause=true 应出口=VISION ====")
check("pause_true_select_vision", True, "VISION_FINGERPRINT", 14)

print("==== CASE3 再切回 pause=false 应回到 DEMO ====")
check("pause_false_again", False, "DEMO_FINGERPRINT", 4)

print("==== CASE4 pause=true 且只发 demo（无 vision）应无出口或保持旧视觉？ ====")
# mux: vision_active && have_vision -> vision; else if !vision_active && have_demo -> demo
# pause=true 但没有新 vision：若 have_vision 仍真，会继续发 last_vision
set_pause(True)
# 先确保有过 vision
blast("vision", 0.5)
# 再只发 demo
out_holder["msg"] = None
before_n = out_holder["n"]
blast("demo", 1.0)
time.sleep(0.3)
msg = out_holder["msg"]
# 期望：pause=true 时即使 demo 在发，出口仍是 VISION（last vision），不是 DEMO
if msg is not None and msg.header.frame_id == "VISION_FINGERPRINT" and msg.cmd == 14:
    print("[PASS] pause_true_demo_still_blocked : out stays VISION while demo spam")
    results.append(("pause_true_demo_blocked", True, "out=VISION"))
else:
    detail = "none" if msg is None else ("frame=%s cmd=%s" % (msg.header.frame_id, msg.cmd))
    print("[FAIL] pause_true_demo_still_blocked : %s" % detail)
    results.append(("pause_true_demo_blocked", False, detail))

print("==== SUMMARY ====")
all_ok = all(ok for _, ok, _ in results)
for name, ok, detail in results:
    print("  %s  %s  %s" % ("PASS" if ok else "FAIL", name, detail))
print("MUX_TEST_PASS" if all_ok else "MUX_TEST_FAIL")
raise SystemExit(0 if all_ok else 1)
PY

rc=$?
echo "======== mux.log ========"
tail -n 30 "$RUN/mux.log" || true
exit $rc
