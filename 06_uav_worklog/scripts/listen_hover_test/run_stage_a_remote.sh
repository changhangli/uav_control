#!/usr/bin/env bash
set -e
RUN_DIR="$HOME/uav_worklog/run/stage_a_$(date +%Y%m%d_%H%M%S)"
SCRIPT_DIR="$HOME/uav_worklog/scripts/listen_hover_test"
mkdir -p "$RUN_DIR"
ln -sfn "$RUN_DIR" "$HOME/uav_worklog/run/stage_a_latest"

# Overlay order: each later workspace must remain visible.
# vision_ws setup can wipe previous paths, so land_ws is sourced LAST.
source /opt/ros/noetic/setup.bash
source /home/uav/catkin_ws/devel/setup.bash
if [ -f /home/uav/vision_ws/devel/setup.bash ]; then
  source /home/uav/vision_ws/devel/setup.bash
fi
source /home/uav/land_ws/devel/setup.bash
# Ensure land_ws/src is searchable even if devel overlay is thin
export ROS_PACKAGE_PATH="/home/uav/land_ws/src:/home/uav/catkin_ws/src:${ROS_PACKAGE_PATH}"

export ROS_MASTER_URI=http://127.0.0.1:11311
export ROS_IP=127.0.0.1

echo "[STAGE_A] RUN_DIR=$RUN_DIR"
echo "[STAGE_A] ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH"
echo "[STAGE_A] checking packages..."
rospack find sunray_tutorial
rospack find swarm_msgs
rospack find detection_msgs || rospack find detection_msg

pkill -f 'fake_demo_cmd.py' 2>/dev/null || true
pkill -f 'fake_tag.py' 2>/dev/null || true
pkill -f 'watch_interrupt.py' 2>/dev/null || true
pkill -f 'auto_listen_hover.launch' 2>/dev/null || true
pkill -f 'vision_cmd_mux' 2>/dev/null || true
pkill -f 'auto_land_by_pose' 2>/dev/null || true
sleep 1

if ! rostopic list >/dev/null 2>&1; then
  echo "[STAGE_A] starting roscore..."
  nohup roscore >"$RUN_DIR/roscore.log" 2>&1 &
  echo $! >"$RUN_DIR/roscore.pid"
  for i in $(seq 1 40); do
    if rostopic list >/dev/null 2>&1; then
      echo "[STAGE_A] roscore ready"
      break
    fi
    sleep 0.5
  done
  if ! rostopic list >/dev/null 2>&1; then
    echo "[STAGE_A] roscore failed"
    tail -n 50 "$RUN_DIR/roscore.log" || true
    exit 1
  fi
else
  echo "[STAGE_A] roscore already up"
fi

echo "[STAGE_A] launching auto_listen_hover bench_mode:=true ..."
nohup roslaunch sunray_tutorial auto_listen_hover.launch bench_mode:=true \
  >"$RUN_DIR/listen.log" 2>&1 &
echo $! >"$RUN_DIR/listen.pid"
sleep 4

if ! grep -q "vision_cmd_mux\|auto_land\|LISTEN\|BENCH\|INIT" "$RUN_DIR/listen.log" 2>/dev/null; then
  echo "[STAGE_A] listen log still quiet, wait more..."
  sleep 3
fi

echo "[STAGE_A] starting fake_demo_cmd ..."
nohup python3 "$SCRIPT_DIR/fake_demo_cmd.py" >"$RUN_DIR/fake_demo.log" 2>&1 &
echo $! >"$RUN_DIR/fake_demo.pid"
sleep 1

echo "[STAGE_A] starting watch_interrupt ..."
nohup python3 "$SCRIPT_DIR/watch_interrupt.py" >"$RUN_DIR/watch.log" 2>&1 &
echo $! >"$RUN_DIR/watch.pid"
sleep 1

echo "[STAGE_A] starting fake_tag quiet_sec:=8 ..."
nohup python3 "$SCRIPT_DIR/fake_tag.py" _quiet_sec:=8 _n_markers:=4 \
  _px:=0.25 _py:=-0.15 _pz:=-1.2 \
  >"$RUN_DIR/fake_tag.log" 2>&1 &
echo $! >"$RUN_DIR/fake_tag.pid"

echo "[STAGE_A] waiting up to 30s for PASS..."
PASS=0
for i in $(seq 1 60); do
  if grep -q "PASS: interrupt OK" "$RUN_DIR/watch.log" 2>/dev/null; then
    PASS=1
    break
  fi
  sleep 0.5
done

echo "======== listen.log (tail) ========"
tail -n 60 "$RUN_DIR/listen.log" || true
echo "======== fake_demo.log (tail) ========"
tail -n 30 "$RUN_DIR/fake_demo.log" || true
echo "======== fake_tag.log (tail) ========"
tail -n 30 "$RUN_DIR/fake_tag.log" || true
echo "======== watch.log ========"
cat "$RUN_DIR/watch.log" || true
echo "======== topics ========"
rostopic list 2>/dev/null | grep -E 'uav1/(demo|vision|pause|listen|uav_control|qrcode)' || true
echo "======== RESULT ========"
if [ "$PASS" -eq 1 ]; then
  echo "STAGE_A_PASS"
  exit 0
fi
echo "STAGE_A_FAIL_OR_TIMEOUT"
exit 1
