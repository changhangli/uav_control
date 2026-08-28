#!/usr/bin/env bash
# Stage B: 真相机 + 假航线 + bench（不真飞）
set -e
RUN_DIR="$HOME/uav_worklog/run/stage_b_$(date +%Y%m%d_%H%M%S)"
SCRIPT_DIR="$HOME/uav_worklog/scripts/listen_hover_test"
mkdir -p "$RUN_DIR"
ln -sfn "$RUN_DIR" "$HOME/uav_worklog/run/stage_b_latest"

source /opt/ros/noetic/setup.bash
source /home/uav/catkin_ws/devel/setup.bash
if [ -f /home/uav/vision_ws/devel/setup.bash ]; then
  source /home/uav/vision_ws/devel/setup.bash
fi
source /home/uav/land_ws/devel/setup.bash
export ROS_PACKAGE_PATH="/home/uav/land_ws/src:/home/uav/catkin_ws/src:${ROS_PACKAGE_PATH}"
export ROS_MASTER_URI=http://127.0.0.1:11311
export ROS_IP=127.0.0.1
export PYTHONUNBUFFERED=1

echo "[STAGE_B] RUN_DIR=$RUN_DIR"
echo "[STAGE_B] 安全说明: bench_mode=true，不解锁、不转桨、不伤飞控"

# 清掉 Stage A 残留（保留 roscore）
pkill -9 -f 'fake_demo_cmd.py' 2>/dev/null || true
pkill -9 -f 'fake_tag.py' 2>/dev/null || true
pkill -9 -f 'watch_interrupt.py' 2>/dev/null || true
pkill -9 -f 'auto_listen_hover.launch' 2>/dev/null || true
pkill -9 -f 'auto_land_by_pose' 2>/dev/null || true
pkill -9 -f 'vision_cmd_mux' 2>/dev/null || true
sleep 1

echo "[STAGE_B] 启动真视觉（daemon）..."
bash "$HOME/uav_worklog/scripts/start_vision_verify.sh" --daemon \
  >"$RUN_DIR/vision_start.log" 2>&1 || {
  echo "[STAGE_B] 视觉启动失败，看日志:"
  tail -n 80 "$RUN_DIR/vision_start.log" || true
  tail -n 40 "$HOME/uav_worklog/run/vision_verify_launch.log" || true
  exit 1
}
cp -f "$RUN_DIR/vision_start.log" "$RUN_DIR/vision_start.ok.log" 2>/dev/null || true
echo "[STAGE_B] 视觉就绪"

echo "[STAGE_B] 启动 auto_listen_hover bench_mode:=true ..."
nohup roslaunch sunray_tutorial auto_listen_hover.launch bench_mode:=true \
  >"$RUN_DIR/listen.log" 2>&1 &
echo $! >"$RUN_DIR/listen.pid"
sleep 4

echo "[STAGE_B] 启动假航线 + 监视..."
nohup python3 -u "$SCRIPT_DIR/fake_demo_cmd.py" >"$RUN_DIR/fake_demo.log" 2>&1 &
echo $! >"$RUN_DIR/fake_demo.pid"
nohup python3 -u "$SCRIPT_DIR/watch_interrupt.py" >"$RUN_DIR/watch.log" 2>&1 &
echo $! >"$RUN_DIR/watch.pid"

echo
echo "============================================================"
echo " 请把摄像头对准降落板（二维码板）"
echo " 看到板后应自动: pause=true + PASS"
echo " 最长等待 120 秒..."
echo "============================================================"

PASS=0
for i in $(seq 1 240); do
  if grep -qF "[WATCH] INTERRUPT_PASS" "$RUN_DIR/watch.log" 2>/dev/null \
     && grep -q "pause=1" "$RUN_DIR/watch.log" 2>/dev/null; then
    PASS=1
    break
  fi
  # 每 5 秒打印一次视觉是否有码
  if [ $((i % 10)) -eq 0 ]; then
    st=$(timeout 1 rostopic echo -n 1 /uav1/listen_status 2>/dev/null | grep "data:" | head -1)
    echo "[STAGE_B] waiting... ${i}/240  ${st:-n/a}"
  fi
  sleep 0.5
done

echo "======== listen.log (key lines) ========"
grep -E 'INIT|BENCH|LISTEN|ROUTE|pause|见码|TX ' "$RUN_DIR/listen.log" | tail -n 40 || true
echo "======== watch.log (key lines) ========"
grep -E 'INTERRUPT_|pause_demo' "$RUN_DIR/watch.log" | tail -n 20 || true
echo "======== live status ========"
timeout 2 rostopic echo -n 1 /uav1/pause_demo || true
timeout 2 rostopic echo -n 1 /uav1/listen_status || true
echo "======== RESULT ========"
if [ "$PASS" -eq 1 ]; then
  echo "STAGE_B_PASS"
  exit 0
fi
echo "STAGE_B_TIMEOUT — 多半是相机还没对准降落板"
echo "节点仍在跑；对准后再看: tail -f $RUN_DIR/watch.log"
exit 2
