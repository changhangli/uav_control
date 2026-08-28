#!/usr/bin/env bash
source /opt/ros/noetic/setup.bash
source /home/uav/catkin_ws/devel/setup.bash
source /home/uav/land_ws/devel/setup.bash
export ROS_PACKAGE_PATH="/home/uav/land_ws/src:/home/uav/catkin_ws/src:${ROS_PACKAGE_PATH}"
export ROS_MASTER_URI=http://127.0.0.1:11311

echo "=== PROCS ==="
pgrep -af "fake_demo|watch_interrupt|auto_land|auto_listen|vision_cmd_mux|web_cam|landmark_detection" || echo none

echo "=== LIVE ==="
timeout 2 rostopic echo -n 1 /uav1/pause_demo 2>/dev/null || echo "(no pause_demo)"
timeout 2 rostopic echo -n 1 /uav1/listen_status 2>/dev/null || echo "(no listen_status)"

echo "=== SEARCH INTERRUPT_PASS ==="
grep -RsnF "[WATCH] INTERRUPT_PASS" /home/uav/uav_worklog/run 2>/dev/null | tail -30 || true
grep -Rsn "pause_demo: False -> True" /home/uav/uav_worklog/run 2>/dev/null | tail -30 || true

echo "=== STAGE_B DIRS ==="
ls -lt /home/uav/uav_worklog/run | head -20

# also check nohup / home terminal leftovers and ~/.ros
echo "=== ROS LOG auto_land (latest) ==="
LATEST=$(ls -dt /home/uav/.ros/log/*/ 2>/dev/null | head -1)
echo "logdir=$LATEST"
if [ -n "$LATEST" ]; then
  grep -nE "见码|pause_demo|LISTEN|ROUTE-LAND|BENCH|TX " "$LATEST"/auto_land_*.log 2>/dev/null | tail -50 || true
fi

# manual runs often print only to terminals; also scan common nohup files
echo "=== ANY WATCH LOGS UNDER run ==="
find /home/uav/uav_worklog/run -name 'watch.log' -o -name '*watch*' 2>/dev/null | head -30
for f in $(find /home/uav/uav_worklog/run -name 'watch.log' 2>/dev/null | xargs ls -t 2>/dev/null | head -5); do
  echo "---- $f ----"
  grep -nE "INTERRUPT_|pause_demo|listen_status|phase=" "$f" | tail -50 || true
done

for f in $(find /home/uav/uav_worklog/run -name 'listen.log' 2>/dev/null | xargs ls -t 2>/dev/null | head -5); do
  echo "---- $f ----"
  grep -nE "见码|LISTEN|ROUTE|pause|TX |nm=|n_markers" "$f" | tail -40 || true
done

# screen/tmux? also check /tmp
echo "=== /tmp watch-ish ==="
ls -lt /tmp/*watch* /tmp/*listen* 2>/dev/null | head || true
