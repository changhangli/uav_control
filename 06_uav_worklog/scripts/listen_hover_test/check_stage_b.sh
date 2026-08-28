#!/usr/bin/env bash
source /opt/ros/noetic/setup.bash
source /home/uav/catkin_ws/devel/setup.bash
source /home/uav/land_ws/devel/setup.bash
export ROS_PACKAGE_PATH="/home/uav/land_ws/src:/home/uav/catkin_ws/src:${ROS_PACKAGE_PATH}"
export ROS_MASTER_URI=http://127.0.0.1:11311
L=$(readlink -f "$HOME/uav_worklog/run/stage_b_latest")
echo "DIR=$L"
echo "=== WATCH ==="
grep -nF "[WATCH] INTERRUPT_PASS" "$L/watch.log" || echo "(no real PASS yet)"
grep -n "pause_demo" "$L/watch.log" | tail -10 || true
echo "=== LIVE ==="
timeout 2 rostopic echo -n 1 /uav1/pause_demo || true
timeout 2 rostopic echo -n 1 /uav1/listen_status || true
