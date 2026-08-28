#!/usr/bin/env bash
source /opt/ros/noetic/setup.bash
source /home/uav/catkin_ws/devel/setup.bash
source /home/uav/land_ws/devel/setup.bash
export ROS_PACKAGE_PATH="/home/uav/land_ws/src:/home/uav/catkin_ws/src:${ROS_PACKAGE_PATH}"
export ROS_MASTER_URI=http://127.0.0.1:11311

echo "=== pause_demo ==="
timeout 2 rostopic echo -n 1 /uav1/pause_demo
echo "=== listen_status ==="
timeout 2 rostopic echo -n 1 /uav1/listen_status
echo "=== demo (should still be fake route) ==="
timeout 2 rostopic echo -n 1 /uav1/demo_control_cmd | head -20
echo "=== vision ==="
timeout 2 rostopic echo -n 1 /uav1/vision_control_cmd | head -20
echo "=== OUT uav_control_cmd (should match vision, not demo) ==="
timeout 2 rostopic echo -n 1 /uav1/uav_control_cmd | head -20
echo "=== auto_land ros log key ==="
grep -E "见码|pause|ROUTE|LISTEN|TX " /home/uav/.ros/log/a5ba0fca-9529-11f1-a611-e1192292e977/auto_land_1-2.log 2>/dev/null | tail -40
