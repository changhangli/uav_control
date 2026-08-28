#!/usr/bin/env bash
source /opt/ros/noetic/setup.bash
source /home/uav/catkin_ws/devel/setup.bash
source /home/uav/land_ws/devel/setup.bash
export ROS_PACKAGE_PATH="/home/uav/land_ws/src:/home/uav/catkin_ws/src:${ROS_PACKAGE_PATH}"
export ROS_MASTER_URI=http://127.0.0.1:11311

L=$(readlink -f "$HOME/uav_worklog/run/stage_a_latest")
echo "DIR=$L"
ls -la "$L"
echo "===== WATCH (PASS/FAIL/pause) ====="
grep -nE 'PASS|FAIL|pause_demo' "$L/watch.log" | tail -40 || true
echo "===== FAKE DEMO (bytes) ====="
wc -c "$L/fake_demo.log"; tail -n 20 "$L/fake_demo.log"
echo "===== FAKE TAG (bytes) ====="
wc -c "$L/fake_tag.log"; tail -n 20 "$L/fake_tag.log"
echo "===== LISTEN (tag/pause/LISTEN/ROUTE) ====="
grep -nE 'LISTEN|ROUTE|pause|见码|BENCH|TX |INIT' "$L/listen.log" | tail -40 || true
echo "===== LIVE TOPICS ====="
timeout 2 rostopic echo -n 1 /uav1/pause_demo || true
timeout 2 rostopic echo -n 1 /uav1/demo_control_cmd || true
timeout 2 rostopic echo -n 1 /uav1/vision_control_cmd || true
timeout 2 rostopic echo -n 1 /uav1/uav_control_cmd || true
timeout 2 rostopic echo -n 1 /uav1/listen_status || true
timeout 2 rostopic hz /uav1/sunray_detect/qrcode_detection_ros || true
