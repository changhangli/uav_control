#!/usr/bin/env bash
# 桌面终端运行：
#   bash ~/uav_worklog/scripts/run_camera_calibrate.sh
# 可选参数：SIZE SQUARE，例如
#   bash ~/uav_worklog/scripts/run_camera_calibrate.sh 8x5 0.020

source /opt/ros/noetic/setup.bash
source "$HOME/vision_ws/devel/setup.bash"
exec python3 "$HOME/uav_worklog/scripts/run_camera_calibrate.py" "$@"
