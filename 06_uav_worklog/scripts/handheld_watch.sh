#!/usr/bin/env bash
# 带 ROS 环境启动白话提示（避免 ModuleNotFoundError）
source /opt/ros/noetic/setup.bash
source "$HOME/vision_ws/devel/setup.bash"
source "$HOME/land_ws/devel/setup.bash"
exec python3 "$HOME/uav_worklog/scripts/handheld_watch.py"
