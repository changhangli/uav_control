#!/usr/bin/env bash
# 录降落相关 ROS bag（写到咱们自己的 records，不碰 catkin_ws）
#
# 用法：
#   bash ~/uav_worklog/scripts/record_land_bag.sh
# 结束：Ctrl+C

set -e

source /opt/ros/noetic/setup.bash
# 只读消息/话题定义，不修改 catkin_ws
source "$HOME/catkin_ws/devel/setup.bash"
source "$HOME/vision_ws/devel/setup.bash"
source "$HOME/land_ws/devel/setup.bash"

OUT_DIR="$HOME/uav_worklog/records"
mkdir -p "$OUT_DIR"
BAG="$OUT_DIR/land_$(date +%Y%m%d_%H%M%S).bag"

echo "recording -> $BAG"
echo "Ctrl+C 停止"
echo

exec rosbag record -O "$BAG" \
  /uav1/uav_control_cmd \
  /uav1/swarm/uav_state \
  /uav1/sunray_detect/qrcode_detection_ros \
  /uav1/mavros/setpoint_raw/local \
  /uav1/mavros/local_position/pose
