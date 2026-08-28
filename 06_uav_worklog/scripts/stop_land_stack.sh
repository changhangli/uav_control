#!/usr/bin/env bash
# 停止 start_land_stack.sh / start_vision_verify 拉起的进程
#
#   bash ~/uav_worklog/scripts/stop_land_stack.sh

source /opt/ros/noetic/setup.bash 2>/dev/null || true
source "$HOME/vision_ws/devel/setup.bash" 2>/dev/null || true
source "$HOME/land_ws/devel/setup.bash" 2>/dev/null || true

PID_DIR="$HOME/uav_worklog/run"

stop_pidfile() {
  local f="$1"
  local name="$2"
  if [ -f "$f" ]; then
    local pid
    pid=$(cat "$f")
    if kill -0 "$pid" 2>/dev/null; then
      echo "kill $pid ($name)"
      kill "$pid" 2>/dev/null || true
      sleep 0.5
      kill -9 "$pid" 2>/dev/null || true
    fi
    rm -f "$f"
  fi
}

echo "停止降落支撑进程 ..."
# 先停降落节点（若还在）
if rostopic list >/dev/null 2>&1; then
  rosnode kill /auto_land_1 >/dev/null 2>&1 || true
fi
pkill -f "auto_land_by_pose.launch" 2>/dev/null || true

stop_pidfile "$PID_DIR/land_bag.pid" "rosbag"
# rosbag 子进程有时不随父死
pkill -f "rosbag record .*uav_control_cmd" 2>/dev/null || true

stop_pidfile "$PID_DIR/watch_land_status.pid" "watch_land_status"
pkill -f "watch_land_status.py" 2>/dev/null || true

stop_pidfile "$PID_DIR/watch_uav_control_cmd.pid" "watch_uav_control_cmd"
pkill -f "watch_uav_control_cmd.py" 2>/dev/null || true

stop_pidfile "$PID_DIR/record_landmark_pose.pid" "record_landmark_pose"
pkill -f "record_landmark_pose.py" 2>/dev/null || true

# 视觉栈（复用原脚本逻辑）
bash "$HOME/uav_worklog/scripts/stop_vision_verify.sh"

echo "完成。"
