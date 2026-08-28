#!/usr/bin/env bash
# 停止 start_vision_verify.sh 拉起的后台进程

source /opt/ros/noetic/setup.bash 2>/dev/null || true
source "$HOME/vision_ws/devel/setup.bash" 2>/dev/null || true

PID_DIR="$HOME/uav_worklog/run"

stop_pidfile() {
  local f="$1"
  if [ -f "$f" ]; then
    local pid
    pid=$(cat "$f")
    if kill -0 "$pid" 2>/dev/null; then
      echo "kill $pid ($f)"
      kill "$pid" 2>/dev/null || true
      sleep 1
      kill -9 "$pid" 2>/dev/null || true
    fi
    rm -f "$f"
  fi
}

echo "停止 gate / vision_verify launch / roscore ..."
stop_pidfile "$PID_DIR/landmark_pose_gate.pid"
stop_pidfile "$PID_DIR/vision_verify_launch.pid"
stop_pidfile "$PID_DIR/roscore.pid"

# 再保险：按节点名停
if rostopic list >/dev/null 2>&1; then
  rosnode kill /landmark_pose_gate >/dev/null 2>&1 || true
  rosnode kill /landmark_detection_ros_node >/dev/null 2>&1 || true
  rosnode kill /web_cam >/dev/null 2>&1 || true
fi

echo "完成。若还要关 roscore: killall roscore rosmaster 2>/dev/null || true"
