#!/usr/bin/env bash
# 视觉验证启动（Ctrl+C 只停记录，不杀相机/识别）
#
# 用法：
#   bash ~/uav_worklog/scripts/start_vision_verify.sh
#   bash ~/uav_worklog/scripts/start_vision_verify.sh --daemon   # 只起后台，不占终端记 CSV
#
# 停止全部：
#   bash ~/uav_worklog/scripts/stop_vision_verify.sh
# 推荐试飞一键：
#   bash ~/uav_worklog/scripts/start_land_stack.sh

set -e

DAEMON=0
for a in "$@"; do
  case "$a" in
    --daemon|-d) DAEMON=1 ;;
  esac
done

source /opt/ros/noetic/setup.bash
source "$HOME/vision_ws/devel/setup.bash"

PID_DIR="$HOME/uav_worklog/run"
mkdir -p "$PID_DIR" "$HOME/uav_worklog/records"

echo "[1/4] 检查 / 启动 roscore ..."
if ! rostopic list >/dev/null 2>&1; then
  nohup roscore >"$HOME/uav_worklog/run/roscore.log" 2>&1 &
  echo $! >"$PID_DIR/roscore.pid"
  for i in $(seq 1 20); do
    if rostopic list >/dev/null 2>&1; then
      break
    fi
    sleep 0.5
  done
  if ! rostopic list >/dev/null 2>&1; then
    echo "roscore 启动失败，看: $HOME/uav_worklog/run/roscore.log"
    exit 1
  fi
  echo "  roscore 已启动"
else
  echo "  roscore 已在运行"
fi

echo "[2/4] 启动相机 + landmark_detection ..."
# 已有节点则不重复拉起
NEED_LAUNCH=0
rosnode list 2>/dev/null | grep -q "/web_cam" || NEED_LAUNCH=1
rosnode list 2>/dev/null | grep -q "/landmark_detection_ros_node" || NEED_LAUNCH=1

if [ "$NEED_LAUNCH" = "1" ]; then
  # 清掉可能残留的同名节点失败状态
  nohup roslaunch landmark_detection_ros vision_verify.launch \
    >"$HOME/uav_worklog/run/vision_verify_launch.log" 2>&1 &
  echo $! >"$PID_DIR/vision_verify_launch.pid"
  disown || true
else
  echo "  相机/识别节点已在运行，跳过 launch"
fi

echo "[3/4] 等待节点就绪 ..."
ok=0
for i in $(seq 1 40); do
  has_cam=0
  has_lm=0
  rosnode list 2>/dev/null | grep -q "/web_cam" && has_cam=1
  rosnode list 2>/dev/null | grep -q "/landmark_detection_ros_node" && has_lm=1
  if [ "$has_cam" = "1" ] && [ "$has_lm" = "1" ]; then
    # 注意：timeout 到期退出码是 124，不能当失败；只要能 echo 到一条消息即可
    if timeout 3 rostopic echo -n 1 /uav1/sunray_detect/qrcode_detection_ros \
        >"$PID_DIR/echo_check.txt" 2>&1; then
      if grep -q "targets:" "$PID_DIR/echo_check.txt"; then
        ok=1
        break
      fi
    else
      # timeout 也可能已经抓到内容
      if grep -q "targets:" "$PID_DIR/echo_check.txt" 2>/dev/null; then
        ok=1
        break
      fi
    fi
  fi
  echo "  等待中... cam=$has_cam landmark=$has_lm ($i/40)"
  sleep 1
done

if [ "$ok" != "1" ]; then
  echo "节点未就绪。最近日志："
  tail -40 "$HOME/uav_worklog/run/vision_verify_launch.log" || true
  echo
  echo "若看到 markerIds.length 报错，先修 json 再重试。"
  exit 1
fi

echo "  节点就绪。检测话题有数据。"

echo "[4/5] 启动位姿门控 ..."
if rosnode list 2>/dev/null | grep -q "/landmark_pose_gate"; then
  echo "  门控节点已在运行"
else
  nohup python3 "$HOME/uav_worklog/scripts/landmark_pose_gate.py" \
    >"$HOME/uav_worklog/run/landmark_pose_gate.log" 2>&1 &
  echo $! >"$PID_DIR/landmark_pose_gate.pid"
  disown || true
  sleep 1
  echo "  门控已启动 → /uav1/sunray_detect/landmark_pose_gated"
fi

echo
echo "重要："
echo "  - 请把摄像头对准降落板"
echo "  - 看画面: rqt_image_view → .../image_rect"
echo

if [ "$DAEMON" = "1" ]; then
  # 后台记 CSV（可选；失败不挡试飞）
  if ! pgrep -f "record_landmark_pose.py" >/dev/null 2>&1; then
    nohup python3 "$HOME/uav_worklog/scripts/record_landmark_pose.py" \
      >"$HOME/uav_worklog/run/record_landmark_pose.log" 2>&1 &
    echo $! >"$PID_DIR/record_landmark_pose.pid"
    disown || true
    echo "[5/5] CSV 记录已后台启动 → records/landmark_pose_gated_*.csv"
  else
    echo "[5/5] CSV 记录已在运行"
  fi
  echo "daemon 模式完成（不占终端）。"
  exit 0
fi

echo "[5/5] 开始记录门控后的 px/py/pz（Ctrl+C 只停记录）..."
python3 "$HOME/uav_worklog/scripts/record_landmark_pose.py"
