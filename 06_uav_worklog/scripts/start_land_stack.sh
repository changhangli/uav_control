#!/usr/bin/env bash
# 一键起降落相关栈：视觉/门控/CSV/录 bag/提示词监视 → 全部后台
# 本终端只负责：等老师起飞悬停后，启动 auto_land_by_pose
#
# 用法：
#   bash ~/uav_worklog/scripts/start_land_stack.sh
#   bash ~/uav_worklog/scripts/start_land_stack.sh --no-bag bench_mode:=true
#   bash ~/uav_worklog/scripts/start_land_stack.sh --no-land
#
# 停止：
#   bash ~/uav_worklog/scripts/stop_land_stack.sh
#
# ★ 另开终端只看提示词：
#   tail -f ~/uav_worklog/run/land_tips.log

set -e

START_LAND=1
START_BAG=1
START_WATCH=1
LAND_ARGS=(auto_takeoff:=false)

for a in "$@"; do
  case "$a" in
    --no-land) START_LAND=0 ;;
    --no-bag) START_BAG=0 ;;
    --no-watch) START_WATCH=0 ;;
    --help|-h)
      sed -n '2,16p' "$0"
      exit 0
      ;;
    *)
      LAND_ARGS+=("$a")
      ;;
  esac
done

PID_DIR="$HOME/uav_worklog/run"
REC_DIR="$HOME/uav_worklog/records"
mkdir -p "$PID_DIR" "$REC_DIR"

source /opt/ros/noetic/setup.bash
source "$HOME/catkin_ws/devel/setup.bash"
source "$HOME/vision_ws/devel/setup.bash"
source "$HOME/land_ws/devel/setup.bash"

echo "========== [1] 视觉栈（后台） =========="
bash "$HOME/uav_worklog/scripts/start_vision_verify.sh" --daemon

echo
echo "========== [2] 录 bag / 提示词（后台） =========="

if [ "$START_BAG" = "1" ]; then
  if pgrep -f "rosbag record .*land_" >/dev/null 2>&1; then
    echo "  bag 已在录，跳过"
  else
    BAG="$REC_DIR/land_$(date +%Y%m%d_%H%M%S).bag"
    echo "$BAG" >"$PID_DIR/land_bag.path"
    nohup rosbag record -O "$BAG" \
      /uav1/uav_control_cmd \
      /uav1/swarm/uav_state \
      /uav1/sunray_detect/qrcode_detection_ros \
      /uav1/mavros/setpoint_raw/local \
      /uav1/mavros/local_position/pose \
      >"$PID_DIR/land_bag.log" 2>&1 &
    echo $! >"$PID_DIR/land_bag.pid"
    disown || true
    echo "  bag → $BAG"
  fi
else
  echo "  跳过 bag (--no-bag)"
fi

if [ "$START_WATCH" = "1" ]; then
  if pgrep -f "watch_land_status.py" >/dev/null 2>&1; then
    echo "  提示词监视已在运行，跳过"
  else
    nohup python3 "$HOME/uav_worklog/scripts/watch_land_status.py" \
      >"$PID_DIR/watch_land_status.out" 2>&1 &
    echo $! >"$PID_DIR/watch_land_status.pid"
    disown || true
    echo "  提示词已开"
  fi
else
  echo "  跳过监视 (--no-watch)"
fi

echo
echo "========== 支撑已就绪 =========="
echo "  停全部: bash ~/uav_worklog/scripts/stop_land_stack.sh"
echo
echo "  ★★★ 另开一个终端，只看这一句：★★★"
echo "      tail -f ~/uav_worklog/run/land_tips.log"
echo

if [ "$START_LAND" != "1" ]; then
  echo "未启动降落 (--no-land)。手动："
  echo "  roslaunch sunray_tutorial auto_land_by_pose.launch ${LAND_ARGS[*]}"
  exit 0
fi

echo "========== [3] 降落节点 =========="
echo "老师起飞悬停并停航线后，按回车启动 auto_land。"
echo "手持请用: ... bench_mode:=true"
echo "参数: ${LAND_ARGS[*]}"
read -r _

echo "启动 auto_land_by_pose ..."
set +e
roslaunch sunray_tutorial auto_land_by_pose.launch "${LAND_ARGS[@]}"
rc=$?
set -e

echo
echo "降落节点已退出 (rc=$rc)。"
echo "全部关掉: bash ~/uav_worklog/scripts/stop_land_stack.sh"
exit "$rc"
