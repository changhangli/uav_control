#!/usr/bin/env bash
# 随用随听 — 对接老师地面站图形界面（默认无 remap、无 mux）
#
# 正确用法（真飞）：
#   1) 老师地面站：解锁 → 起飞 → 选方/圆/六边航线（图形界面，不要改 catkin）
#   2) 本机只起视觉监听：
#        bash ~/uav_worklog/scripts/start_listen_hover.sh
#   3) 摄像头见码 → 切 control_mode 打断航线 → 飞到码正上方悬停（可略降增码）
#
# 手持/台架（不真飞）：
#   bash ~/uav_worklog/scripts/start_listen_hover.sh --bench
#
# 可选：不经过地面站、自己起 demo（才会 remap，仅调试用）：
#   bash ~/uav_worklog/scripts/start_listen_hover.sh --with-demo 3
#
# 见码后改降落而不是悬停：
#   bash ~/uav_worklog/scripts/start_listen_hover.sh --land

set -e

WITH_DEMO=0
DEMO_ID=3
BENCH=0
LAND=0
for a in "$@"; do
  case "$a" in
    --with-demo) WITH_DEMO=1 ;;
    --with-square) WITH_DEMO=1; DEMO_ID=3 ;;
    --with-demo=*) DEMO_ID="${a#*=}"; WITH_DEMO=1 ;;
    --bench|-b) BENCH=1 ;;
    --land) LAND=1 ;;
  esac
done

PID_DIR="$HOME/uav_worklog/run"
mkdir -p "$PID_DIR"

source /opt/ros/noetic/setup.bash
source "$HOME/catkin_ws/devel/setup.bash"
source "$HOME/vision_ws/devel/setup.bash"
source "$HOME/land_ws/devel/setup.bash"

echo "========== [1] 视觉栈 =========="
bash "$HOME/uav_worklog/scripts/start_vision_verify.sh" --daemon

if ! pgrep -f "watch_land_status.py" >/dev/null 2>&1; then
  nohup python3 "$HOME/uav_worklog/scripts/watch_land_status.py" \
    >"$PID_DIR/watch_land_status.out" 2>&1 &
  echo $! >"$PID_DIR/watch_land_status.pid"
  disown || true
fi

if pgrep -f "auto_land_by_pose" >/dev/null 2>&1 || pgrep -f "vision_cmd_mux" >/dev/null 2>&1; then
  echo "已有 listen/mux 在跑，请先: bash ~/uav_worklog/scripts/stop_land_stack.sh"
  exit 1
fi

if [ "$LAND" = "1" ]; then
  MODE_ARGS=(listen_hover_mode:=false route_land_mode:=true)
  MODE_NAME="见码降落"
else
  MODE_ARGS=(listen_hover_mode:=true route_land_mode:=false)
  MODE_NAME="见码悬停"
fi

if [ "$WITH_DEMO" = "1" ]; then
  case "$DEMO_ID" in
    1) DEMO_NAME="圆形" ;;
    2) DEMO_NAME="六边形" ;;
    3) DEMO_NAME="方形" ;;
    *) echo "demo_id 只能是 1/2/3"; exit 1 ;;
  esac
  echo "========== [2] 【调试】${DEMO_NAME}航线 remap+mux + ${MODE_NAME} =========="
  echo "注意：这不是老师地面站路径；真飞请去掉 --with-demo"
  LAUNCH_ARGS=(demo_id:="$DEMO_ID" "${MODE_ARGS[@]}")
  [ "$BENCH" = "1" ] && LAUNCH_ARGS+=(bench_mode:=true)
  nohup roslaunch sunray_tutorial demo_with_listen.launch "${LAUNCH_ARGS[@]}" \
    >"$PID_DIR/listen_hover.log" 2>&1 &
else
  echo "========== [2] 地面站路径：仅视觉监听 + ${MODE_NAME}（无 remap） =========="
  LAUNCH_ARGS=("${MODE_ARGS[@]}" use_cmd_mux:=false)
  [ "$BENCH" = "1" ] && LAUNCH_ARGS+=(bench_mode:=true)
  nohup roslaunch sunray_tutorial auto_listen_hover.launch "${LAUNCH_ARGS[@]}" \
    >"$PID_DIR/listen_hover.log" 2>&1 &
fi

echo $! >"$PID_DIR/listen_hover.pid"
disown || true

echo
if [ "$BENCH" = "1" ]; then
  echo "★ 手持 bench：勿解锁起桨；对二维码看 [LISTEN]/[INTERRUPT] 日志"
else
  echo "真飞顺序："
  echo "  1. 老师地面站图形界面：解锁 → 起飞 → 选航线"
  echo "  2. 本脚本已在听摄像头（不要再 roslaunch 带 remap 的 demo）"
  if [ "$LAND" = "1" ]; then
    echo "  3. 见码 → 打断航线 → 自动降落"
  else
    echo "  3. 见码 → 打断航线（仿点降落切模式）→ 飞到码正上方悬停"
  fi
  echo "  成功标志：日志出现 [INTERRUPT] 或 [LISTEN] 见码"
fi
echo
echo "  tail -f $PID_DIR/listen_hover.log"
echo "  tail -f ~/uav_worklog/run/land_tips.log"
