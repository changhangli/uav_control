#!/usr/bin/env bash
# 启动脚本接管 — 对接老师地面站图形界面 demo
#
# 正确用法（真飞）：
#   1) 老师地面站：解锁 → 起飞 → 选方/圆/六边航线（图形界面）
#   2) 本机起视觉 + 接管脚本：
#        bash ~/uav_worklog/scripts/start_home_land.sh
#   3) 节点就绪后：Hover → 杀 demo → 回 (0,0,1) → 悬停 2.5s → 视觉自动降落
#
# 手持/台架（不真飞）：
#   bash ~/uav_worklog/scripts/start_home_land.sh --bench

set -e

BENCH=0
for a in "$@"; do
  case "$a" in
    --bench|-b) BENCH=1 ;;
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
  echo "已有 land/mux 在跑，请先: bash ~/uav_worklog/scripts/stop_land_stack.sh"
  exit 1
fi

echo "========== [2] 地面站路径：home_land（杀 demo → 回原点 → 降落） =========="
LAUNCH_ARGS=(home_land_mode:=true listen_hover_mode:=false route_land_mode:=false use_cmd_mux:=false)
[ "$BENCH" = "1" ] && LAUNCH_ARGS+=(bench_mode:=true)

nohup roslaunch sunray_tutorial auto_home_land.launch "${LAUNCH_ARGS[@]}" \
  >"$PID_DIR/home_land.log" 2>&1 &

echo $! >"$PID_DIR/home_land.pid"
disown || true

echo
if [ "$BENCH" = "1" ]; then
  echo "★ 手持 bench：勿解锁起桨；看 [KILL-DEMO]/[HOME] 日志"
else
  echo "真飞顺序："
  echo "  1. 老师地面站图形界面：解锁 → 起飞 → 选航线（方/圆/六边）"
  echo "  2. 本脚本已启动 home_land 接管"
  echo "  3. 节点就绪 → 杀 demo → 飞 (0,0,1) → 悬停 → 视觉降落"
  echo "  成功标志：日志出现 [KILL-DEMO]、[HOME-READY]、[HOME] auto land"
fi
echo
echo "  tail -f $PID_DIR/home_land.log"
echo "  tail -f ~/uav_worklog/run/land_tips.log"
