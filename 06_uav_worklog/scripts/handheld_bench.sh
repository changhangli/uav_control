#!/usr/bin/env bash
# 手持 bench：一共就 3 个命令，测之前/测之后各敲一次，中间不用敲。
#
#   bash ~/uav_worklog/scripts/handheld_bench.sh start   # 拿起飞机之前
#   bash ~/uav_worklog/scripts/handheld_bench.sh watch   # 可选，第二终端盯着看
#   bash ~/uav_worklog/scripts/handheld_bench.sh stop    # 放下飞机之后
#
# 只有一个终端也行：start 后看 ~/uav_worklog/run/handheld_now.txt（另开文件管理器/第二屏）

set -e

PID_DIR="$HOME/uav_worklog/run"
mkdir -p "$PID_DIR"

cmd="${1:-}"

case "$cmd" in
  start)
    echo ""
    echo "============================================"
    echo "  手持测试 · 启动（请勿解锁/起桨）"
    echo "============================================"
    echo ""

    if pgrep -f "auto_land_by_pose" >/dev/null 2>&1; then
      echo "已在运行。若要重来请先: bash ~/uav_worklog/scripts/handheld_bench.sh stop"
      exit 1
    fi

    bash "$HOME/uav_worklog/scripts/start_listen_hover.sh" --bench

    if ! pgrep -f "handheld_watch.py" >/dev/null 2>&1; then
      nohup bash "$HOME/uav_worklog/scripts/handheld_watch.sh" \
        >"$PID_DIR/handheld_watch.out" 2>&1 &
      echo $! >"$PID_DIR/handheld_watch.pid"
      disown || true
    fi

    sleep 2
    if ! pgrep -f "handheld_watch.py" >/dev/null 2>&1; then
      echo ""
      echo "  [错误] 白话提示没起来，看: $PID_DIR/handheld_watch.out"
      tail -5 "$PID_DIR/handheld_watch.out" 2>/dev/null || true
      exit 1
    fi
    # 先写一屏占位，避免文件一直是空的
    cat >"$PID_DIR/handheld_now.txt" <<'EOF'
======== 手持测试（看这一屏就行）========
① 把相机对准二维码板（还没认到码）
   动作：慢慢移，别抖太快
   状态：等认码…
更新 --:--:--
测完放下飞机 → bash ~/uav_worklog/scripts/handheld_bench.sh stop

EOF
    echo ""
    echo "============================================"
    echo "  ✓ 好了，现在可以拿起飞机对准二维码"
    echo "============================================"
    echo ""
    echo "  看状态（二选一，测的时候不用敲命令）："
    echo "    · 本机第二终端: bash ~/uav_worklog/scripts/handheld_bench.sh watch"
    echo "    · 文件路径: $PID_DIR/handheld_now.txt"
    echo "      （SSH 时用 cat 或 tail -f 看这个文件；不会自动弹窗）"
    echo ""
    echo "  原始状态话题（可选）:"
    echo "    rostopic echo /uav1/listen_status"
    echo ""
    echo "  测完放下飞机后只敲这一条:"
    echo "    bash ~/uav_worklog/scripts/handheld_bench.sh stop"
    echo ""
    ;;

  watch)
    echo "盯着这一屏即可（Ctrl+C 只停显示，不杀后台）"
    echo ""
    touch "$PID_DIR/handheld_now.txt"
    tail -f "$PID_DIR/handheld_now.txt"
    ;;

  stop)
    echo "停止手持测试…"
    if [ -f "$PID_DIR/handheld_watch.pid" ]; then
      pid=$(cat "$PID_DIR/handheld_watch.pid")
      kill "$pid" 2>/dev/null || true
      rm -f "$PID_DIR/handheld_watch.pid"
    fi
    pkill -f "handheld_watch.py" 2>/dev/null || true
    bash "$HOME/uav_worklog/scripts/stop_land_stack.sh"
    echo ""
    echo "已停止。下次测前: bash ~/uav_worklog/scripts/handheld_bench.sh start"
    ;;

  *)
    echo "用法:"
    echo "  bash ~/uav_worklog/scripts/handheld_bench.sh start   # 测前"
    echo "  bash ~/uav_worklog/scripts/handheld_bench.sh watch   # 可选，看屏幕"
    echo "  bash ~/uav_worklog/scripts/handheld_bench.sh stop    # 测后"
    exit 1
    ;;
esac
