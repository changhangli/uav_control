#!/usr/bin/env bash
# 手持 bench · home_land 链路验证（不解锁、不起桨）
#
#   bash ~/uav_worklog/scripts/handheld_home_bench.sh start
#   bash ~/uav_worklog/scripts/handheld_home_bench.sh watch   # 可选
#   bash ~/uav_worklog/scripts/handheld_home_bench.sh stop
#
# 验证段：视觉栈 → [HOME-LAND] → [KILL-DEMO] bench skip → goto/dwell → 见码对准/下降（不发 Land）

set -e

PID_DIR="$HOME/uav_worklog/run"
mkdir -p "$PID_DIR"

cmd="${1:-}"

case "$cmd" in
  start)
    echo ""
    echo "============================================"
    echo "  手持 home_land 链路测试（请勿解锁/起桨）"
    echo "============================================"
    echo ""

    if pgrep -f "auto_land_by_pose" >/dev/null 2>&1; then
      echo "已在运行。若要重来请先: bash ~/uav_worklog/scripts/handheld_home_bench.sh stop"
      exit 1
    fi

    bash "$HOME/uav_worklog/scripts/start_home_land.sh" --bench

    sleep 3
    if ! pgrep -f "auto_land_by_pose" >/dev/null 2>&1; then
      echo ""
      echo "  [错误] auto_land 节点未在跑！看: $PID_DIR/home_land.log"
      tail -15 "$PID_DIR/home_land.log" 2>/dev/null || true
      exit 1
    fi

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

    cat >"$PID_DIR/handheld_now.txt" <<'EOF'
======== 手持 home_land 测试 ========
① 节点刚起：等 [HOME-LAND] / [KILL-DEMO] bench skip
   约 2s 后进入 dwell，再对准二维码
更新 --:--:--
测完 → bash ~/uav_worklog/scripts/handheld_home_bench.sh stop

EOF
    echo ""
    echo "  ★ 前 ~5s 是 goto/dwell（与见码无关），之后才对准二维码"
    echo "  看状态：bash ~/uav_worklog/scripts/handheld_home_bench.sh watch"
    echo "  或 tail -f $PID_DIR/handheld_now.txt"
    echo "  原始日志：tail -f $PID_DIR/home_land.log"
    echo ""
    ;;

  watch)
    touch "$PID_DIR/handheld_now.txt"
    tail -f "$PID_DIR/handheld_now.txt"
    ;;

  stop)
    echo "停止 handheld home_land 测试…"
    if [ -f "$PID_DIR/handheld_watch.pid" ]; then
      pid=$(cat "$PID_DIR/handheld_watch.pid")
      kill "$pid" 2>/dev/null || true
      rm -f "$PID_DIR/handheld_watch.pid"
    fi
    pkill -f "handheld_watch.py" 2>/dev/null || true
    bash "$HOME/uav_worklog/scripts/stop_land_stack.sh"
    echo "已停止。"
    ;;

  *)
    echo "用法:"
    echo "  bash ~/uav_worklog/scripts/handheld_home_bench.sh start"
    echo "  bash ~/uav_worklog/scripts/handheld_home_bench.sh watch"
    echo "  bash ~/uav_worklog/scripts/handheld_home_bench.sh stop"
    exit 1
    ;;
esac
