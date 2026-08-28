#!/usr/bin/env bash
# 停 Stage B 的 listen/假航线/监视；默认不停视觉（方便接着测）
KEEP_VISION=1
if [ "$1" = "--all" ]; then
  KEEP_VISION=0
fi

pkill -9 -f 'fake_demo_cmd.py' 2>/dev/null || true
pkill -9 -f 'fake_tag.py' 2>/dev/null || true
pkill -9 -f 'watch_interrupt.py' 2>/dev/null || true
pkill -9 -f 'auto_listen_hover.launch' 2>/dev/null || true
pkill -9 -f 'auto_land_by_pose' 2>/dev/null || true
pkill -9 -f 'vision_cmd_mux' 2>/dev/null || true

if [ "$KEEP_VISION" = "0" ]; then
  bash "$HOME/uav_worklog/scripts/stop_vision_verify.sh" || true
  echo "Stage B + vision stopped"
else
  echo "Stage B control nodes stopped; vision left running"
fi
