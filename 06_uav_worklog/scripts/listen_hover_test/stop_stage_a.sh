#!/usr/bin/env bash
pkill -f 'fake_demo_cmd.py' 2>/dev/null || true
pkill -f 'fake_tag.py' 2>/dev/null || true
pkill -f 'watch_interrupt.py' 2>/dev/null || true
pkill -f 'auto_listen_hover.launch' 2>/dev/null || true
pkill -f 'vision_cmd_mux' 2>/dev/null || true
pkill -f 'auto_land_by_pose' 2>/dev/null || true
# optional: leave roscore running for next tests
echo "Stage A processes stopped (roscore left running if any)."
