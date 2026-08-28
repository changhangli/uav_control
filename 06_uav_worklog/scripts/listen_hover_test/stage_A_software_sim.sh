#!/usr/bin/env bash
# Stage A：纯软件打断仿真（不真飞、可不接相机）
# 在 Mini-Pc 上新开一个终端执行本脚本；其它节点用另外的终端按注释开。
set -e
source /opt/ros/noetic/setup.bash
source ~/land_ws/devel/setup.bash
DIR="$(cd "$(dirname "$0")" && pwd)"

echo "=========================================="
echo " Stage A 软件打断仿真"
echo " 先确保下面 3 个终端已开（本脚本只开监视）:"
echo "  T1: roslaunch sunray_tutorial auto_listen_hover.launch bench_mode:=true"
echo "  T2: python3 $DIR/fake_demo_cmd.py"
echo "  T3: python3 $DIR/fake_tag.py _quiet_sec:=8"
echo "=========================================="
python3 "$DIR/watch_interrupt.py"
