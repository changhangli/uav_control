#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""降落状态提示：指令有没有发出、飞机有没有照做。

只看：
  tail -f ~/uav_worklog/run/land_tips.log

或直接跑：
  python3 ~/uav_worklog/scripts/watch_land_status.py
"""

from __future__ import print_function

import math
import os
import time
from collections import deque

import rospy
from swarm_msgs.msg import UAVControlCMD
from detection_msgs.msg import TargetsInFrameMsg

try:
    from geometry_msgs.msg import PoseStamped
except Exception:
    PoseStamped = None

try:
    from mavros_msgs.msg import PositionTarget
except Exception:
    PositionTarget = None

TIP_FILE = os.path.expanduser("~/uav_worklog/run/land_tips.log")


class State(object):
    def __init__(self):
        self.cmd = None
        self.cmd_t = 0.0
        self.has_tag = False
        self.tag_t = 0.0
        self.n_markers = 0
        self.pz = 0.0
        # 飞机实际位置历史 (t,x,y,z)
        self.pose_hist = deque(maxlen=40)
        self.pose_t = 0.0
        # 最新 setpoint
        self.sp = None  # (x,y,z)
        self.sp_t = 0.0
        self.sp_hist = deque(maxlen=40)

    def on_cmd(self, msg):
        self.cmd = msg
        self.cmd_t = time.time()

    def on_tag(self, msg):
        if not msg.targets:
            self.has_tag = False
            return
        t = msg.targets[0]
        self.has_tag = True
        self.tag_t = time.time()
        self.n_markers = int(t.category_id)
        self.pz = t.pz

    def on_pose(self, msg):
        p = msg.pose.position
        now = time.time()
        self.pose_t = now
        self.pose_hist.append((now, p.x, p.y, p.z))

    def on_sp(self, msg):
        # PositionTarget: position may be nan if masked; still try
        now = time.time()
        x, y, z = msg.position.x, msg.position.y, msg.position.z
        if any(math.isnan(v) for v in (x, y, z)):
            return
        self.sp = (x, y, z)
        self.sp_t = now
        self.sp_hist.append((now, x, y, z))


def _move_in_window(hist, sec=2.0):
    """返回 (dx,dy,dz, path_xy) 在最近 sec 秒内。"""
    if len(hist) < 2:
        return 0.0, 0.0, 0.0, 0.0
    now = hist[-1][0]
    # 找约 sec 秒前的点
    old = hist[0]
    for item in hist:
        if now - item[0] <= sec:
            old = item
            break
    dx = hist[-1][1] - old[1]
    dy = hist[-1][2] - old[2]
    dz = hist[-1][3] - old[3]
    path = 0.0
    prev = None
    for item in hist:
        if now - item[0] > sec:
            continue
        if prev is not None:
            path += math.hypot(item[1] - prev[1], item[2] - prev[2])
        prev = item
    return dx, dy, dz, path


def tip_23(st, now):
    """返回 (短标签, 指令状态提示, 是否为我们的纠偏/下降指令)."""
    our = False
    if st.cmd is None or (now - st.cmd_t) > 2.0:
        if st.has_tag and (now - st.tag_t) < 1.0:
            return "等待发令", "指令：未发出（看得到码，但降落程序还没开始发）", False
        return "无指令", "指令：未发出", False

    m = st.cmd
    x, y, z = m.desired_pos[0], m.desired_pos[1], m.desired_pos[2]
    vx, vy, vz = m.desired_vel[0], m.desired_vel[1], m.desired_vel[2]

    if m.cmd == 4 and abs(x) < 5 and abs(y) < 5:
        return "惯性纠偏", "指令：正常（惯性系对准）", True
    if m.cmd in (1,) or (m.cmd == 14 and (abs(x) > 0.8 or abs(y) > 0.8)):
        return "被覆盖", "指令：被别的程序覆盖（有大范围航点在发，请先停掉）", False
    if m.cmd == 15 and (abs(vx) > 0.35 or abs(vy) > 0.35):
        return "被覆盖", "指令：被别的程序覆盖（有速度点动在发，请先停掉）", False
    if m.cmd == 102:
        return "悬停", "指令：悬停", False

    if m.cmd == 14:
        our = True
        ask_xy = (abs(x) ** 2 + abs(y) ** 2) ** 0.5
        if abs(z) < 1e-3:
            if ask_xy > 0.03:
                return "只纠偏", "指令：正常（只水平纠偏，不下降）", True
            return "保高", "指令：正常（保高）", True
        if ask_xy > 0.03 and z < 0:
            return "纠偏下降", "指令：正常（纠偏 + 慢降）", True
        if z > 0.05:
            return "抬高", "指令：正常（抬高）", True
        if z < -0.05:
            return "下降", "指令：正常（下降）", True
        return "位置指令", "指令：正常", True

    if m.cmd == 15 and abs(vx) < 0.05 and abs(vy) < 0.05 and vz < -0.1:
        return "最后下降", "指令：正常（最后垂直下降）", True

    if m.cmd == 101:
        return "落地", "指令：落地", False
    if m.cmd == 100:
        return "起飞", "指令：起飞", False

    return "其它", "指令：未知（cmd=%d）" % m.cmd, False


def tip_34(st, now, our_active):
    """飞机有没有照做：要看设定点是否在改，以及是否朝设定方向走。"""
    has_pose = (now - st.pose_t) < 1.5 and len(st.pose_hist) >= 3
    has_sp = (now - st.sp_t) < 1.5 and st.sp is not None

    if not has_pose:
        return "飞机：看不出来（没有位置数据，手持时正常）"

    dx, dy, dz, path = _move_in_window(st.pose_hist, 2.0)
    moved_xy = math.hypot(dx, dy)
    # 摇摆特征：走了不少路但净位移很小
    wobble_like = path > 0.04 and moved_xy < 0.02

    if not our_active:
        if wobble_like:
            return "飞机：在轻微晃动，当前不是我们的指令"
        if moved_xy > 0.04 or path > 0.05:
            return "飞机：在动，但当前不是我们的指令"
        return "飞机：基本没动，当前也没有我们的指令"

    m = st.cmd
    ask_xy = 0.0
    ask_down = False
    ask_up = False
    cmd_dx = cmd_dy = 0.0
    if m is not None and m.cmd == 14:
        cmd_dx, cmd_dy = m.desired_pos[0], m.desired_pos[1]
        ask_xy = math.hypot(cmd_dx, cmd_dy)
        if m.desired_pos[2] < -0.05:
            ask_down = True
        if m.desired_pos[2] > 0.05:
            ask_up = True
    if m is not None and m.cmd == 15 and m.desired_vel[2] < -0.1:
        ask_down = True

    # 设定点 2 秒内有没有被改（晃机不会单独改 setpoint）
    sp_net = 0.0
    sp_path = 0.0
    if has_sp and len(st.sp_hist) >= 2:
        sdx, sdy, sdz, sp_path = _move_in_window(st.sp_hist, 2.0)
        sp_net = math.hypot(sdx, sdy)

    if ask_down:
        if dz < -0.03:
            return "飞机：正常（在下降）"
        return "飞机：未生效（发了下降，高度没变）"

    if ask_up:
        if dz > 0.03:
            return "飞机：正常（在上升）"
        return "飞机：未生效（发了抬高，高度没变）"

    # 水平对准：要同时确认设定点在改、且飞机朝那个方向挪，才算生效

    if ask_xy < 0.03:
        return "飞机：不用判断（水平指令已经很小）"

    if not has_sp:
        return "飞机：看不出来（没有 setpoint 数据，请确认 mavros 已开）"

    if sp_net < 0.015 and sp_path < 0.02:
        if wobble_like or moved_xy > 0.01:
            return "飞机：不算生效（只是悬停晃动，设定点没改）"
        return "飞机：未生效（有对准指令，但控制栈没转发出去）"

    # 设定点在改：再看实际运动方向是否和「指令 xy」一致
    dir_ok = False
    if moved_xy > 0.02 and ask_xy > 1e-6:
        # 实际位移 · 指令方向
        cosang = (dx * cmd_dx + dy * cmd_dy) / (moved_xy * ask_xy + 1e-9)
        dir_ok = cosang > 0.3  # 夹角别太离谱

    # 或：朝当前 setpoint 靠近
    toward_sp = False
    if has_sp and len(st.pose_hist) >= 2 and moved_xy > 0.02:
        px, py = st.pose_hist[-1][1], st.pose_hist[-1][2]
        sx, sy = st.sp[0], st.sp[1]
        # 2 秒前到现在，与 setpoint 的距离是否变小
        old = st.pose_hist[0]
        for item in st.pose_hist:
            if now - item[0] <= 2.0:
                old = item
                break
        dist_old = math.hypot(old[1] - sx, old[2] - sy)
        dist_new = math.hypot(px - sx, py - sy)
        toward_sp = dist_new < dist_old - 0.015

    if (dir_ok or toward_sp) and (moved_xy >= 0.025 or path >= 0.03):
        return "飞机：正常（在朝对准方向挪）"

    if sp_net >= 0.015 and moved_xy < 0.015:
        return "飞机：未生效（控制栈已转发，但飞机几乎不动）"

    if wobble_like and not dir_ok:
        return "飞机：不算生效（只是来回晃，方向对不上）"

    return "飞机：观察中（再等一两秒看是否持续朝目标挪）"


def main():
    os.makedirs(os.path.dirname(TIP_FILE), exist_ok=True)
    rospy.init_node("watch_land_tips", anonymous=True)
    st = State()
    rospy.Subscriber("/uav1/uav_control_cmd", UAVControlCMD, st.on_cmd, queue_size=20)
    rospy.Subscriber(
        "/uav1/sunray_detect/qrcode_detection_ros",
        TargetsInFrameMsg,
        st.on_tag,
        queue_size=1,
    )
    if PoseStamped is not None:
        rospy.Subscriber(
            "/uav1/mavros/local_position/pose", PoseStamped, st.on_pose, queue_size=20
        )
    if PositionTarget is not None:
        rospy.Subscriber(
            "/uav1/mavros/setpoint_raw/local", PositionTarget, st.on_sp, queue_size=20
        )

    print("")
    print("======== 降落状态提示 ========")
    print("只看: tail -f %s" % TIP_FILE)
    print("==============================")
    print("")

    rate = rospy.Rate(1.0)
    while not rospy.is_shutdown():
        now = time.time()
        short, t23, our = tip_23(st, now)
        t34 = tip_34(st, now, our)

        tag_info = ""
        if st.has_tag and (now - st.tag_t) < 1.0:
            tag_info = "码=%d 距约%.2fm" % (st.n_markers, abs(st.pz))
        else:
            tag_info = "当前没看到码"

        lines = [
            "[%s] %s" % (short, tag_info),
            t23,
            t34,
            "更新: %s" % time.strftime("%H:%M:%S"),
            "",
        ]
        text = "\n".join(lines)
        print(text)
        try:
            with open(TIP_FILE, "w") as f:
                f.write(text)
        except Exception:
            pass
        rate.sleep()


if __name__ == "__main__":
    main()
