#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""订阅 land_ws 的 /uav1/listen_status，写 handheld_now.txt 大白话。"""

from __future__ import print_function

import os
import time

import rospy
from std_msgs.msg import String, Bool

NOW_FILE = os.path.expanduser("~/uav_worklog/run/handheld_now.txt")
STATUS_TOPIC = "/uav1/listen_status"
PAUSE_TOPIC = "/uav1/pause_demo"


def parse_status(raw):
    d = {}
    for part in raw.split(";"):
        if "=" in part:
            k, v = part.split("=", 1)
            d[k.strip()] = v.strip()
    return d


def phase_to_lines(phase, px, py, pz, nm, pause, bench, armed):
    ax, ay = abs(px), abs(py)
    dist = abs(pz)

    if phase == "land_warmup":
        return (
            "④ 视觉预热中（见码了，几帧后进对准）",
            "   偏差：左右%.0fcm  前后%.0fcm  码%d个" % (ax * 100, ay * 100, nm),
            "   保持对准板子",
        )
    if phase == "home_takeover":
        return (
            "① home_land：接管段（bench 不杀 demo）",
            "   下一步：模拟 goto 原点 (0,0,1)",
            "   看 log: [KILL-DEMO] bench skip",
        )
    if phase == "home_goto":
        return (
            "② 正在 goto 原点上方（bench 约 2s 自动进 dwell）",
            "   真飞需 mocap 到位；手持只需等计时",
            "   phase=home_goto",
        )
    if phase == "home_dwell":
        return (
            "③ 原点悬停中（约 2.5s）",
            "   之后进入视觉对准/下降",
            "   请把相机对准二维码板",
        )
    if phase.startswith("sparse_align") or phase.startswith("align_xy"):
        return (
            "④ 视觉降落：水平对准中",
            "   偏差：左右%.0fcm  前后%.0fcm  码%d个" % (ax * 100, ay * 100, nm),
            "   bench 不发 Land，可看 [TX] 指令",
        )
    if phase.startswith("fine_descend") or phase.startswith("align_descend"):
        return (
            "④ 视觉降落：对准 OK，慢降中",
            "   偏差：左右%.0fcm  前后%.0fcm  码%d个" % (ax * 100, ay * 100, nm),
            "   bench 到 FINAL 也会 skip Land",
        )
    if phase.startswith("final"):
        return (
            "⑤ FINAL 下降段（bench 跳过 Land）",
            "   码%d个 | 高度约 %.0fcm" % (nm, dist * 100),
            "   链路验完可 stop",
        )
    if phase in ("lost_tag/holdz", "tag_gap/holdz"):
        return (
            "④ 码丢了，保高等待",
            "   上次：左右%.0fcm  前后%.0fcm  码%d个" % (ax * 100, ay * 100, nm),
            "   移回板子上方继续",
        )
    if phase == "listen_idle":
        return (
            "① 还没见码，航线在飞（或等你对准板子）",
            "   动作：飞过二维码上方或 handheld 对准",
            "   pause=0  demo 指令在走",
        )
    if phase == "listen_warmup":
        return (
            "② 见码了，预热中（3帧后才开始动）",
            "   偏差：左右%.0fcm  前后%.0fcm  码%d个" % (ax * 100, ay * 100, nm),
            "   请保持稳，别抖",
        )
    if phase == "listen_hover/centered":
        return (
            "③ 对准完成！" + ("（bench 可放下）" if bench else "（真飞悬停中）"),
            "   偏差：左右%.0fcm  前后%.0fcm  码%d个" % (ax * 100, ay * 100, nm),
            "   高度约 %.0fm | armed=%s pause=%s" % (dist, armed, pause),
        )
    if phase == "listen_descend/for_markers":
        return (
            "② 见码了，正在慢降认更多码",
            "   偏差：左右%.0fcm  前后%.0fcm  码%d个" % (ax * 100, ay * 100, nm),
            "   目标：码≥4 或高度够近",
        )
    if phase == "listen_gap/holdz":
        return (
            "② 码暂时丢了，保高等待（航线仍停）",
            "   上次：左右%.0fcm  前后%.0fcm  码%d个" % (ax * 100, ay * 100, nm),
            "   慢慢移回板子上方",
        )
    if phase == "listen_approach/holdz":
        return (
            "② 粗对准好了，保高微调",
            "   偏差：左右%.0fcm  前后%.0fcm  码%d个" % (ax * 100, ay * 100, nm),
            "   目标：两个方向 <8cm",
        )
    if phase.startswith("listen_approach"):
        return (
            "② 见码了，正在对准中心",
            "   偏差：左右%.0fcm  前后%.0fcm  码%d个" % (ax * 100, ay * 100, nm),
            "   目标：两个方向 <8cm",
        )
    if pause:
        return (
            "② 视觉已接管（pause=1）",
            "   phase=%s" % (phase or "?"),
            "   偏差：左右%.0fcm  前后%.0fcm  码%d个" % (ax * 100, ay * 100, nm),
        )
    return (
        "① 等待 land_ws 状态…",
        "   请确认 listen 节点在跑",
        "   调试: rostopic echo %s" % STATUS_TOPIC,
    )


class S(object):
    def __init__(self):
        self.raw = ""
        self.status_t = 0.0
        self.pause = False


def main():
    os.makedirs(os.path.dirname(NOW_FILE), exist_ok=True)
    rospy.init_node("handheld_watch", anonymous=True)
    st = S()

    def on_status(msg):
        st.raw = msg.data
        st.status_t = time.time()

    def on_pause(msg):
        st.pause = bool(msg.data)

    rospy.Subscriber(STATUS_TOPIC, String, on_status, queue_size=10)
    rospy.Subscriber(PAUSE_TOPIC, Bool, on_pause, queue_size=5)

    print("订阅 %s → %s" % (STATUS_TOPIC, NOW_FILE))
    rate = rospy.Rate(2.0)
    while not rospy.is_shutdown():
        now = time.time()
        if st.raw and (now - st.status_t) < 5.0:
            d = parse_status(st.raw)
            phase = d.get("phase", "")
            px = float(d.get("px", 0))
            py = float(d.get("py", 0))
            pz = float(d.get("pz", 0))
            nm = int(float(d.get("nm", 0)))
            pause = d.get("pause", "0") == "1" or st.pause
            bench = d.get("bench", "0") == "1"
            armed = d.get("armed", "?")
            line1, line2, line3 = phase_to_lines(phase, px, py, pz, nm, pause, bench, armed)
            src = "listen_status"
        else:
            line1 = "① 还没收到 listen_status"
            line2 = "   先: handheld_bench.sh 或 handheld_home_bench.sh start"
            line3 = "   或: rostopic echo %s" % STATUS_TOPIC
            src = "waiting"

        text = "\n".join(
            [
                "======== 手持/真飞状态（%s）========" % src,
                line1,
                line2,
                line3,
                "更新 %s" % time.strftime("%H:%M:%S"),
                "测完 → handheld_bench.sh stop 或 handheld_home_bench.sh stop",
                "",
            ]
        )
        print(text)
        try:
            with open(NOW_FILE, "w") as f:
                f.write(text)
        except Exception:
            pass
        rate.sleep()


if __name__ == "__main__":
    main()
