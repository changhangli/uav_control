#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
假二维码：向视觉话题发 TargetsInFrameMsg。
可选：前 quiet_sec 秒不发（模拟航线中），之后发码（模拟看见板）。

用法:
  python3 fake_tag.py                  # 立即见码
  python3 fake_tag.py _quiet_sec:=8    # 8 秒后见码，专门测打断时刻
  python3 fake_tag.py _n_markers:=4 _px:=0.20 _py:=-0.10 _pz:=-1.20
"""

from __future__ import print_function
import rospy
from detection_msgs.msg import TargetsInFrameMsg, TargetMsg


def main():
    rospy.init_node("fake_tag")
    uav_id = rospy.get_param("~uav_id", 1)
    uav_name = rospy.get_param("~uav_name", "uav")
    quiet_sec = float(rospy.get_param("~quiet_sec", 0.0))
    rate_hz = float(rospy.get_param("~rate", 10.0))
    n_markers = int(rospy.get_param("~n_markers", 4))
    px = float(rospy.get_param("~px", 0.25))
    py = float(rospy.get_param("~py", -0.15))
    pz = float(rospy.get_param("~pz", -1.20))
    yaw = float(rospy.get_param("~yaw", 10.0))
    # 可选：见码后逐渐把水平偏差收到 0，模拟对准过程
    converge = bool(rospy.get_param("~converge", True))

    topic = "/{}{}/sunray_detect/qrcode_detection_ros".format(uav_name, uav_id)
    pub = rospy.Publisher(topic, TargetsInFrameMsg, queue_size=1)
    rate = rospy.Rate(rate_hz)
    t0 = rospy.Time.now()
    engaged = False

    rospy.logwarn("[FAKE_TAG] -> %s quiet=%.1fs n=%d", topic, quiet_sec, n_markers)
    print("[FAKE_TAG] 前 %.1fs 静默，之后发假码。listen 应立刻 pause_demo。" % quiet_sec)

    while not rospy.is_shutdown():
        age = (rospy.Time.now() - t0).to_sec()
        if age < quiet_sec:
            if int(age * 2) % 2 == 0:
                print("[FAKE_TAG] waiting... %.1fs / %.1fs" % (age, quiet_sec))
            rate.sleep()
            continue

        if not engaged:
            engaged = True
            print("[FAKE_TAG] >>> 开始发码（模拟看见二维码）")

        # 见码后 5s 内把 px/py 收到接近 0
        k = 1.0
        if converge:
            k = max(0.05, 1.0 - (age - quiet_sec) / 5.0)

        tgt = TargetMsg()
        tgt.category_id = n_markers
        tgt.tracked_id = n_markers
        tgt.px = px * k
        tgt.py = py * k
        tgt.pz = pz
        tgt.yaw = yaw * k
        tgt.score = 1.0

        msg = TargetsInFrameMsg()
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = "fake_cam"
        msg.frame_id = int(age * rate_hz)
        msg.width = 1280
        msg.height = 720
        msg.fps = rate_hz
        msg.targets = [tgt]
        pub.publish(msg)
        rate.sleep()


if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass
