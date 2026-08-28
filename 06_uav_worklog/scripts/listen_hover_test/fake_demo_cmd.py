#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
假航线：持续向 /uav1/demo_control_cmd 发 UAVControlCMD。
用于 Stage A：验证 vision_cmd_mux 能否在 pause_demo=true 时打断航线。

用法（Mini-Pc）:
  source /opt/ros/noetic/setup.bash
  source ~/land_ws/devel/setup.bash
  python3 ~/uav_worklog/scripts/listen_hover_test/fake_demo_cmd.py
"""

from __future__ import print_function
import math
import rospy
from swarm_msgs.msg import UAVControlCMD


def main():
    rospy.init_node("fake_demo_cmd")
    uav_id = rospy.get_param("~uav_id", 1)
    uav_name = rospy.get_param("~uav_name", "uav")
    rate_hz = float(rospy.get_param("~rate", 20.0))
    amp = float(rospy.get_param("~amp", 0.5))  # 假方形幅度，仅作指纹
    prefix = "/{}{}".format(uav_name, uav_id)
    topic = prefix + "/demo_control_cmd"

    pub = rospy.Publisher(topic, UAVControlCMD, queue_size=1)
    rate = rospy.Rate(rate_hz)
    t0 = rospy.Time.now()
    seq = 0

    rospy.logwarn("[FAKE_DEMO] publishing to %s @ %.0fHz (amp=%.2f)", topic, rate_hz, amp)
    print("[FAKE_DEMO] 假航线已开。见码后应被 mux 挡住，出口不再是这条指令。")

    while not rospy.is_shutdown():
        t = (rospy.Time.now() - t0).to_sec()
        # 缓慢变化的“方形味”位置指令，方便和视觉指令区分
        phase = int(t / 2.0) % 4
        if phase == 0:
            x, y = amp, 0.0
        elif phase == 1:
            x, y = amp, amp
        elif phase == 2:
            x, y = 0.0, amp
        else:
            x, y = 0.0, 0.0

        msg = UAVControlCMD()
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = "fake_demo"
        msg.cmd = UAVControlCMD.XyzPosYaw  # 4，真飞航线常用
        msg.desired_pos = [x, y, 1.5]
        msg.desired_vel = [0.0, 0.0, 0.0]
        msg.desired_acc = [0.0, 0.0, 0.0]
        msg.desired_yaw = 0.1 * math.sin(t)  # 非零指纹，区别于视觉常发的小步长
        pub.publish(msg)

        seq += 1
        if seq % int(rate_hz) == 0:
            print("[FAKE_DEMO] still sending corner=%d pos=[%.2f %.2f 1.50] yaw=%.3f"
                  % (phase, x, y, msg.desired_yaw))
        rate.sleep()


if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass
