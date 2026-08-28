#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
监视打断是否成功：对比 demo / vision / 出口 uav_control_cmd，以及 pause_demo。

通过标准（软件层）:
  1) pause_demo 从 False -> True
  2) 出口话题内容从“航线指纹”切到“视觉指纹”
  3) pause=true 期间出口不再跟随 demo 的 desired_yaw / frame_id
"""

from __future__ import print_function
import rospy
from std_msgs.msg import Bool, String
from swarm_msgs.msg import UAVControlCMD


class Watcher(object):
    def __init__(self):
        uav_id = rospy.get_param("~uav_id", 1)
        uav_name = rospy.get_param("~uav_name", "uav")
        p = "/{}{}".format(uav_name, uav_id)
        self.pause = False
        self.last_demo = None
        self.last_vision = None
        self.last_out = None
        self.switched = False

        rospy.Subscriber(p + "/pause_demo", Bool, self.on_pause, queue_size=1)
        rospy.Subscriber(p + "/demo_control_cmd", UAVControlCMD, self.on_demo, queue_size=1)
        rospy.Subscriber(p + "/vision_control_cmd", UAVControlCMD, self.on_vision, queue_size=1)
        rospy.Subscriber(p + "/uav_control_cmd", UAVControlCMD, self.on_out, queue_size=1)
        rospy.Subscriber(p + "/listen_status", String, self.on_status, queue_size=1)
        rospy.logwarn("[WATCH] monitoring %s pause/demo/vision/out/listen_status", p)
        print("[WATCH] armed; point camera at landing board when ready")

    def fingerprint(self, msg):
        if msg is None:
            return "none"
        return "cmd=%u pos=[%.2f %.2f %.2f] yaw=%.3f frame=%s" % (
            msg.cmd,
            msg.desired_pos[0], msg.desired_pos[1], msg.desired_pos[2],
            msg.desired_yaw,
            msg.header.frame_id or "-",
        )

    def on_pause(self, msg):
        if msg.data != self.pause:
            print("[WATCH] pause_demo: %s -> %s" % (self.pause, msg.data))
            self.pause = msg.data

    def on_demo(self, msg):
        self.last_demo = msg

    def on_vision(self, msg):
        self.last_vision = msg

    def on_status(self, msg):
        print("[WATCH] listen_status: %s" % msg.data)

    def on_out(self, msg):
        self.last_out = msg
        if not self.pause:
            return
        if self.last_vision is None:
            return
        # pause 期间出口应接近视觉（允许时间戳不同）
        same_cmd = msg.cmd == self.last_vision.cmd
        close_xy = (
            abs(msg.desired_pos[0] - self.last_vision.desired_pos[0]) < 0.05
            and abs(msg.desired_pos[1] - self.last_vision.desired_pos[1]) < 0.05
        )
        demo_still = False
        if self.last_demo is not None:
            # 若出口还在跟假航线的 yaw 指纹，则失败
            demo_still = abs(msg.desired_yaw - self.last_demo.desired_yaw) < 1e-4 and (
                self.last_demo.header.frame_id == "fake_demo"
            )
        if same_cmd and close_xy and not demo_still and not self.switched:
            self.switched = True
            # 用唯一标记，避免和提示语互相误匹配
            print("========================================")
            print("[WATCH] INTERRUPT_PASS")
            print("  pause=True")
            print("  out   :", self.fingerprint(msg))
            print("  vision:", self.fingerprint(self.last_vision))
            print("  demo  :", self.fingerprint(self.last_demo), "(仍在发但未出口)")
            print("========================================")
        elif self.pause and self.last_demo is not None and msg.cmd == self.last_demo.cmd and msg.cmd != (
            self.last_vision.cmd if self.last_vision is not None else -1
        ):
            print("[WATCH] INTERRUPT_FAIL out still follows demo | out=%s" % self.fingerprint(msg))


def main():
    rospy.init_node("watch_interrupt")
    Watcher()
    rospy.spin()


if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass
