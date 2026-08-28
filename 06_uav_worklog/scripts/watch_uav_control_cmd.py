#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""旁路监听 /uav1/uav_control_cmd（默认 1Hz，不刷屏）。

用法：
  source /opt/ros/noetic/setup.bash
  source ~/catkin_ws/devel/setup.bash
  source ~/land_ws/devel/setup.bash
  python3 ~/uav_worklog/scripts/watch_uav_control_cmd.py          # 1Hz
  python3 ~/uav_worklog/scripts/watch_uav_control_cmd.py --fast   # 每条都打（会刷屏）
"""

from __future__ import print_function

import sys
import time

import rospy
from swarm_msgs.msg import UAVControlCMD

CMD_NAMES = {
    1: "XyzPos",
    2: "XyzVel",
    3: "XyVelZPos",
    4: "XyzPosYaw",
    14: "XyzPosYawBody",
    15: "XyzVelYawBody",
    100: "Takeoff",
    101: "Land",
    102: "Hover",
}

FAST = "--fast" in sys.argv
_last_print = 0.0
_latest = None
_count = 0


def cb(msg):
    global _last_print, _latest, _count
    _latest = msg
    _count += 1
    if FAST:
        _print_msg(msg, _count)
        return
    now = time.time()
    if now - _last_print >= 1.0:
        _last_print = now
        _print_msg(msg, _count)


def _print_msg(msg, n):
    name = CMD_NAMES.get(msg.cmd, "CMD(%d)" % msg.cmd)
    print(
        "[LISTEN] #%d cmd=%s(%d) pos=[%+.3f %+.3f %+.3f] vel=[%+.3f %+.3f %+.3f] yaw=%.3f"
        % (
            n,
            name,
            msg.cmd,
            msg.desired_pos[0],
            msg.desired_pos[1],
            msg.desired_pos[2],
            msg.desired_vel[0],
            msg.desired_vel[1],
            msg.desired_vel[2],
            msg.desired_yaw,
        )
    )


def main():
    topic = "/uav1/uav_control_cmd"
    rospy.init_node("watch_uav_control_cmd", anonymous=True)
    rospy.Subscriber(topic, UAVControlCMD, cb, queue_size=20)
    print("listening %s ... (%s)" % (topic, "FAST" if FAST else "1Hz slow"))
    rospy.spin()


if __name__ == "__main__":
    main()
