#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""记录 landmark_detection RANSAC 融合后的 px/py/pz。

用法：
  source /opt/ros/noetic/setup.bash && source ~/vision_ws/devel/setup.bash
  python3 ~/uav_worklog/scripts/record_landmark_pose.py

输出：
  ~/uav_worklog/records/landmark_pose_YYYYMMDD_HHMMSS.csv

说明（融合后字段含义）：
  n_markers  = category_id  本帧认出的码数
  n_big      = w            用到的大码数
  n_selected = tracked_id   参与融合的码数
  score      = RANSAC误差   越小越好
  px,py,pz   = 板中心相对相机，单位米
"""

from __future__ import print_function

import csv
import datetime
import os
import signal
import sys

import rospy
from detection_msgs.msg import TargetsInFrameMsg


# 默认记门控后的话题；若要记原始结果：
#   python3 record_landmark_pose.py raw
TOPIC_GATED = "/uav1/sunray_detect/landmark_pose_gated"
TOPIC_RAW = "/uav1/sunray_detect/qrcode_detection_ros"
OUT_DIR = os.path.expanduser("~/uav_worklog/records")
# 写入频率上限（Hz），避免 CSV 太大
MAX_HZ = 5.0
# 标记疑似废数据（记 raw 时用；gated 话题本身已过滤）
PZ_MAX_M = 5.0
SCORE_MAX = 0.06
N_MARKERS_MIN = 2


class Recorder(object):
    def __init__(self, path):
        self.path = path
        self.file = open(path, "w", newline="")
        self.writer = csv.writer(self.file)
        self.writer.writerow(
            [
                "time",
                "ros_sec",
                "ros_nsec",
                "seq",
                "n_markers",
                "n_big",
                "n_selected",
                "score",
                "px",
                "py",
                "pz",
                "yaw",
                "flag",
            ]
        )
        self.count = 0
        self.last_write = rospy.Time(0)
        self.min_dt = rospy.Duration(1.0 / MAX_HZ)

    def callback(self, msg):
        now = rospy.Time.now()
        if self.last_write.to_sec() > 0 and (now - self.last_write) < self.min_dt:
            return
        self.last_write = now

        if not msg.targets:
            row_time = datetime.datetime.now().strftime("%H:%M:%S.%f")[:-3]
            self.writer.writerow(
                [
                    row_time,
                    msg.header.stamp.secs,
                    msg.header.stamp.nsecs,
                    msg.header.seq,
                    0,
                    0,
                    0,
                    "",
                    "",
                    "",
                    "",
                    "",
                    "NO_TARGET",
                ]
            )
            self.file.flush()
            self.count += 1
            # 不要刷屏：每秒最多提示一次
            if not hasattr(self, "_last_no_tgt_print") or (
                now - self._last_no_tgt_print
            ).to_sec() >= 1.0:
                print(
                    "[{}] NO_TARGET  （把摄像头对准屏幕上的板；看 image_rect 是否有框）".format(
                        row_time
                    )
                )
                self._last_no_tgt_print = now
            return

        t = msg.targets[0]
        n_markers = int(t.category_id)
        n_big = int(t.w)
        n_selected = int(t.tracked_id)
        score = float(t.score)
        px, py, pz = float(t.px), float(t.py), float(t.pz)
        yaw = float(t.yaw)

        flags = []
        if n_markers < N_MARKERS_MIN:
            flags.append("FEW_MARKERS")
        if score > SCORE_MAX:
            flags.append("HIGH_SCORE")
        if pz < 0 or pz > PZ_MAX_M:
            flags.append("PZ_OUTLIER")
        flag = "|".join(flags) if flags else "OK"

        row_time = datetime.datetime.now().strftime("%H:%M:%S.%f")[:-3]
        self.writer.writerow(
            [
                row_time,
                msg.header.stamp.secs,
                msg.header.stamp.nsecs,
                msg.header.seq,
                n_markers,
                n_big,
                n_selected,
                "{:.6f}".format(score),
                "{:.6f}".format(px),
                "{:.6f}".format(py),
                "{:.6f}".format(pz),
                "{:.3f}".format(yaw),
                flag,
            ]
        )
        self.file.flush()
        self.count += 1

        print(
            "[{}] n={:2d} big={:d} score={:.4f}  px={:+.3f} py={:+.3f} pz={:.3f}  {}".format(
                row_time, n_markers, n_big, score, px, py, pz, flag
            )
        )

    def close(self):
        try:
            self.file.close()
        except Exception:
            pass


def main():
    use_raw = len(sys.argv) > 1 and sys.argv[1].lower() == "raw"
    topic = TOPIC_RAW if use_raw else TOPIC_GATED
    prefix = "landmark_pose_raw_" if use_raw else "landmark_pose_gated_"

    os.makedirs(OUT_DIR, exist_ok=True)
    stamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
    out_path = os.path.join(OUT_DIR, "{}{}.csv".format(prefix, stamp))

    rospy.init_node("record_landmark_pose", anonymous=True)
    rec = Recorder(out_path)

    def _stop(*_args):
        print("\n已保存 {} 行到:\n  {}".format(rec.count, out_path))
        rec.close()
        rospy.signal_shutdown("user stop")

    signal.signal(signal.SIGINT, _stop)

    rospy.Subscriber(topic, TargetsInFrameMsg, rec.callback, queue_size=1)
    print("订阅:", topic, ("(原始)" if use_raw else "(门控后)"))
    print("记录到:", out_path)
    print("Ctrl+C 结束\n")
    if not use_raw:
        print("提示: 若一直无数据，先确认门控节点已启动 landmark_pose_gate.py\n")
    rospy.spin()
    rec.close()
    print("文件:", out_path)
    return 0


if __name__ == "__main__":
    sys.exit(main())
