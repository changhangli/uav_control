#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""降落板位姿门控节点。

订阅 landmark 原始融合结果，过滤离谱帧，少码时平滑后发布。

用法：
  source /opt/ros/noetic/setup.bash && source ~/vision_ws/devel/setup.bash
  python3 ~/uav_worklog/scripts/landmark_pose_gate.py

输出话题：
  /uav1/sunray_detect/landmark_pose_gated   (detection_msgs/TargetsInFrameMsg)
  /uav1/sunray_detect/landmark_pose_gate_status  (std_msgs/String)  简短状态
"""

from __future__ import print_function

import math
import os
import sys

import rospy
from detection_msgs.msg import TargetsInFrameMsg, TargetMsg
from std_msgs.msg import String

try:
    import yaml
except ImportError:
    yaml = None


DEFAULT_CFG = os.path.expanduser("~/uav_worklog/config/landmark_pose_gate.yaml")


def load_cfg(path):
    cfg = {
        "topic_in": "/uav1/sunray_detect/qrcode_detection_ros",
        "topic_out": "/uav1/sunray_detect/landmark_pose_gated",
        "min_markers": 2,
        "require_big_marker": True,
        "max_score": 0.02,
        "max_score_sparse": 0.05,
        "sparse_marker_threshold": 4,
        "pz_min": 0.05,
        "pz_max": 5.0,
        "max_jump_xy": 0.25,
        "max_jump_z": 0.40,
        "max_jump_xy_sparse": 0.18,
        "max_jump_z_sparse": 0.30,
        "max_jump_yaw": 40.0,
        "max_jump_yaw_sparse": 25.0,
        "lost_timeout": 0.6,
        "smooth_enable": True,
        "smooth_alpha_dense": 0.40,
        "smooth_alpha_sparse": 0.12,
        "smooth_alpha_yaw_dense": 0.35,
        "smooth_alpha_yaw_sparse": 0.08,
        "hold_on_reject": True,
        "invert_py": False,
        "body_offset_x": 0.0,
        "body_offset_y": 0.0,
        "body_offset_z": 0.0,
    }
    if not os.path.exists(path):
        return cfg
    with open(path, "r") as f:
        text = f.read()
    if yaml is not None:
        data = yaml.safe_load(text) or {}
    else:
        data = {}
        for line in text.splitlines():
            line = line.split("#", 1)[0].strip()
            if not line or ":" not in line:
                continue
            k, v = line.split(":", 1)
            k, v = k.strip(), v.strip()
            if v.lower() in ("true", "false"):
                data[k] = v.lower() == "true"
            else:
                try:
                    data[k] = float(v) if "." in v else int(v)
                except ValueError:
                    data[k] = v.strip().strip("'\"")
    cfg.update(data)
    return cfg


def _wrap_deg(d):
    """把角度差折到 (-180, 180]."""
    return (d + 180.0) % 360.0 - 180.0


def _lerp_angle(a, b, alpha):
    return a + alpha * _wrap_deg(b - a)


class PoseGate(object):
    def __init__(self, cfg):
        self.cfg = cfg
        # 上一帧通过门控的原始值（用于跳变检测）
        self.last_raw = None  # (px, py, pz, yaw, stamp)
        # 平滑状态（发布用）
        self.smooth = None  # (px, py, pz, yaw)
        self.last_pub_meta = None  # 最近一次发布用的 n/big/score 等
        self.pub = rospy.Publisher(cfg["topic_out"], TargetsInFrameMsg, queue_size=1)
        self.pub_status = rospy.Publisher(
            "/uav1/sunray_detect/landmark_pose_gate_status", String, queue_size=10
        )
        self.stats = {
            "in": 0,
            "pass": 0,
            "hold": 0,
            "reject_empty": 0,
            "reject_markers": 0,
            "reject_big": 0,
            "reject_score": 0,
            "reject_pz": 0,
            "reject_jump": 0,
        }
        rospy.Subscriber(cfg["topic_in"], TargetsInFrameMsg, self.callback, queue_size=1)
        rospy.Timer(rospy.Duration(2.0), self._print_stats)

    def _status(self, text):
        self.pub_status.publish(String(data=text))

    def _print_stats(self, _evt):
        s = self.stats
        total = max(s["in"], 1)
        rospy.loginfo_throttle(
            2.0,
            "gate: in=%d pass=%d hold=%d (pass %.0f%%) empty=%d few=%d big=%d score=%d pz=%d jump=%d"
            % (
                s["in"],
                s["pass"],
                s["hold"],
                100.0 * s["pass"] / total,
                s["reject_empty"],
                s["reject_markers"],
                s["reject_big"],
                s["reject_score"],
                s["reject_pz"],
                s["reject_jump"],
            ),
        )

    def callback(self, msg):
        self.stats["in"] += 1
        reason = self._check(msg)
        if reason is not None:
            self.stats[reason] += 1
            if self._maybe_hold(msg, reason):
                return
            self._status("REJECT:" + reason)
            return

        t = msg.targets[0]
        n_markers = int(t.category_id)
        px_raw, py_raw, pz_raw = float(t.px), float(t.py), float(t.pz)
        yaw_raw = float(t.yaw)
        stamp = msg.header.stamp if msg.header.stamp.to_sec() > 0 else rospy.Time.now()

        px_s, py_s, pz_s, yaw_s = self._update_smooth(
            px_raw, py_raw, pz_raw, yaw_raw, n_markers
        )
        px_out, py_out, pz_out = self._to_body_frame(px_s, py_s, pz_s)

        self.last_pub_meta = {
            "cx": t.cx,
            "cy": t.cy,
            "w": t.w,
            "h": t.h,
            "score": t.score,
            "category": t.category,
            "category_id": t.category_id,
            "tracked_id": t.tracked_id,
            "los_ax": t.los_ax,
            "los_ay": t.los_ay,
            "pitch": t.pitch,
            "roll": t.roll,
        }
        self._publish(msg, px_out, py_out, pz_out, yaw_s, self.last_pub_meta)

        self.last_raw = (px_raw, py_raw, pz_raw, yaw_raw, stamp)
        self.stats["pass"] += 1
        self._status(
            "OK n=%d big=%d score=%.4f -> sm px=%+.3f py=%+.3f pz=%.3f yaw=%+.1f"
            % (
                n_markers,
                int(t.w),
                float(t.score),
                px_out,
                py_out,
                pz_out,
                yaw_s,
            )
        )

    def _maybe_hold(self, msg, reason):
        """尖峰被拒时短暂 hold 上一帧平滑结果，减少降落端丢目标。"""
        cfg = self.cfg
        if not cfg.get("hold_on_reject", True):
            return False
        if reason not in ("reject_jump", "reject_score"):
            return False
        if self.smooth is None or self.last_raw is None or self.last_pub_meta is None:
            return False
        lstamp = self.last_raw[4]
        dt = (rospy.Time.now() - lstamp).to_sec()
        if dt > float(cfg["lost_timeout"]):
            return False

        px_s, py_s, pz_s, yaw_s = self.smooth
        px_out, py_out, pz_out = self._to_body_frame(px_s, py_s, pz_s)
        self._publish(msg, px_out, py_out, pz_out, yaw_s, self.last_pub_meta)
        self.stats["hold"] += 1
        self._status(
            "HOLD(%s) px=%+.3f py=%+.3f pz=%.3f yaw=%+.1f"
            % (reason, px_out, py_out, pz_out, yaw_s)
        )
        return True

    def _publish(self, msg, px, py, pz, yaw, meta):
        tout = TargetMsg()
        tout.cx = meta["cx"]
        tout.cy = meta["cy"]
        tout.w = meta["w"]
        tout.h = meta["h"]
        tout.score = meta["score"]
        tout.category = meta["category"]
        tout.category_id = meta["category_id"]
        tout.tracked_id = meta["tracked_id"]
        tout.px = px
        tout.py = py
        tout.pz = pz
        tout.los_ax = meta["los_ax"]
        tout.los_ay = meta["los_ay"]
        tout.yaw = yaw
        tout.pitch = meta["pitch"]
        tout.roll = meta["roll"]

        out = TargetsInFrameMsg()
        out.header = msg.header
        out.frame_id = msg.frame_id
        out.height = msg.height
        out.width = msg.width
        out.fps = msg.fps
        out.fov_x = msg.fov_x
        out.fov_y = msg.fov_y
        out.targets = [tout]
        self.pub.publish(out)

    def _update_smooth(self, px, py, pz, yaw, n_markers):
        cfg = self.cfg
        if not cfg.get("smooth_enable", True):
            self.smooth = (px, py, pz, yaw)
            return self.smooth

        sparse = self._is_sparse(n_markers)
        a = float(
            cfg["smooth_alpha_sparse"] if sparse else cfg["smooth_alpha_dense"]
        )
        ay = float(
            cfg["smooth_alpha_yaw_sparse"]
            if sparse
            else cfg["smooth_alpha_yaw_dense"]
        )
        # 码越多，略加快跟踪（最多再抬 0.15）
        boost = min(0.15, max(0.0, (n_markers - 2) * 0.03))
        a = min(0.9, a + boost)
        ay = min(0.9, ay + boost * 0.5)

        if self.smooth is None:
            self.smooth = (px, py, pz, yaw)
            return self.smooth

        spx, spy, spz, syaw = self.smooth
        spx = (1.0 - a) * spx + a * px
        spy = (1.0 - a) * spy + a * py
        spz = (1.0 - a) * spz + a * pz
        syaw = _lerp_angle(syaw, yaw, ay)
        self.smooth = (spx, spy, spz, syaw)
        return self.smooth

    def _to_body_frame(self, px, py, pz):
        cfg = self.cfg
        if cfg.get("invert_py", False):
            py = -py
        px = px + float(cfg.get("body_offset_x", 0.0))
        py = py + float(cfg.get("body_offset_y", 0.0))
        pz = pz + float(cfg.get("body_offset_z", 0.0))
        return px, py, pz

    def _is_sparse(self, n_markers):
        return n_markers < int(self.cfg.get("sparse_marker_threshold", 4))

    def _score_limit(self, n_markers):
        cfg = self.cfg
        if self._is_sparse(n_markers):
            return float(cfg.get("max_score_sparse", cfg["max_score"]))
        return float(cfg["max_score"])

    def _jump_limits(self, n_markers):
        cfg = self.cfg
        if self._is_sparse(n_markers):
            return (
                float(cfg.get("max_jump_xy_sparse", cfg["max_jump_xy"])),
                float(cfg.get("max_jump_z_sparse", cfg["max_jump_z"])),
                float(cfg.get("max_jump_yaw_sparse", cfg.get("max_jump_yaw", 40.0))),
            )
        return (
            float(cfg["max_jump_xy"]),
            float(cfg["max_jump_z"]),
            float(cfg.get("max_jump_yaw", 40.0)),
        )

    def _check(self, msg):
        cfg = self.cfg
        if not msg.targets:
            return "reject_empty"

        t = msg.targets[0]
        n_markers = int(t.category_id)
        n_big = int(t.w)
        score = float(t.score)
        px, py, pz = float(t.px), float(t.py), float(t.pz)
        yaw = float(t.yaw)

        if n_markers < int(cfg["min_markers"]):
            return "reject_markers"
        if cfg.get("require_big_marker") and n_big < 1:
            return "reject_big"
        if score > self._score_limit(n_markers):
            return "reject_score"
        if pz < float(cfg["pz_min"]) or pz > float(cfg["pz_max"]):
            return "reject_pz"
        if math.isnan(px) or math.isnan(py) or math.isnan(pz) or math.isnan(yaw):
            return "reject_pz"

        if self.last_raw is not None:
            lpx, lpy, lpz, lyaw, lstamp = self.last_raw
            dt = (rospy.Time.now() - lstamp).to_sec()
            if dt <= float(cfg["lost_timeout"]) * 4:
                jump_xy, jump_z, jump_yaw = self._jump_limits(n_markers)
                if abs(px - lpx) > jump_xy or abs(py - lpy) > jump_xy:
                    return "reject_jump"
                if abs(pz - lpz) > jump_z:
                    return "reject_jump"
                if abs(_wrap_deg(yaw - lyaw)) > jump_yaw:
                    return "reject_jump"

        return None


def main():
    cfg_path = DEFAULT_CFG
    if len(sys.argv) > 1:
        cfg_path = sys.argv[1]
    cfg = load_cfg(cfg_path)

    rospy.init_node("landmark_pose_gate")
    PoseGate(cfg)
    rospy.loginfo("landmark_pose_gate started")
    rospy.loginfo("  in : %s", cfg["topic_in"])
    rospy.loginfo("  out: %s", cfg["topic_out"])
    rospy.loginfo("  cfg: %s", cfg_path)
    rospy.loginfo(
        "  min_markers=%s require_big=%s smooth=%s alpha_sparse=%s",
        cfg["min_markers"],
        cfg.get("require_big_marker"),
        cfg.get("smooth_enable"),
        cfg.get("smooth_alpha_sparse"),
    )
    rospy.spin()
    return 0


if __name__ == "__main__":
    sys.exit(main())
