#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Save EgoPlanner trajectories to CSV and republish as a thick Path for RViz."""
import os
import datetime
import rospy
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
from visualization_msgs.msg import Marker
from traj_utils.msg import Bspline

OUTDIR = os.path.expanduser("~/ego_ws/run/trajs")


class TrajSaver(object):
    def __init__(self):
        os.makedirs(OUTDIR, exist_ok=True)
        self.pub = rospy.Publisher("/ego_traj_path", Path, queue_size=1, latch=True)
        rospy.Subscriber("/drone_0_planning/bspline", Bspline, self.on_bspline, queue_size=20)
        rospy.Subscriber("/drone_0_ego_planner_node/optimal_list", Marker, self.on_marker, queue_size=20)
        rospy.logwarn("traj_saver ready -> %s  (also /ego_traj_path)", OUTDIR)

    def write_csv(self, pts, prefix):
        ts = datetime.datetime.now().strftime("%Y%m%d_%H%M%S_%f")[:-3]
        path = os.path.join(OUTDIR, "%s_%s.csv" % (prefix, ts))
        with open(path, "w") as f:
            f.write("x,y,z\n")
            for x, y, z in pts:
                f.write("%.6f,%.6f,%.6f\n" % (x, y, z))
        rospy.logwarn("SAVED TRAJ %s  points=%d", path, len(pts))
        latest = os.path.join(OUTDIR, "latest.csv")
        try:
            if os.path.islink(latest) or os.path.exists(latest):
                os.remove(latest)
            os.symlink(path, latest)
        except Exception:
            with open(latest, "w") as f:
                f.write("x,y,z\n")
                for x, y, z in pts:
                    f.write("%.6f,%.6f,%.6f\n" % (x, y, z))
        return path

    def publish_path(self, pts):
        msg = Path()
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = "world"
        for x, y, z in pts:
            ps = PoseStamped()
            ps.header = msg.header
            ps.pose.position.x = x
            ps.pose.position.y = y
            ps.pose.position.z = z
            ps.pose.orientation.w = 1.0
            msg.poses.append(ps)
        self.pub.publish(msg)

    def on_bspline(self, msg):
        pts = [(p.x, p.y, p.z) for p in msg.pos_pts]
        if len(pts) < 2:
            return
        self.write_csv(pts, "bspline")
        self.publish_path(pts)

    def on_marker(self, msg):
        pts = [(p.x, p.y, p.z) for p in msg.points]
        if len(pts) < 2:
            return
        self.write_csv(pts, "optimal")
        self.publish_path(pts)


if __name__ == "__main__":
    rospy.init_node("traj_saver")
    TrajSaver()
    rospy.spin()
