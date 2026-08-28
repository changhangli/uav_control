#!/usr/bin/env python3
"""相机棋盘格标定启动脚本（显式重映射 image → /web_cam/image_raw）。

在 Mini-Pc 桌面终端运行：
  bash ~/uav_worklog/scripts/run_camera_calibrate.sh
"""
from __future__ import print_function

import functools
import os
import subprocess
import sys


def ensure_ros():
    try:
        import rospy  # noqa: F401
        import camera_calibration  # noqa: F401
        import cv2  # noqa: F401
    except ImportError:
        cmd = (
            "source /opt/ros/noetic/setup.bash && "
            "source \"$HOME/vision_ws/devel/setup.bash\" && "
            "exec python3 \"%s\" %s"
            % (os.path.abspath(__file__), " ".join(sys.argv[1:]))
        )
        os.execv("/bin/bash", ["bash", "-lc", cmd])


def check_camera():
    r = subprocess.call(
        "source /opt/ros/noetic/setup.bash && "
        "source \"$HOME/vision_ws/devel/setup.bash\" && "
        "timeout 3 rostopic echo -n 1 /web_cam/image_raw >/dev/null",
        shell=True,
        executable="/bin/bash",
    )
    return r == 0


def main():
    size = sys.argv[1] if len(sys.argv) > 1 else "8x5"
    square = sys.argv[2] if len(sys.argv) > 2 else "0.020"

    if not os.environ.get("DISPLAY"):
        print("没有 DISPLAY。请在 Mini-Pc 桌面终端运行。")
        return 1

    if not check_camera():
        print("收不到 /web_cam/image_raw。请先另开终端：")
        print("  source /opt/ros/noetic/setup.bash && source ~/vision_ws/devel/setup.bash")
        print("  roslaunch web_cam web_cam.launch")
        return 1

    import cv2
    import message_filters
    import rospy
    from camera_calibration.calibrator import ChessboardInfo, Patterns
    from camera_calibration.camera_calibrator import OpenCVCalibrationNode

    # 防止还没出图就点击崩溃
    _orig = OpenCVCalibrationNode.on_mouse

    def _safe_on_mouse(self, event, x, y, flags, param):
        if not hasattr(self, "displaywidth"):
            return
        return _orig(self, event, x, y, flags, param)

    OpenCVCalibrationNode.on_mouse = _safe_on_mouse

    print("参数: --size %s --square %s" % (size, square))
    print("准备订阅: /web_cam/image_raw")
    print("等窗口出现彩色角点后再多角度移动；够了再点 CALIBRATE → SAVE")
    print("手绘板：黑格尽量涂满涂深，整板进画面、放平")
    print()

    boards = []
    wh = tuple(int(c) for c in size.split("x"))
    boards.append(ChessboardInfo("chessboard", wh[0], wh[1], float(square)))

    sync = message_filters.TimeSynchronizer

    # 官方默认会 FIX_K3；这里保持与官方 cameracalibrator 类似
    num_ks = 2
    calib_flags = 0
    if num_ks < 6:
        calib_flags |= cv2.CALIB_FIX_K6
    if num_ks < 5:
        calib_flags |= cv2.CALIB_FIX_K5
    if num_ks < 4:
        calib_flags |= cv2.CALIB_FIX_K4
    if num_ks < 3:
        calib_flags |= cv2.CALIB_FIX_K3
    if num_ks < 2:
        calib_flags |= cv2.CALIB_FIX_K2
    if num_ks < 1:
        calib_flags |= cv2.CALIB_FIX_K1

    fisheye_calib_flags = 0
    # 手绘棋盘对比度差时关掉 FAST_CHECK，更容易检出
    checkerboard_flags = 0
    print("已关闭 FAST_CHECK（更适合手绘棋盘）")

    # 关键：重映射必须作为 init_node 的 argv 传入（不要依赖 sys.argv 旁路）
    init_argv = [
        "cameracalibrator.py",
        "image:=/web_cam/image_raw",
        "camera:=/web_cam",
    ]
    rospy.init_node("cameracalibrator", argv=init_argv, anonymous=False)

    resolved = rospy.resolve_name("image")
    mappings = rospy.names.get_mappings()
    print("mappings:", mappings)
    print("实际订阅 image ->", resolved)
    if resolved != "/web_cam/image_raw":
        print("ERROR: 重映射失败，请把上面两行发我")
        return 2

    OpenCVCalibrationNode(
        boards,
        False,  # no service check
        sync,
        calib_flags,
        fisheye_calib_flags,
        Patterns.Chessboard,
        "head_camera",
        checkerboard_flags=checkerboard_flags,
        max_chessboard_speed=-1.0,
        queue_size=1,
    )
    rospy.spin()
    return 0


if __name__ == "__main__":
    ensure_ros()
    sys.exit(main() or 0)
