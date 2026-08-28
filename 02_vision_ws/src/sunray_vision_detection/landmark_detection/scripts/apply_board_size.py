#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""根据 board_physical.yaml 计算并写入 aruco_detector_params.json 的 markerLengths。

重要：该包的 gason 加载方式是按行 getline 再拼接。原文件使用 CRLF（\\r\\n）。
若改成 Linux 的 LF（\\n），会出现 markerIds/markerLengths 长度不一致并崩溃。
因此本脚本必须用字节替换，并保持 CRLF。
"""

from __future__ import print_function

import os
import re
import sys

try:
    import yaml
except ImportError:
    yaml = None


PKG_DIR = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
BOARD_YAML = os.path.join(PKG_DIR, "config", "board_physical.yaml")
PARAMS_JSON = os.path.join(PKG_DIR, "config", "aruco_detector_params.json")


def load_board_yaml(path):
    with open(path, "r") as f:
        text = f.read()
    if yaml is not None:
        return yaml.safe_load(text)

    data = {}
    for line in text.splitlines():
        line = line.split("#", 1)[0].strip()
        if not line or ":" not in line:
            continue
        key, val = line.split(":", 1)
        data[key.strip()] = float(val.strip())
    return data


def fmt_len(x):
    """格式化成与原文件相近的短小数，例如 0.0097 / 0.0390。"""
    s = "%.4f" % float(x)
    # 去掉多余 0，但保证至少有一位小数，且看起来像 0.xxxx
    if "." in s:
        whole, frac = s.split(".", 1)
        frac = frac.rstrip("0")
        if not frac:
            frac = "0"
        # 为稳定解析，统一补到 4 位（与原 0.0128 风格接近）
        frac = (frac + "0000")[:4]
        s = whole + "." + frac
    return s


def main():
    board = load_board_yaml(BOARD_YAML)
    design_board = float(board["design_board_side_m"])
    design_small = float(board["design_small_marker_m"])
    design_large = float(board["design_large_marker_m"])

    measured_small = board.get("measured_small_marker_m")
    measured_large = board.get("measured_large_marker_m")

    if measured_small is not None and measured_large is not None:
        small = float(measured_small)
        large = float(measured_large)
        if small <= 0 or large <= 0:
            print("measured_*_marker_m 必须 > 0", file=sys.stderr)
            return 1
        print("使用卷尺实测黑块边长（米）")
    else:
        board_side = float(board["board_side_m"])
        if board_side <= 0 or design_board <= 0:
            print("board_side_m / design_board_side_m 必须 > 0", file=sys.stderr)
            return 1
        scale = board_side / design_board
        small = design_small * scale
        large = design_large * scale
        print("按整板比例缩放（无实测黑块时） scale=%.4f" % scale)

    depth_scale = float(board.get("depth_scale", 1.0) or 1.0)
    if depth_scale <= 0:
        print("depth_scale 必须 > 0", file=sys.stderr)
        return 1
    if abs(depth_scale - 1.0) > 1e-6:
        print("应用 depth_scale = %.4f（临时深度修正，标定后应改回 1.0）" % depth_scale)
        small *= depth_scale
        large *= depth_scale

    small_s = fmt_len(small)
    large_s = fmt_len(large)

    with open(PARAMS_JSON, "rb") as f:
        data = f.read()

    if b"\r\n" not in data:
        # 强制转成 CRLF，避免再次踩坑
        data = data.replace(b"\r\n", b"\n").replace(b"\n", b"\r\n")

    # 找到 markerLengths 数组并整体替换
    m = re.search(br'"markerLengths"\s*:\s*\[(.*?)\]', data, flags=re.S)
    if not m:
        print("未找到 markerLengths 数组", file=sys.stderr)
        return 1

    ids_m = re.search(br'"markerIds"\s*:\s*\[(.*?)\]', data, flags=re.S)
    if not ids_m:
        print("未找到 markerIds 数组", file=sys.stderr)
        return 1

    ids_txt = ids_m.group(1).decode("ascii", errors="ignore")
    ids = [int(x.strip()) for x in ids_txt.split(",") if x.strip()]
    lengths = [large_s if i >= 91 else small_s for i in ids]
    new_arr = ",".join(lengths).encode("ascii")

    data2 = data[: m.start(1)] + new_arr + data[m.end(1) :]

    if b"\r\n" not in data2:
        print("内部错误：写出结果不是 CRLF", file=sys.stderr)
        return 1

    with open(PARAMS_JSON, "wb") as f:
        f.write(data2)

    print("已根据 board_physical.yaml 更新 markerLengths（保持 CRLF）")
    if measured_small is not None and measured_large is not None:
        print("  来源     卷尺实测黑块")
    else:
        print("  整板边长 board_side_m = {:.6f} m ({:.2f} cm)".format(board_side, board_side * 100))
        print("  缩放比   scale        = {:.6f}".format(scale))
    print("  小码边长 small        = {} m".format(small_s))
    print("  大码边长 large        = {} m".format(large_s))
    print("  depth_scale           = {:.4f}".format(depth_scale))
    print("  ids/lengths 个数      = %d / %d" % (len(ids), len(lengths)))
    print("  写入文件:", PARAMS_JSON)
    print()
    print("下一步：重启 landmark_detection 节点使参数生效")
    return 0


if __name__ == "__main__":
    sys.exit(main())
