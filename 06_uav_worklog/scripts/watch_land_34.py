#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""与 watch_land_status 完全相同，保留这个名字只是为了旧习惯还能用。

  python3 ~/uav_worklog/scripts/watch_land_34.py
  tail -f ~/uav_worklog/run/land_tips.log
"""

import os
import runpy

runpy.run_path(
    os.path.join(os.path.dirname(os.path.abspath(__file__)), "watch_land_status.py"),
    run_name="__main__",
)
