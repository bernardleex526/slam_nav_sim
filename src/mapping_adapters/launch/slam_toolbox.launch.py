#!/usr/bin/env python3
"""slam_toolbox（2D baseline）适配入口。等价于 mapping.launch.py mapping:=slam_toolbox。"""

import os
import sys

from ament_index_python.packages import get_package_share_directory

sys.path.insert(0, os.path.join(get_package_share_directory("mapping_adapters"), "launch"))
import mapping_common  # noqa: E402


def generate_launch_description():
    return mapping_common.make_mapping_launch_description("slam_toolbox")
