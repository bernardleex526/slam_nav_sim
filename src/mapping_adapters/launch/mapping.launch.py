#!/usr/bin/env python3
"""
mapping.launch.py — 建图算法统一分发入口
========================================
    ros2 launch mapping_adapters mapping.launch.py mapping:=fast_lio
    ros2 launch mapping_adapters mapping.launch.py mapping:=lio_sam
    ros2 launch mapping_adapters mapping.launch.py mapping:=rtabmap
    ros2 launch mapping_adapters mapping.launch.py mapping:=slam_toolbox
    ros2 launch mapping_adapters mapping.launch.py mapping:=none      # 只起契约适配

这就是需求 #3「暴露出对接测试环境的建图和导航接口，可通过选择不同的建图算法」
的落点：换算法只改一个 launch 参数，下游（rviz / 指标 / benchmark）完全不受影响。
"""

import os
import sys

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, OpaqueFunction
from launch.substitutions import LaunchConfiguration

sys.path.insert(0, os.path.join(get_package_share_directory("mapping_adapters"), "launch"))
import mapping_common  # noqa: E402


def _dispatch(context):
    algorithm = LaunchConfiguration("mapping").perform(context)
    if algorithm == "none":
        return [
            LogInfo(
                msg="[mapping] mapping:=none —— 不启动建图算法，"
                "只保留契约适配节点（用于纯导航测试或从 bag 回放）。"
            )
        ]
    return mapping_common.setup_mapping(context, algorithm)


def generate_launch_description():
    matrix = mapping_common.load_matrix()
    choices = ["none"] + list(matrix["algorithms"].keys())

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "mapping", default_value="none", choices=choices,
                description="要启动的建图算法",
            ),
            DeclareLaunchArgument(
                "external_launch", default_value="",
                description="外部算法 launch 文件的绝对路径；留空则自动在包内查找",
            ),
            DeclareLaunchArgument(
                "lidar_mode", default_value="points", choices=["points", "scan", "both"],
                description="是否额外切出 2D /scan（2D SLAM 需要）",
            ),
            DeclareLaunchArgument("publish_2d_map", default_value="true"),
            DeclareLaunchArgument("publish_downsample_levels", default_value="true"),
            OpaqueFunction(function=_dispatch),
        ]
    )
