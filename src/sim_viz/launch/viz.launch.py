#!/usr/bin/env python3
"""
viz.launch.py
=============
启动 rviz2，加载三套配置之一。

    ros2 launch sim_viz viz.launch.py view:=mapping
    ros2 launch sim_viz viz.launch.py view:=navigation
    ros2 launch sim_viz viz.launch.py view:=evaluation

关于 --ros-args -p use_sim_time:=true：
  rviz2 必须开 use_sim_time，否则在仿真时间下 TF 查询会用到 wall clock，
  报 "Timed out waiting for transform"，表现为模型和点云全部显示不出来。
  这是接仿真时最常见的 rviz 坑，这里默认就开好。
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

VIEWS = ["mapping", "navigation", "evaluation"]


def launch_setup(context, *args, **kwargs):
    view = LaunchConfiguration("view").perform(context)
    rviz_share = get_package_share_directory("sim_viz")
    cfg = os.path.join(rviz_share, "rviz", f"{view}.rviz")

    if not os.path.isfile(cfg):
        return [
            LogInfo(
                msg=f"[sim_viz] 找不到配置 {cfg}。\n"
                    f"  请先生成：python3 {os.path.join(rviz_share, '..', 'lib', 'sim_viz', 'gen_rviz_configs.py')}"
            )
        ]

    return [
        LogInfo(msg=f"[sim_viz] 加载 rviz2 配置：{cfg}"),
        Node(
            package="rviz2",
            executable="rviz2",
            name="rviz2",
            output="screen",
            arguments=["-d", cfg],
            parameters=[{"use_sim_time": True}],
        ),
    ]


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument("view", default_value="mapping", choices=VIEWS),
            OpaqueFunction(function=launch_setup),
        ]
    )
