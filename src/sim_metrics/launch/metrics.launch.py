#!/usr/bin/env python3
"""
metrics.launch.py
=================
启动指标采集节点。

    ros2 launch sim_metrics metrics.launch.py run_name:=complex_baseline
    ros2 launch sim_metrics metrics.launch.py run_name:=complex_smac_mppi

每次跑测试用一个独立的 run_name，这样多组实验的结果分目录存放，
benchmark runner 汇总时不会串。
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "run_name", default_value="",
                description="本次运行名；留空则用时间戳",
            ),
            DeclareLaunchArgument("output_dir", default_value="~/.ros/sim_eval"),
            DeclareLaunchArgument("target_frame", default_value="map",
                                  description="轨迹统一到的坐标系，不存在时自动回退 odom"),
            DeclareLaunchArgument("save_trajectory_csv", default_value="true"),
            Node(
                package="sim_metrics",
                executable="metrics_collector.py",
                name="metrics_collector",
                output="screen",
                parameters=[{
                    "use_sim_time": True,
                    "run_name": LaunchConfiguration("run_name"),
                    "output_dir": LaunchConfiguration("output_dir"),
                    "target_frame": LaunchConfiguration("target_frame"),
                    "save_trajectory_csv": LaunchConfiguration("save_trajectory_csv"),
                }],
            ),
        ]
    )
