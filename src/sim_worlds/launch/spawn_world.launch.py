#!/usr/bin/env python3
"""
spawn_world.launch.py
=====================
启动 Gazebo 并加载三张测试地图之一。

这里用 ExecuteProcess 直接拉 gzserver/gzclient，而不是 include gazebo_ros 的
launch 文件 —— 官方那个 launch 的参数名在不同发行版里变过，直接拉进程更可控。

关键点：必须显式加载 libgazebo_ros_init.so 和 libgazebo_ros_factory.so，
否则 gzserver 里没有任何 ROS 接口，spawn_entity 会连不上（报 timeout）。
"""

import os
import tempfile
import xml.etree.ElementTree as ET

from ament_index_python.packages import get_package_share_directory, get_packages_with_prefixes
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, LogInfo, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration


def launch_setup(context, *args, **kwargs):
    world_name = LaunchConfiguration("world").perform(context)
    verbose = LaunchConfiguration("verbose").perform(context)
    gui = LaunchConfiguration("gui").perform(context)
    headless = LaunchConfiguration("headless").perform(context)
    paused = LaunchConfiguration("paused").perform(context)

    worlds_dir = os.path.join(get_package_share_directory("sim_worlds"), "worlds")
    world_file = os.path.join(worlds_dir, f"{world_name}.world")

    if not os.path.isfile(world_file):
        raise RuntimeError(
            f"找不到 world 文件：{world_file}\n"
            f"请先运行：python3 {os.path.join(get_package_share_directory('sim_worlds'), '..', '..', 'lib', 'sim_worlds', 'generate_worlds.py')}"
        )

    # ROS 2 uses state services, not the ROS 1 /gazebo/set_model_state topic.
    tree = ET.parse(world_file)
    world = tree.getroot().find("world")
    if not any(p.get("filename") == "libgazebo_ros_state.so" for p in world.findall("plugin")):
        plugin = ET.SubElement(world, "plugin", name="gazebo_ros_state", filename="libgazebo_ros_state.so")
        ET.SubElement(ET.SubElement(plugin, "ros"), "namespace").text = "/gazebo"
        ET.SubElement(plugin, "update_rate").text = "20.0"
    fd, runtime_world = tempfile.mkstemp(prefix=f"sim_{world_name}_", suffix=".world")
    os.close(fd)
    tree.write(runtime_world, encoding="utf-8", xml_declaration=True)
    world_file = runtime_world

    # headless 优先：批量跑测试时不需要 GUI，省掉大量渲染开销
    show_gui = (gui == "true") and (headless != "true")

    gz_args = [
        "gzserver",
        "--verbose" if verbose == "true" else "-q",
        "-s", "libgazebo_ros_init.so",
        "-s", "libgazebo_ros_factory.so",
    ]
    if paused == "true":
        gz_args.append("-u")
    gz_args.append(world_file)

    # Gazebo converts URDF package:// mesh URIs to model:// URIs. Include
    # package share parents so installed external robot meshes resolve locally.
    model_paths = [os.path.join(prefix, "share")
                   for prefix in get_packages_with_prefixes().values()]
    existing = os.environ.get("GAZEBO_MODEL_PATH", "")
    model_path = os.pathsep.join(dict.fromkeys(model_paths + ([existing] if existing else [])))

    actions = [
        LogInfo(msg=f"[sim_worlds] 加载地图：{world_file}（GUI={'on' if show_gui else 'off'}）"),
        ExecuteProcess(cmd=gz_args, output="screen",
                       additional_env={"GAZEBO_MODEL_PATH": model_path}),
        ExecuteProcess(
            cmd=["gzclient"],
            output="screen",
            condition=IfCondition(str(show_gui).lower()),
        ),
    ]
    return actions


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "world",
                default_value="simple",
                choices=["simple", "medium", "complex"],
                description="测试地图：simple / medium / complex",
            ),
            DeclareLaunchArgument("gui", default_value="true", description="是否启动 gzclient"),
            DeclareLaunchArgument(
                "headless", default_value="false",
                description="true 时强制不开 GUI（批量 benchmark 用）",
            ),
            DeclareLaunchArgument("verbose", default_value="false"),
            DeclareLaunchArgument("paused", default_value="false", description="以暂停状态启动"),
            OpaqueFunction(function=launch_setup),
        ]
    )
