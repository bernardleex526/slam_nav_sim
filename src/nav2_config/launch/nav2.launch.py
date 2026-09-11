#!/usr/bin/env python3
"""
nav2.launch.py
==============
Nav2 统一入口。

    ros2 launch nav2_config nav2.launch.py \\
        robot:=wheeled_quadruped \\
        global_planner:=smac_2d \\
        local_planner:=dwb \\
        odom_source:=gt \\
        localization:=slam

流程：
  1. 调用 tools/compose_nav2_params.py 合成参数文件（同步，先落盘再启动，
     避免 Nav2 读到半成品）
  2. include nav2_bringup 的标准 bringup

关于 localization：
  slam 模式 —— 交给建图算法发布 map->odom，Nav2 不启动 AMCL。
               这是建图 + 导航同时验证时的正确姿势。
  amcl 模式 —— 用已保存的地图做定位，Nav2 启动 map_server + amcl。
               这是"规划器参数对比"实验的正确姿势：地图固定，
               变量只有规划器，否则每次建图都不一样，没法比。

  ⚠️ 两种模式绝对不能同时用：SLAM 和 AMCL 都会发布 map->odom，
     同时跑会让 TF 树出现双父节点，rviz2 报错但 Nav2 可能仍"看起来在跑"。
"""

import os
import subprocess
import sys
import tempfile
import yaml

from ament_index_python.packages import get_package_prefix, get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    LogInfo,
    OpaqueFunction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def _two_bool(context, name):
    return LaunchConfiguration(name).perform(context).strip().lower() == "true"


def unique_params_path(directory, profile):
    descriptor, path = tempfile.mkstemp(prefix=f"nav2_{profile}_", suffix=".yaml", dir=directory)
    os.close(descriptor)
    return path


def launch_setup(context, *args, **kwargs):
    robot = LaunchConfiguration("robot").perform(context)
    g_planner = LaunchConfiguration("global_planner").perform(context)
    l_planner = LaunchConfiguration("local_planner").perform(context)
    odom_source = LaunchConfiguration("odom_source").perform(context)
    localization = LaunchConfiguration("localization").perform(context)
    map_yaml = LaunchConfiguration("map").perform(context)
    autostart = _two_bool(context, "autostart")

    nav2_share = get_package_share_directory("nav2_config")
    prefix = get_package_prefix("nav2_config")
    # 用 get_package_prefix 定位可执行脚本，不要手拼路径。
    # 之前写成 os.path.dirname(share) 拼出来的是 share/lib/... ，少了一级，
    # 结果这里是"找不到脚本 -> 合成失败 -> 直接 return"，Nav2 静默不启动，
    # 而日志里只有一行 [nav2] 参数合成失败，非常容易被忽略。
    candidates = [
        os.path.join(prefix, "lib", "nav2_config", "compose_nav2_params.py"),
        os.path.join(nav2_share, "tools", "compose_nav2_params.py"),
    ]
    composer = next((c for c in candidates if os.path.isfile(c)), None)
    if composer is None:
        return [
            LogInfo(
                msg="[nav2] 找不到 compose_nav2_params.py，Nav2 无法启动。\n"
                    "  已尝试：\n" + "\n".join(f"    - {c}" for c in candidates) +
                    "\n  请先执行 colcon build --packages-select nav2_config"
            )
        ]

    out_dir = os.path.join(tempfile.gettempdir(), "sim_ws_nav2")
    os.makedirs(out_dir, exist_ok=True)
    out_file = unique_params_path(out_dir, f"{robot}_{g_planner}_{l_planner}_{odom_source}_{localization}")

    cmd = [
        sys.executable, composer,
        "--robot", robot,
        "--global-planner", g_planner,
        "--local-planner", l_planner,
        "--odom-source", odom_source,
        "--localization", localization,
        "--out", out_file,
    ]
    proc = subprocess.run(cmd, capture_output=True, text=True)
    if proc.returncode != 0:
        return [
            LogInfo(msg=f"[nav2] 参数合成失败：\n{proc.stdout}\n{proc.stderr}"),
        ]

    # AMCL must initialize at the same world pose used by Gazebo spawn.
    with open(out_file, encoding="utf-8") as stream:
        params = yaml.safe_load(stream)
    params["amcl"]["ros__parameters"]["initial_pose"] = {
        key: float(LaunchConfiguration("initial_" + key).perform(context))
        for key in ("x", "y", "yaw")
    }
    params["amcl"]["ros__parameters"]["initial_pose"]["z"] = 0.0
    with open(out_file, "w", encoding="utf-8") as stream:
        yaml.safe_dump(params, stream, sort_keys=False)

    actions = [
        LogInfo(msg=f"[nav2] 参数已合成：{out_file}\n{proc.stdout.strip()}"),
    ]

    bringup_share = get_package_share_directory("nav2_bringup")
    bringup = os.path.join(bringup_share, "launch", "bringup_launch.py")

    use_slam = localization == "slam"
    launch_args = {
        "use_sim_time": "true",
        "params_file": out_file,
        "autostart": "true" if autostart else "false",
        # use_composition=False：组合容器在 WSL 里偶发加载失败，拆开更好调试
        "use_composition": "False",
        "slam": "True" if use_slam else "False",
    }
    if not use_slam:
        if not map_yaml:
            actions.append(
                LogInfo(
                    msg="[nav2] localization:=amcl 但未提供 map:=<yaml 路径>。\n"
                        "  AMCL 需要一张已保存的地图。可以先用 SLAM 跑一圈然后：\n"
                        "    ros2 run nav2_map_server map_saver_cli "
                        "-f $HOME/maps/my_map --ros-args -p save_map_timeout:=10.0\n"
                        "  再用 map:=$HOME/maps/my_map.yaml 重跑。"
                )
            )
            return actions
        launch_args["map"] = map_yaml
    else:
        # nav2_bringup 把 'map' 声明成【必填参数】（无默认值），即使 slam:=True
        # 时根本不会用到它。不传就会直接抛
        #   "Included launch description missing required argument 'map'"
        # 整个 Nav2 一行都不启动。传空串是安全的：slam 分支不会读它。
        launch_args["map"] = ""
        actions.append(
            LogInfo(
                msg="[nav2] localization:=slam —— 不启动 AMCL。"
                "请确保建图算法已在运行并发布 map->odom，否则 Nav2 会一直等 TF。"
            )
        )

    actions.append(
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(bringup),
            launch_arguments=launch_args.items(),
        )
    )
    return actions


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument("robot", default_value="wheeled_quadruped"),
            DeclareLaunchArgument(
                "global_planner", default_value="smac_2d",
                choices=["navfn", "smac_2d", "smac_hybrid"],
            ),
            DeclareLaunchArgument(
                "local_planner", default_value="dwb",
                choices=["dwb", "rpp", "mppi", "teb"],
            ),
            DeclareLaunchArgument(
                "odom_source", default_value="gt", choices=["gt", "lio"],
                description="里程计来源，必须与顶层 launch 一致",
            ),
            DeclareLaunchArgument(
                "localization", default_value="slam", choices=["slam", "amcl"],
            ),
            DeclareLaunchArgument(
                "map", default_value="",
                description="localization:=amcl 时必须提供已保存地图的 yaml 路径",
            ),
            DeclareLaunchArgument("autostart", default_value="true"),
            DeclareLaunchArgument("initial_x", default_value="0.0"),
            DeclareLaunchArgument("initial_y", default_value="0.0"),
            DeclareLaunchArgument("initial_yaw", default_value="0.0"),
            OpaqueFunction(function=launch_setup),
        ]
    )
