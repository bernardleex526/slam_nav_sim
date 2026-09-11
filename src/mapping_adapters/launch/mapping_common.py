#!/usr/bin/env python3
"""
mapping_common
==============
所有建图算法 launch 的共享装配逻辑。

存在的理由：FAST-LIO2 / LIO-SAM / RTAB-Map / slam_toolbox 的 launch 差别只在
「外部包怎么起 + 输出话题叫什么」，其余的适配节点装配完全一样。把这些共同逻辑
抽出来，算法 launch 就退化成一张表 —— 加新算法只需要在 mapping_matrix.yaml
加一段，不用再抄一份 150 行的 launch。

注意本文件放在 launch/ 目录下并被安装到 share/，供各 launch 通过 sys.path 导入。
它不是可执行 launch 文件。
"""

import os

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    LogInfo,
    OpaqueFunction,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def load_matrix():
    path = os.path.join(
        get_package_share_directory("mapping_adapters"), "config", "mapping_matrix.yaml"
    )
    with open(path, "r") as f:
        return yaml.safe_load(f)


def find_external_launch(package: str, launch_file: str):
    """尝试定位外部算法包里的 launch 文件。找不到返回 None，不抛异常 ——
    用户很可能是分步搭建，先跑通仿真再装算法。"""
    if not package or not launch_file:
        return None
    try:
        share = get_package_share_directory(package)
    except Exception:
        return None
    for sub in ("launch", "launch/", ""):
        p = os.path.join(share, sub, launch_file)
        if os.path.isfile(p):
            return p
    return None


def setup_mapping(context, algorithm: str):
    matrix = load_matrix()
    algo_cfg = matrix["algorithms"].get(algorithm)
    if algo_cfg is None:
        raise RuntimeError(
            f"mapping_matrix.yaml 中没有算法 '{algorithm}'。"
            f"可选：{', '.join(matrix['algorithms'].keys())}"
        )

    adapter_cfg = algo_cfg.get("adapter", {})
    external_pkg = algo_cfg.get("external_package", "")
    external_launch_name = algo_cfg.get("external_launch", "")

    # 用户可直接指定外部 launch 的绝对路径，优先级最高
    override = LaunchConfiguration("external_launch").perform(context)
    external_path = override or find_external_launch(external_pkg, external_launch_name)

    lidar_mode = LaunchConfiguration("lidar_mode").perform(context)  # points | scan | both
    publish_2d = LaunchConfiguration("publish_2d_map").perform(context)
    publish_downsample = LaunchConfiguration("publish_downsample_levels").perform(context)
    # 顶层 launch 已经起了 points_to_scan 时必须置 false，否则 /scan 会出现两个
    # 发布者。虽然数据一样不至于崩，但会让时间戳互相穿插、下游做同步时抓狂。
    own_scan = LaunchConfiguration("start_points_to_scan").perform(context) == "true"

    own_algo = LaunchConfiguration("start_external_algorithm").perform(context) == "true"

    actions = []

    # ---------------- 1. 外部算法本身 ----------------
    if not own_algo:
        actions.append(LogInfo(msg=(
            "[mapping] start_external_algorithm:=false —— 本 launch 不启动建图算法，"
            "只运行契约适配与可视化节点。\n"
            "  典型场景：navigation:=nav2 + localization:=slam 时，"
            "nav2_bringup 的 slam_launch.py 已经起了 slam_toolbox；\n"
            "  这里再起一个会造成【两个 map->odom 发布者】，TF 树被污染，"
            "导航目标会大面积 abort 且报错指向不明。"
        )))
    elif external_path and os.path.isfile(external_path):
        actions.append(
            LogInfo(msg=f"[mapping] 启动外部算法：{algo_cfg['display_name']} <- {external_path}")
        )
        # slam_toolbox 的官方 launch 用 slam_params_file 指定参数文件。
        # 不传的话它会 load 自己的 mapper_params_online_sync.yaml，
        # 那份文件里 min_laser_range=0.0，与 points_to_scan 的盲区 0.6 冲突 ——
        # 机器人周围被刷出假自由空间、近处墙被"擦除"，占据召回率因此偏低。
        # 所以这里必须把本项目调好的参数文件塞进去。
        inc_args = {"use_sim_time": "true"}
        if algorithm == "slam_toolbox":
            our_params = os.path.join(
                get_package_share_directory("mapping_adapters"),
                "config", "slam_toolbox_params.yaml",
            )
            if os.path.isfile(our_params):
                inc_args["slam_params_file"] = our_params
                actions.append(LogInfo(msg=f"[mapping] slam_toolbox 参数：{our_params}"))
            else:
                actions.append(LogInfo(msg=(
                    "[mapping] ⚠️ 未找到 slam_toolbox_params.yaml，将回退到官方默认参数"
                    "（min_laser_range=0.0，会导致机器人周围出现假自由空间）"
                )))
        actions.append(
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(external_path),
                launch_arguments=inc_args.items(),
            )
        )
    else:
        raise RuntimeError(
            f"Mapping algorithm unavailable: {external_pkg}/{external_launch_name}. "
            f"Install the algorithm or provide external_launch. "
            f"For an explicitly external process use start_external_algorithm:=false. "
            f"Upstream: {algo_cfg.get('upstream', 'N/A')}"
        )

    # ---------------- 2. 若算法需要 2D scan，切一个出来 ----------------
    needs_scan = algorithm in matrix.get("constraints", {}).get("requires_scan", [])
    if own_scan and (needs_scan or lidar_mode in ("scan", "both")):
        actions.append(
            Node(
                package="mapping_adapters",
                executable="points_to_scan.py",
                name="points_to_scan",
                output="screen",
                parameters=[{
                    "in_topic": "/points",
                    "out_topic": "/scan",
                    "use_sim_time": True,
                }],
            )
        )

    # ---------------- 3. 契约适配器（核心）----------------
    adapter_params = {
        "in_cloud_topic": adapter_cfg.get("in_cloud_topic", "/points"),
        "out_cloud_topic": matrix["contract"]["out"]["cloud"],
        "out_downsampled_topic": matrix["contract"]["out"]["cloud_downsampled"],
        "out_map_topic": matrix["contract"]["out"]["map"],
        "map_frame": matrix["contract"]["out"]["tf_map_to_odom"][0],
        "publish_2d_map": (publish_2d == "true") and bool(adapter_cfg.get("publish_2d_map", True)),
        "height_min": float(adapter_cfg.get("height_min", -0.5)),
        "height_max": float(adapter_cfg.get("height_max", 2.0)),
        "use_sim_time": True,
    }
    actions.append(
        Node(
            package="mapping_adapters",
            executable="mapping_contract_adapter.py",
            name="mapping_contract_adapter",
            output="screen",
            parameters=[adapter_params],
        )
    )

    # ---------------- 4. 稀疏度分层（可视化对比用）----------------
    actions.append(
        Node(
            package="mapping_adapters",
            executable="cloud_downsampler.py",
            name="cloud_downsampler",
            output="screen",
            condition=IfCondition(publish_downsample),
            parameters=[{
                "in_topic": "/points",
                "publish_levels": True,
                "use_sim_time": True,
            }],
        )
    )

    # ---------------- 5. 约束校验 ----------------
    self_odom = algorithm in matrix.get("constraints", {}).get("self_odometry_algorithms", [])
    if self_odom:
        actions.append(
            LogInfo(
                msg=(
                    f"[mapping] 提示：{algo_cfg['display_name']} 自带里程计，"
                    "请把顶层 launch 的 odom_source 设为 lio，"
                    "否则 odom->base 会出现两个发布者，TF 树直接崩。"
                )
            )
        )

    return actions


def make_mapping_launch_description(algorithm: str, extra_args=None):
    """生成某个算法的 LaunchDescription。algorithm 为 'none' 时只起适配节点。"""

    dl = [
        DeclareLaunchArgument(
            "external_launch", default_value="",
            description="外部算法 launch 文件的绝对路径；留空则自动在包内查找",
        ),
        DeclareLaunchArgument(
            "lidar_mode", default_value="points",
            choices=["points", "scan", "both"],
            description="是否额外切出 2D /scan（2D SLAM 需要）",
        ),
        DeclareLaunchArgument(
            "publish_2d_map", default_value="true",
            description="是否把点云投影成 2D 占据栅格发布到 /map",
        ),
        DeclareLaunchArgument(
            "publish_downsample_levels", default_value="true",
            description="是否发布多稀疏度点云（可视化对比用）",
        ),
        DeclareLaunchArgument(
            "start_external_algorithm", default_value="true",
            description="是否由本 launch 启动外部建图算法本体；"
                        "顶层已由 nav2_bringup 起 SLAM 时必须置 false，"
                        "否则会有两个 map->odom 发布者",
        ),
        DeclareLaunchArgument(
            "start_points_to_scan", default_value="true",
            description="是否由本 launch 启动 3D->2D 虚拟扫描；"
                        "顶层 sim.launch.py 已启动时应置 false，避免 /scan 双发布者",
        ),
    ]
    if extra_args:
        dl.extend(extra_args)
    dl.append(OpaqueFunction(function=setup_mapping, args=[algorithm]))
    return LaunchDescription(dl)
