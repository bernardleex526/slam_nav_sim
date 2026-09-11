#!/usr/bin/env python3
"""
spawn_robot.launch.py
=====================
把已生成的 robot_description 作为实体投入正在运行的 Gazebo 世界。

默认场景布置：
  - 出生点在 (spawn_x, spawn_y)，朝向 spawn_yaw
  - 出生高度自动取机器人 profile 推导出的站姿高度 + 余量，避免出生瞬间砸地

用法（Gazebo 已在运行时）：
  ros2 launch robot_descriptions spawn_robot.launch.py robot:=wheeled_quadruped
"""

import os

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def launch_setup(context, *args, **kwargs):
    robot_name = LaunchConfiguration("robot").perform(context)
    entity = LaunchConfiguration("entity").perform(context) or f"{robot_name}_sim"

    params_file = os.path.join(
        get_package_share_directory("robot_descriptions"), "config", "robot_params.yaml"
    )
    with open(params_file, "r") as f:
        robots = yaml.safe_load(f).get("robots", {})

    cfg = robots.get(robot_name, {})
    body_z = cfg.get("body_size", [0.6, 0.3, 0.2])[2]
    thigh = cfg.get("thigh_length", 0.2)
    shin = cfg.get("shin_length", thigh)
    wheel_r = cfg.get("wheel_radius", None)

    # ⚠️⚠️ spawn_z 必须是 0，不是 stand！
    # ------------------------------------------------------------------
    # URDF 里 base_footprint 是【根 link】，定义就是"地面投影原点，z 恒为 0"，
    # base_link 通过 fixed joint 挂在它下面 stand_height 处。
    # spawn_entity 的 -z 参数定位的是【模型的根 link】，也就是 base_footprint。
    #
    # 曾经的错误：spawn_z = stand（把 stand_height 当成 spawn 高度传进去），
    # 于是 base_footprint 被放到 stand 高度，base_link 就被顶到 2×stand。
    # 实测（simple / wheeled_quadruped）：
    #     odom -> base_footprint  z = 0.385   ← 整车悬空一个站姿高度
    #     odom -> base_link       z = 0.770   ← 两倍
    # 在【物理模式】下这还能自愈（重力把车拉回地面落地），
    # 但本项目现在跑的是 sim_kinematic_freeze 运动学模式：
    #     <gravity>false</gravity> + 接触参数全 0
    # z 完全由 spawn 决定、永不下落 ⇒ 机器人【永远悬空 stand 高度】。
    #
    # 后果链：
    #   雷达实际高度 = 0.385(悬空) + 0.385(URDF 标称) + mount_z ≈ 0.93m
    #   而 points_to_scan 的高度带是按 URDF 标称高度切的
    #   ⇒ 切出来的扫描几何与实际不符，墙体投影系统性偏移。
    #   这也是"点云几何整体错位"的根源。
    #
    # 正确做法：根 link 贴着地面 spawn，让 URDF 内部的 stand_height 偏移
    # 自己把 base_link 抬到站姿高度。z 恒为 0，与运动学模式自洽。
    spawn_z = 0.0

    # 保留 stand 变量用于日志输出（让人看清 URDF 标称站姿是多少），
    # 但【绝不再把它当 spawn 高度】。
    if cfg.get("external_model") and cfg.get("stand_height") is not None:
        stand = cfg["stand_height"]
    elif cfg.get("robot_type") == "wheeled_quadruped":
        stand = (wheel_r or 0.085) + thigh + body_z / 2.0
    elif cfg.get("robot_type") == "quadruped":
        stand = cfg.get("foot_radius", 0.025) + shin + thigh + body_z / 2.0
    elif cfg.get("robot_type") == "agv":
        stand = (wheel_r or 0.1) + body_z / 2.0
    else:
        stand = 0.3

    spawn = Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        name="spawn_entity",
        output="screen",
        arguments=[
            "-topic", "robot_description",
            "-entity", entity,
            "-x", LaunchConfiguration("spawn_x"),
            "-y", LaunchConfiguration("spawn_y"),
            "-z", str(round(spawn_z, 4)),
            "-Y", LaunchConfiguration("spawn_yaw"),
            "-timeout", "120.0",
        ],
    )

    return [
        LogInfo(msg=(
            f"[spawn_robot] {entity} 根 link(base_footprint) 在 z={spawn_z:.3f} 出生；"
            f"URDF 标称站姿 {stand:.3f} ⇒ base_link 将落在 z={spawn_z + stand:.3f}"
        )),
        spawn,
    ]


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument("robot", default_value="wheeled_quadruped"),
            DeclareLaunchArgument("entity", default_value="",
                                  description="实体名；留空则用 <robot>_sim"),
            DeclareLaunchArgument("spawn_x", default_value="0.0"),
            DeclareLaunchArgument("spawn_y", default_value="0.0"),
            DeclareLaunchArgument("spawn_yaw", default_value="0.0"),
            OpaqueFunction(function=launch_setup),
        ]
    )
