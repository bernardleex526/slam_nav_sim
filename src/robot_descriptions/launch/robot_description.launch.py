#!/usr/bin/env python3
"""
robot_description.launch.py
===========================
URDF 加载管线：把 robot_params.yaml 里的一个 profile 展开成可用的机器人描述。

对应需求 #1：「创建存放 URDF 的空间，可直接解析机器人的 URDF 将其模型投入
仿真空间」。

这里做的事：
  1. 读 robot_params.yaml 的 profile -> 得到尺寸 / 质量 / 传感器挂载 / 档位
  2. 把这些值作为 xacro args 喂给参数化模板 -> 生成 URDF
  3. 起 robot_state_publisher（发静态 TF）、关节状态桥、可选稳定云台

换机器人 = 改 robot:= 参数。这里的逻辑一行都不用动。
"""

import os
import shlex

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import Command, FindExecutable, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def _fmt(v):
    return " ".join(str(x) for x in v)


def load_profile(robot_name: str):
    params_file = os.path.join(
        get_package_share_directory("robot_descriptions"), "config", "robot_params.yaml"
    )
    with open(params_file, "r") as f:
        data = yaml.safe_load(f)

    robots = data.get("robots", {})
    if robot_name not in robots:
        raise RuntimeError(
            f"robot_params.yaml 中没有 profile '{robot_name}'。"
            f"可选：{', '.join(robots.keys())}"
        )
    return robots[robot_name]


def launch_setup(context, *args, **kwargs):
    robot_name = LaunchConfiguration("robot").perform(context)
    cfg = load_profile(robot_name)

    sensor_profile = LaunchConfiguration("sensor_profile").perform(context) or cfg.get(
        "sensor_profile", "lidar3d_imu"
    )
    use_gimbal = LaunchConfiguration("use_gimbal").perform(context)
    odom_source = LaunchConfiguration("odom_source").perform(context) or cfg.get(
        "odom_source", "gt"
    )
    # 关节刚性化：默认开。只有在挂真实步态控制器（非 planar_move）时才该关掉。
    rigid_joints = LaunchConfiguration("rigid_joints").perform(context).lower() in (
        "1", "true", "yes", "on")
    keep_joints = [
        s.strip() for s in LaunchConfiguration("keep_joints").perform(context).split(",")
        if s.strip()
    ]

    description_pkg = cfg.get("description_package", "robot_descriptions")
    xacro_rel = cfg["description_xacro"]
    xacro_file = os.path.join(get_package_share_directory(description_pkg), xacro_rel)

    if not os.path.isfile(xacro_file):
        raise RuntimeError(f"机器人描述文件不存在：{xacro_file}")

    mounts = cfg.get("sensor_mounts", {})
    lidar = mounts.get("lidar", {"xyz": [0.0, 0.0, 0.0], "rpy": [0.0, 0.0, 0.0]})
    camera = mounts.get("camera", {"xyz": [0.0, 0.0, 0.0], "rpy": [0.0, 0.0, 0.0]})
    imu = mounts.get("imu", {"xyz": [0.0, 0.0, 0.0], "rpy": [0.0, 0.0, 0.0]})

    # 从 profile 拼出 xacro 参数。
    #
    # external_model=true 的 profile（如 go2w）几何来自上游官方 URDF，
    # 我们的模板不读 body_x/hip_x/thigh_len 这些 arg（它只负责 include +
    # 注入传感器/插件）。多传无害但会误导读者以为改得动尺寸，
    # 所以这类 profile 只传传感器/云台/里程计这几个真正生效的 arg。
    common_args = {
        "robot_name": robot_name,
        "sensor_profile": sensor_profile,
        "use_gimbal": use_gimbal,
        "odom_source": odom_source,
        "lidar_xyz": _fmt(lidar["xyz"]),
        "lidar_rpy": _fmt(lidar["rpy"]),
        "camera_xyz": _fmt(camera["xyz"]),
        "camera_rpy": _fmt(camera["rpy"]),
        "imu_xyz": _fmt(imu["xyz"]),
        "imu_rpy": _fmt(imu["rpy"]),
    }

    if cfg.get("external_model"):
        xacro_args = common_args
        # 站立高度：外部模型必须显式给（模板无法从尺寸推出来）
        xacro_args["stand_height"] = cfg.get("stand_height", 0.30)
        # 允许覆盖上游 URDF 的查找路径（默认按包名找）
        if cfg.get("upstream_urdf"):
            xacro_args["go2w_urdf"] = cfg["upstream_urdf"]
    else:
        xacro_args = dict(common_args)
        xacro_args.update({
            "body_x": cfg.get("body_size", [0.6, 0.3, 0.2])[0],
            "body_y": cfg.get("body_size", [0.6, 0.3, 0.2])[1],
            "body_z": cfg.get("body_size", [0.6, 0.3, 0.2])[2],
            "body_mass": cfg.get("body_mass", 10.0),
            "hip_x": cfg.get("hip_offset", [0.25, 0.15])[0],
            "hip_y": cfg.get("hip_offset", [0.25, 0.15])[1],
            "thigh_len": cfg.get("thigh_length", 0.2),
            "shin_len": cfg.get("shin_length", cfg.get("thigh_length", 0.2)),
            "foot_radius": cfg.get("foot_radius", 0.025),
            "wheel_radius": cfg.get("wheel_radius", 0.085),
            "wheel_width": cfg.get("wheel_width", 0.05),
        })

    # ---- 拼 xacro 命令行 ----
    #
    # 必须对每个 token 做 shell 转义。
    # 传感器挂载点的值是 "0.18 0.0 0.16" 这种含空格的串 —— 不转义就会被 shell
    # 拆成 3 个参数，xacro 收到额外位置参数直接报
    # 「expected exactly one input file」，结果是 robot_state_publisher 起不来、
    # 整棵 TF 树全空。报错信息完全没提空格，很容易误判成 xacro 用法问题。
    xacro_tokens = [shlex.quote(xacro_file)]
    for k, v in xacro_args.items():
        xacro_tokens.append(shlex.quote(f"{k}:={v}"))

    # ---- 关节刚性化 ----
    #
    # 展开完 xacro 后再过一道 rigidify_joints.py，把腿/轮这类"无力矩源"的
    # revolute/continuous 关节锁成 fixed。原因见 scripts/rigidify_joints.py：
    # planar_move 只写平面运动学、不提供任何支撑力，被动关节在重力下会塌陷
    # → 车体下沉到 z≈-0.23 → 底盘推不动 → 割草机探索卡在 TURN 一步都走不出去。
    #
    # ⚠️ 这里必须显式用 bash -c 包一层。
    # launch 的 Command 是把 token 拼成一个字符串后【直接 execve】，没有 shell
    # 参与，所以管道符 '|' 会被当成 xacro 的普通参数传进去，报的错是
    # 「expected exactly one input file as argument」—— 完全指不到管道上。
    # 想用管道就必须自己把命令塞给 bash -c。
    if rigid_joints:
        rigid_script = os.path.join(
            get_package_share_directory("robot_descriptions"), "scripts", "rigidify_joints.py"
        )
        inner = "xacro %s" % " ".join(xacro_tokens)
        pipeline = "%s | python3 %s --quiet /dev/stdin /dev/stdout" % (
            inner, shlex.quote(rigid_script))
        if keep_joints:
            pipeline += " --keep %s" % shlex.quote(",".join(keep_joints))
        xacro_cmd = ["bash -c ", shlex.quote(pipeline)]
    else:
        xacro_cmd = ["xacro "] + [t + " " for t in xacro_tokens]

    robot_description = ParameterValue(Command(xacro_cmd), value_type=str)

    nodes = [
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            name="robot_state_publisher",
            output="screen",
            parameters=[
                {
                    # use_sim_time 是必须的：本节点发布的 TF 会被 slam_toolbox /
                    # Nav2 按【仿真时间】去查。不设的话 TF 带的是墙钟时间戳
                    # （约 1.7e9 秒），而传感器消息是仿真时间（约 5 秒），
                    # tf2 的 MessageFilter 会以
                    #   "timestamp earlier than all the data in the transform cache"
                    # 为由丢掉【每一帧】。症状是建图节点活着、话题有数据、
                    # 但地图永远是空的 —— 极难定位。
                    "robot_description": robot_description,
                    "publish_frequency": 100.0,
                    "frame_prefix": "",
                    "use_sim_time": True,
                }
            ],
        ),
        # 关节状态桥：planar_move 模式下没有任何东西发布关节状态，
        # 缺了它传感器 frame 的 TF 就不存在（见脚本内注释）。
        Node(
            package="robot_descriptions",
            executable="sim_joint_state_bridge.py",
            name="sim_joint_state_bridge",
            output="screen",
            parameters=[{"publish_rate": 50.0, "use_sim_time": True}],
        ),
        # 虚拟稳定云台
        Node(
            package="robot_descriptions",
            executable="gimbal_stabilizer.py",
            name="gimbal_stabilizer",
            output="screen",
            condition=IfCondition(use_gimbal),
            parameters=[
                {
                    "enabled": True,
                    "world_frame": "odom",
                    "base_frame": "base_link",
                    "stabilized_frame": "lidar_stabilized",
                    "mount_xyz": list(lidar["xyz"]),
                    "mount_rpy": list(lidar["rpy"]),
                    "publish_rate": 100.0,
                    "lag_alpha": LaunchConfiguration("gimbal_lag_alpha"),
                    # 同上：本节点动态发布 lidar_stabilized 的 TF，
                    # 时间戳必须走仿真时间，否则 slam_toolbox 会丢掉所有激光帧
                    "use_sim_time": True,
                }
            ],
        ),
    ]

    return nodes


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "robot",
                default_value="wheeled_quadruped",
                description="robot_params.yaml 中的机器人 profile 名",
            ),
            DeclareLaunchArgument(
                "sensor_profile",
                default_value="",
                description="覆盖 profile 里的传感器档位；留空则用 profile 默认值",
            ),
            DeclareLaunchArgument(
                "use_gimbal",
                default_value="true",
                description="是否启用虚拟稳定云台（lidar_stabilized frame）",
            ),
            DeclareLaunchArgument(
                "gimbal_lag_alpha",
                default_value="1.0",
                description="云台响应滞后：1.0=理想瞬时补偿，越小越滞后",
            ),
            DeclareLaunchArgument(
                "odom_source",
                default_value="",
                description="gt 或 lio；留空则用 profile 默认值",
            ),
            DeclareLaunchArgument(
                "rigid_joints",
                default_value="true",
                description=(
                    "把腿/轮的被动关节锁成 fixed。planar_move 无力矩源，不锁会塌陷下沉。"
                    "挂真实步态控制器时传 false。"
                ),
            ),
            DeclareLaunchArgument(
                "keep_joints",
                default_value="",
                description="逗号分隔的关节名，刚性化时保持原类型（如轮子做视觉动画）",
            ),
            OpaqueFunction(function=launch_setup),
        ]
    )
