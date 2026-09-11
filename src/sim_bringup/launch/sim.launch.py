#!/usr/bin/env python3
"""
sim.launch.py — 仿真环境总入口
==============================
一条命令把整个世界串起来：

    世界 + 机器人 + 传感器 + 稳定云台 + 建图 + 导航 + 可视化 + 指标

    ros2 launch sim_bringup sim.launch.py robot:=wheeled_quadruped world:=medium

常用组合：
    # 只建图，看地图质量
    ros2 launch sim_bringup sim.launch.py world:=complex mapping:=fast_lio

    # 建图 + 导航同时跑（SLAM 提供 map->odom）
    ros2 launch sim_bringup sim.launch.py world:=medium mapping:=slam_toolbox navigation:=nav2

    # 纯导航，换规划器对比（需要先保存一张地图）
    ros2 launch sim_bringup sim.launch.py world:=simple navigation:=nav2 \
        localization:=amcl map:=$HOME/maps/simple.yaml \
        global_planner:=smac_hybrid local_planner:=mppi

    # 无头批量跑
    ros2 launch sim_bringup sim.launch.py world:=complex headless:=true metrics:=true

这个 launch 同时承担"参数一致性守门人"的角色：odom 来源冲突、定位模式与地图
不匹配、算法自带里程计却没切 odom_source —— 这些都直接在这里报错拦掉。
理由很简单：这几类错误在运行期都表现为"某个东西莫名其妙不动了"，
排查成本远高于在启动时直接告诉你。
"""

import json
import os
import subprocess

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    LogInfo,
    OpaqueFunction,
    TimerAction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _inc(pkg, launch_file, args):
    return IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory(pkg), "launch", launch_file)
        ),
        launch_arguments=args.items(),
    )


def launch_setup(context, *args, **kwargs):
    g = lambda n: LaunchConfiguration(n).perform(context)  # noqa: E731

    robot = g("robot")
    world = g("world")
    mapping = g("mapping")
    navigation = g("navigation")
    odom_source = g("odom_source")
    localization = g("localization")
    headless = g("headless")
    viz = g("viz")
    metrics = g("metrics")
    run_name = g("run_name")
    sensor_profile = g("sensor_profile")
    use_gimbal = g("use_gimbal")
    map_yaml = g("map")

    is_headless = headless == "true"
    show_viz = (viz == "true") and not is_headless

    # ------------------------------------------------------------------
    # 1. 参数一致性守门
    # ------------------------------------------------------------------
    worlds_share = get_package_share_directory("sim_worlds")
    meta_path = os.path.join(worlds_share, "maps", f"{world}_meta.json")
    meta = {}
    if os.path.isfile(meta_path):
        with open(meta_path) as f:
            meta = json.load(f)
    spawn = meta.get("recommended_spawn", {"x": 0.0, "y": 0.0, "yaw": 0.0})

    problems = []
    if odom_source == "lio" and mapping == "none":
        problems.append(
            "odom_source:=lio 但 mapping:=none —— 没有任何节点会发布 odom->base 的 TF，"
            "机器人不会动。要么开建图，要么把 odom_source 改回 gt。"
        )
    if localization == "amcl" and not map_yaml:
        problems.append(
            "localization:=amcl 但没有提供 map:=<yaml 路径>。"
            "AMCL 需要一张已保存的地图。"
        )
    if (navigation == "nav2" and localization == "slam"
            and mapping not in ("none", "slam_toolbox")):
        problems.append(
            f"navigation:=nav2 + localization:=slam 时，SLAM 由 nav2_bringup 独占启动，"
            f"它只会拉起 slam_toolbox。此时 mapping:={mapping} 不会被启动 —— "
            "想用 fast_lio / lio_sam 建图，请用 mapping:=<算法> navigation:=none，"
            "建好图保存后再用 localization:=amcl 跑导航。"
        )
    if localization == "slam" and mapping == "none" and navigation != "none":
        problems.append(
            "localization:=slam 但 mapping:=none —— 没有节点发布 map->odom，"
            "Nav2 会一直等 TF。请改用 localization:=amcl，或先启动建图。"
        )
    if problems:
        return [
            LogInfo(msg="\n[sim] 参数冲突，已中止启动：\n"
                        + "\n".join(f"  ✗ {p}" for p in problems)
                        + "\n")
        ]

    # ------------------------------------------------------------------
    # 2. 运行环境守门：检测是否已有另一个仿真/批量实验在跑
    # ------------------------------------------------------------------
    # 为什么必须查（本项目血泪教训）：
    #   从宿主机强杀后台任务是杀不干净的 —— `ros2 launch` 的进程树会逃逸成活
    #   孤儿，继续往话题里灌数据。两套环境同时在跑时：
    #     · /map 会有 2 个 publisher（两个 slam_toolbox 抢发 map->odom）
    #     · /gimbal_stabilizer ×3 抢发雷达外挂 TF
    #     · /points_to_scan ×2 抢发 /scan
    #   后果：建图结果里出现【大片场外幽灵墙】、真值里程计报出机器人没去过的
    #   位置、同一实验两次跑结果差异巨大且无法复现。
    #   这些幽灵节点不占 CPU、不报错、日志里完全看不到 —— 只能在启动前主动查。
    #
    # ⚠️ 策略选择：只【警告】不【中止】。
    #    原因：批量实验是「起一个 → 跑完 → 杀掉 → 起下一个」的循环，
    #    两个实验之间只有 8s 冷却。若上一个 gzserver 尚未完全退出，
    #    「直接中止启动」会把正常流程误伤成全线失败。
    #    而这里若只是警告，真正的串扰仍会被后续的 /map publisher 检查暴露出来。
    #    所以：守门负责【把问题喊出来】，不负责阻断流程。
    warnings = []
    if is_headless:
        try:
            _ps = subprocess.run(["ps", "-eo", "cmd="],
                                 capture_output=True, text=True, timeout=5).stdout
        except Exception:
            _ps = ""
        _n_gz = sum(1 for ln in _ps.splitlines()
                    if "gzserver" in ln and "-s libgazebo_ros" in ln)
        if _n_gz > 0:
            warnings.append(
                f"检测到已有 {_n_gz} 个 gzserver 实例在运行。"
                "若是上一轮实验的残留（未随 launch 正常退出），会导致 DDS 串扰："
                "/map 出现多个 publisher、重名节点抢发 TF/scan，"
                "建图结果不可信且不可复现。"
                "若是连续实验的正常间隔，可忽略；若结果异常，先跑 bash tools/clean_sim.sh。"
            )

    actions = []

    # ------------------------------------------------------------------
    # 2. Gazebo 世界
    # ------------------------------------------------------------------
    actions.append(_inc("sim_worlds", "spawn_world.launch.py", {
        "world": world,
        "headless": headless,
        "gui": "true",
    }))

    # ------------------------------------------------------------------
    # 3. 机器人（URDF 加载 + 关节状态 + 稳定云台）
    # ------------------------------------------------------------------
    actions.append(_inc("robot_descriptions", "robot_description.launch.py", {
        "robot": robot,
        "sensor_profile": sensor_profile,
        "use_gimbal": use_gimbal,
        "odom_source": odom_source,
    }))

    # 稍等再 spawn：gzserver 需要一点时间把 ROS 插件加载完
    actions.append(TimerAction(period=3.0, actions=[
        _inc("robot_descriptions", "spawn_robot.launch.py", {
            "robot": robot,
            "spawn_x": str(spawn.get("x", 0.0)),
            "spawn_y": str(spawn.get("y", 0.0)),
            "spawn_yaw": str(spawn.get("yaw", 0.0)),
        }),
    ]))

    # ------------------------------------------------------------------
    # 4. 动态障碍（medium / complex 才有）
    # ------------------------------------------------------------------
    n_dyn = int(meta.get("dynamic_obstacles", 0) or 0)
    if n_dyn > 0:
        actions.append(TimerAction(period=6.0, actions=[
            Node(
                package="sim_worlds",
                executable="dynamic_obstacle_driver.py",
                name="dynamic_obstacle_driver",
                output="screen",
                parameters=[{
                    "use_sim_time": True,
                    "max_models": n_dyn,
                    "world": world,
                    "amplitude": 1.2,
                    "period": 12.0,
                }],
            ),
        ]))

    # ------------------------------------------------------------------
    # 5. 虚拟 2D 扫描：导航需要 LaserScan 喂 costmap
    # ------------------------------------------------------------------
    # 由本 launch 独占负责，mapping 那边传 lidar_mode:=points 避免重复启动。
    if navigation == "nav2":
        actions.append(Node(
            package="mapping_adapters",
            executable="points_to_scan.py",
            name="points_to_scan",
            output="screen",
            parameters=[{
                "use_sim_time": True,
                "in_topic": "/points",
                "out_topic": "/scan",
            }],
        ))

    # ------------------------------------------------------------------
    # 6. 建图
    # ------------------------------------------------------------------
    if mapping != "none":
        actions.append(TimerAction(period=4.0, actions=[
            _inc("mapping_adapters", "mapping.launch.py", {
                "mapping": mapping,
                "lidar_mode": "points",
                "publish_2d_map": "true",
                "publish_downsample_levels": "true",
                # 导航开启时 points_to_scan 由本 launch 独占启动（见上面第 5 节），
                # 这里必须关掉，否则 /scan 会有两个发布者
                "start_points_to_scan": "false" if navigation == "nav2" else "true",
                # ⚠️ nav2_bringup 在 slam:=True 时会自己启动 slam_toolbox。
                # 如果这里再起一个，就有【两个 map->odom 发布者】，
                # TF 树被污染、导航大面积 abort（实测踩了整整一轮）。
                # 所以导航+SLAM 组合下，SLAM 归 nav2_bringup 独占。
                "start_external_algorithm":
                    "false" if (navigation == "nav2" and localization == "slam") else "true",
            }),
        ]))

    # ------------------------------------------------------------------
    # 7. 导航
    # ------------------------------------------------------------------
    if navigation == "nav2":
        actions.append(TimerAction(period=6.0, actions=[
            _inc("nav2_config", "nav2.launch.py", {
                "robot": robot,
                "global_planner": g("global_planner"),
                "local_planner": g("local_planner"),
                "odom_source": odom_source,
                "localization": localization,
                "map": map_yaml,
                "initial_x": str(spawn.get("x", 0.0)),
                "initial_y": str(spawn.get("y", 0.0)),
                "initial_yaw": str(spawn.get("yaw", 0.0)),
            }),
        ]))

    # ------------------------------------------------------------------
    # 8. 可视化
    # ------------------------------------------------------------------
    view = "navigation" if navigation == "nav2" else "mapping"
    if show_viz:
        actions.append(TimerAction(period=8.0, actions=[
            _inc("sim_viz", "viz.launch.py", {"view": view}),
        ]))

    # ------------------------------------------------------------------
    # 9. 指标采集
    # ------------------------------------------------------------------
    if metrics == "true":
        actions.append(TimerAction(period=8.0, actions=[
            _inc("sim_metrics", "metrics.launch.py", {
                "run_name": run_name or f"{robot}_{world}_{mapping}_{g('local_planner')}",
                "output_dir": g("metrics_output_dir"),
                "target_frame": "map" if (mapping != "none" or localization == "amcl") else "odom",
            }),
        ]))

    # ------------------------------------------------------------------
    # 10. 接口自检（可选，延迟到系统起来之后）
    # ------------------------------------------------------------------
    if g("check") == "true":
        actions.append(TimerAction(period=25.0, actions=[
            Node(
                package="robot_interface_spec",
                executable="check_interface.py",
                name="check_interface",
                output="screen",
                arguments=["", "15.0"],
                parameters=[{"use_sim_time": True}],
            ),
        ]))

    actions.insert(0, LogInfo(msg=(
        f"\n[sim] robot={robot}  world={world}  mapping={mapping}  navigation={navigation}\n"
        f"      odom_source={odom_source}  localization={localization}  "
        f"gimbal={use_gimbal}  headless={is_headless}\n"
        f"      出生点=({spawn.get('x',0):.2f}, {spawn.get('y',0):.2f})  "
        f"动态障碍={n_dyn} 个\n"
        + "".join(f"      ⚠️ {w}\n" for w in warnings)
    )))

    return actions


def generate_launch_description():
    T = ["true", "false"]
    return LaunchDescription([
        # ---- 核心变量（对应 SPEC.md 第 4.3 节）----
        DeclareLaunchArgument("robot", default_value="wheeled_quadruped",
                              description="机器人 profile：wheeled_quadruped / quadruped / go2w / agv"),
        DeclareLaunchArgument("world", default_value="simple",
                              choices=["simple", "medium", "complex"],
                              description="测试地图难度"),
        DeclareLaunchArgument("mapping", default_value="none",
                              choices=["none", "fast_lio", "lio_sam", "rtabmap", "slam_toolbox"],
                              description="建图算法；none = 不建图"),
        DeclareLaunchArgument("navigation", default_value="none", choices=["none", "nav2"]),
        DeclareLaunchArgument("odom_source", default_value="gt", choices=["gt", "lio"],
                              description="里程计来源：gt=仿真真值，lio=算法估计"),
        DeclareLaunchArgument("sensor_profile", default_value="",
                              description="覆盖机器人 profile 的传感器档位；留空用默认"),
        DeclareLaunchArgument("use_gimbal", default_value="true", choices=T,
                              description="虚拟稳定云台；false = 未减震对照"),
        # ---- 导航 ----
        DeclareLaunchArgument("global_planner", default_value="smac_2d",
                              choices=["navfn", "smac_2d", "smac_hybrid"]),
        DeclareLaunchArgument("local_planner", default_value="dwb",
                              choices=["dwb", "rpp", "mppi", "teb"]),
        DeclareLaunchArgument("localization", default_value="slam", choices=["slam", "amcl"]),
        DeclareLaunchArgument("map", default_value="",
                              description="localization:=amcl 时的地图 yaml 路径"),
        # ---- 运行模式 ----
        DeclareLaunchArgument("headless", default_value="false", choices=T,
                              description="无头模式，批量测试用"),
        DeclareLaunchArgument("viz", default_value="true", choices=T),
        DeclareLaunchArgument("metrics", default_value="false", choices=T,
                              description="启动指标采集"),
        DeclareLaunchArgument("check", default_value="false", choices=T,
                              description="启动后做一次接口自检"),
        DeclareLaunchArgument("run_name", default_value="",
                              description="本次运行名，用于指标输出分目录"),
        DeclareLaunchArgument("metrics_output_dir", default_value="~/.ros/sim_eval"),
        OpaqueFunction(function=launch_setup),
    ])
