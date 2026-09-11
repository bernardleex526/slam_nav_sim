#!/usr/bin/env python3
"""
compose_nav2_params
===================
按「机器人 + 全局规划器 + 局部规划器」三层合成一份 Nav2 参数文件。

这是需求 #6 的核心机制：
  「对于不同的机器人可以通过在仿真中进行参数更改来实现 nav2 的全局规划器和
    局部规划器的参数适配」

设计要点（也是这个脚本存在的全部理由）：

  1. **单一真源**。机器人的 footprint、速度上限、加速度上限来自
     robot_descriptions/config/robot_params.yaml —— 和生成 URDF 用的是同一份。
     改一处，机器人和导航参数同时跟着走，不可能出现"URDF 说车宽 0.6m、
     costmap 说 0.4m"这种对不上的情况。

  2. **分层不重叠**。base（通用）+ 机器人（几何/运动学）+ 规划器（算法参数）
     各自只管自己那一层，深合并。改规划器不会碰机器人参数，反之亦然。

  3. **组合爆炸可控**。2 机器人 x 3 全局 x 4 局部 = 24 种组合，
     但只需要维护 2 + 3 + 4 = 9 个文件，而不是 24 份完整 yaml。

用法：
  python3 compose_nav2_params.py \
      --robot wheeled_quadruped \
      --global-planner smac_2d \
      --local-planner dwb \
      --odom-source gt \
      --out /tmp/nav2_params.yaml
"""

import argparse
import copy
import os
import sys

import yaml
import json
import math

# 各局部规划器的速度参数命名完全不同 —— 这是 Nav2 一个很坑的现实。
# 必须把机器人 profile 的速度上限注入到【控制器插件自己的】参数段里，
# 写在 controller_server 下是无效的（ROS2 会静默忽略未声明参数），
# 后果是"速度限制看着配了、实际完全没生效"。
CONTROLLER_SPEED_PARAMS = {
    "dwb": {
        "max_vel_x":     ("max", "x"),
        "min_vel_x":     ("min", "x"),
        "max_vel_theta": ("max", "theta"),
        "min_speed_xy":  ("zero", None),
        "min_speed_theta": ("zero", None),
    },
    "rpp": {
        "desired_linear_vel":            ("max", "x"),
        "rotate_to_heading_angular_vel": ("max", "theta"),
    },
    "mppi": {
        "vx_max": ("max", "x"),
        "vx_min": ("min", "x"),
        "wz_max": ("max", "theta"),
    },
    "teb": {
        "max_vel_x":         ("max", "x"),
        "max_vel_x_backwards": ("absmin", "x"),
        "max_vel_theta":     ("max", "theta"),
        "acc_lim_x":         ("acc", "x"),
        "acc_lim_theta":     ("acc", "theta"),
    },
}

GLOBAL_PLANNERS = ["navfn", "smac_2d", "smac_hybrid"]
LOCAL_PLANNERS = ["dwb", "rpp", "mppi", "teb"]


def _share(pkg):
    from ament_index_python.packages import get_package_share_directory

    return get_package_share_directory(pkg)


def _load_slam_toolbox_params() -> dict:
    """读取 mapping_adapters 里的 slam_toolbox_params.yaml，取出 ros__parameters 段。

    为什么放在 mapping_adapters 而不是 nav2_config：
      slam_toolbox 是【建图算法】，它的参数属于建图适配层的职责。
      nav2_config 只在合成时把它拉过来注入，保持"建图参数归建图、导航参数归导航"
      的边界。这样以后换 SLAM 算法，也是改 mapping_adapters 一处。
    """
    try:
        share = _share("mapping_adapters")
    except Exception:
        return {}
    path = os.path.join(share, "config", "slam_toolbox_params.yaml")
    if not os.path.isfile(path):
        return {}
    try:
        with open(path, "r") as f:
            raw = yaml.safe_load(f) or {}
    except Exception:
        return {}
    return (raw.get("slam_toolbox", {}) or {}).get("ros__parameters", {}) or {}


def deep_merge(base: dict, overlay: dict) -> dict:
    """递归合并。字典递归，列表整体替换（规划器插件列表本来就是要整体换的）。"""
    out = copy.deepcopy(base)
    for k, v in (overlay or {}).items():
        if k in out and isinstance(out[k], dict) and isinstance(v, dict):
            out[k] = deep_merge(out[k], v)
        else:
            out[k] = copy.deepcopy(v)
    return out


def build_robot_overlay(robot_cfg: dict, odom_source: str) -> dict:
    """从 robot_params.yaml 的 profile 推出导航侧参数。

    每个派生量都写清为什么这么推 —— 这些关系不理解的话，换机器人时
    你只能靠试错去调 costmap，非常低效。
    """
    fp = robot_cfg.get("footprint", {})
    max_vel = robot_cfg.get("max_vel", {})
    min_vel = robot_cfg.get("min_vel", {})
    acc = robot_cfg.get("acc_lim", {})
    dec = robot_cfg.get("dec_lim", acc)

    if fp.get("shape") == "circle":
        radius = float(fp.get("radius", 0.35))
        # polygon 用外接正方形近似圆，略保守
        footprint_poly = (
            f"[[{radius:.3f}, {radius:.3f}], [{radius:.3f}, {-radius:.3f}], "
            f"[-{radius:.3f}, {-radius:.3f}], [-{radius:.3f}, {radius:.3f}]]"
        )
        inscribed = radius
    else:
        footprint_poly = fp.get("vertices", "[[0.4,0.3],[0.4,-0.3],[-0.4,-0.3],[-0.4,0.3]]")
        # 内切半径：取到最近边的距离，近似用最小 |y| 或 |x| 顶点
        try:
            pts = json.loads(footprint_poly)
            if len(pts) < 3 or any(len(p) != 2 or any(not isinstance(v, (int, float)) or
                    not math.isfinite(v) for v in p) for p in pts):
                raise ValueError("Footprint requires at least three finite [x,y] points")
            inscribed = min(min(abs(p[0]), abs(p[1])) for p in pts)
        except (TypeError, ValueError) as exc:
            raise ValueError("Invalid footprint JSON polygon") from exc

    # 膨胀半径：至少要大于内切半径，否则机器人贴墙时会判定为"可通行"
    # 这里取 内切半径 + 20cm 安全余量，是可解释的起点，不是拍脑袋
    inflation_radius = round(inscribed + 0.20, 3)
    # 代价衰减长度：略小于膨胀半径，保证高代价区集中在障碍附近
    cost_scaling = round(max(2.0, inflation_radius * 10.0), 2)

    return {
        "local_costmap": {
            "local_costmap": {
                "ros__parameters": {"footprint": footprint_poly},
            }
        },
        "global_costmap": {
            "global_costmap": {
                "ros__parameters": {"footprint": footprint_poly},
            }
        },
        "controller_server": {
            "ros__parameters": {
                "min_vel_x": float(min_vel.get("x", 0.0)),
                "max_vel_x": float(max_vel.get("x", 1.0)),
                "max_vel_theta": float(max_vel.get("theta", 1.5)),
                "min_speed_xy": float(min_vel.get("x", 0.0)),
                "max_speed_xy": float(max_vel.get("x", 1.0)),
                "min_speed_theta": float(min_vel.get("theta", -1.5)),
            }
        },
        "velocity_smoother": {
            "ros__parameters": {
                # 速度/加速度上限也必须来自 robot_params.yaml，否则会出现
                # "规划器知道车只能跑 1.5m/s、速度平滑器却按 2.5m/s 放行"
                # 这种内部矛盾 —— 表现是过弯甩出去，且极难归因。
                "max_velocity": [float(max_vel.get("x", 1.0)), 0.0,
                                 float(max_vel.get("theta", 1.5))],
                "min_velocity": [float(min_vel.get("x", 0.0)), 0.0,
                                 float(min_vel.get("theta", -1.5))],
                "max_accel": [float(acc.get("x", 1.0)), 0.0,
                              float(acc.get("theta", 2.0))],
                # 减速度在 Nav2 里约定为负值
                "max_decel": [-abs(float(dec.get("x", 1.0))), 0.0,
                              -abs(float(dec.get("theta", 2.0)))],
            }
        },
        "local_costmap_plugins": {
            "inflation_layer": {
                "inflation_radius": inflation_radius,
                "cost_scaling_factor": cost_scaling,
            }
        },
        "global_costmap_plugins": {
            "inflation_layer": {
                "inflation_radius": inflation_radius,
                "cost_scaling_factor": cost_scaling,
            }
        },
        "odom_topic": "/odom" if odom_source == "gt" else "/odom",
        "derived": {
            "footprint": footprint_poly,
            "inscribed_radius": inscribed,
            "inflation_radius": inflation_radius,
            "cost_scaling_factor": cost_scaling,
            "note": "footprint 与速度上限来自 robot_params.yaml，改机器人自动生效",
        },
    }


def inject_controller_speeds(merged: dict, local_planner: str, robot_cfg: dict) -> dict:
    """把机器人 profile 的速度上限写进当前控制器插件的参数段。

    DWB 用 max_vel_x/min_vel_x/max_vel_theta，
    RPP 用 desired_linear_vel/rotate_to_heading_angular_vel，
    MPPI 用 vx_max/vx_min/wz_max，TEB 又是另一套。
    所以必须按规划器查表注入，不能写一个通用字段名了事。
    """
    max_vel = robot_cfg.get("max_vel", {})
    min_vel = robot_cfg.get("min_vel", {})
    acc = robot_cfg.get("acc_lim", {})

    cs = merged.get("controller_server", {}).get("ros__parameters", {})
    plugins = cs.get("controller_plugins") or []
    if not plugins:
        return merged
    plugin_name = plugins[0]
    sect = cs.get(plugin_name)
    if not isinstance(sect, dict):
        return merged

    sources = {
        "max": lambda ax: float(max_vel.get(ax, 1.0)),
        "min": lambda ax: float(min_vel.get(ax, 0.0)),
        # 速度"幅值"类参数不能为负，取绝对值的意义是"允许的最小速度大小"
        "absmin": lambda ax: abs(float(min_vel.get(ax, 0.0))),
        "acc": lambda ax: float(acc.get(ax, 1.0)),
        "zero": lambda ax: 0.0,
    }
    for key, (kind, axis) in CONTROLLER_SPEED_PARAMS.get(local_planner, {}).items():
        if key in sect:                     # 只覆盖插件里确实存在的字段
            sect[key] = sources[kind](axis)
    return merged


def main():
    ap = argparse.ArgumentParser(description="合成 Nav2 参数文件")
    ap.add_argument("--robot", required=True, help="robot_params.yaml 中的 profile 名")
    ap.add_argument("--global-planner", default="smac_2d", choices=GLOBAL_PLANNERS)
    ap.add_argument("--local-planner", default="dwb", choices=LOCAL_PLANNERS)
    ap.add_argument("--odom-source", default="gt", choices=["gt", "lio"])
    ap.add_argument("--localization", default="slam", choices=["slam", "amcl"])
    ap.add_argument("--out", required=True, help="输出文件路径")
    ap.add_argument("--print-derived", action="store_true", help="打印派生参数，便于核对")
    args = ap.parse_args()

    nav2_share = _share("nav2_config")
    robot_share = _share("robot_descriptions")

    # ---- 1. 读机器人 profile（与生成 URDF 用的是同一份）----
    with open(os.path.join(robot_share, "config", "robot_params.yaml"), "r") as f:
        robots = yaml.safe_load(f).get("robots", {})
    if args.robot not in robots:
        print(f"[compose_nav2_params] 未知机器人 '{args.robot}'，可选：{list(robots)}", file=sys.stderr)
        sys.exit(2)
    robot_overlay = build_robot_overlay(robots[args.robot], args.odom_source)

    # ---- 2. 读三层配置 ----
    def load(rel):
        p = os.path.join(nav2_share, rel)
        if not os.path.isfile(p):
            return {}
        with open(p, "r") as f:
            return yaml.safe_load(f) or {}

    base = load("config/base/nav2_params.yaml")
    g_planner = load(f"config/planners/{args.global_planner}.yaml")
    l_planner = load(f"config/controllers/{args.local_planner}.yaml")

    # ---- 3. 深合并 ----
    merged = deep_merge(base, g_planner)
    merged = deep_merge(merged, l_planner)

    # 机器人几何 -> 注入到 costmap 的 inflation_layer 参数里
    for scope in ("local_costmap", "global_costmap"):
        plugins = merged.get(scope, {}).get(scope, {}).get("ros__parameters", {})
        infl = plugins.get("inflation_layer")
        if isinstance(infl, dict):
            infl["inflation_radius"] = robot_overlay["local_costmap_plugins"]["inflation_layer"]["inflation_radius"]
            infl["cost_scaling_factor"] = robot_overlay["local_costmap_plugins"]["inflation_layer"]["cost_scaling_factor"]

    merged = deep_merge(merged, {
        "local_costmap": {"local_costmap": {"ros__parameters": {
            "footprint": robot_overlay["derived"]["footprint"]}}},
        "global_costmap": {"global_costmap": {"ros__parameters": {
            "footprint": robot_overlay["derived"]["footprint"]}}},
        "velocity_smoother": robot_overlay["velocity_smoother"],
    })

    # 速度上限必须落到控制器插件自己的参数段里（见 inject_controller_speeds 注释）
    with open(os.path.join(robot_share, "config", "robot_params.yaml"), "r") as f:
        _robots = yaml.safe_load(f).get("robots", {})
    merged = inject_controller_speeds(merged, args.local_planner, _robots[args.robot])

    # ---- 4. 里程计来源 / 定位模式 ----
    for node in ("controller_server", "planner_server", "behavior_server",
                 "bt_navigator", "velocity_smoother", "collision_monitor"):
        if node in merged:
            merged[node].setdefault("ros__parameters", {})["use_sim_time"] = True

    if args.localization == "slam":
        # SLAM 模式：由建图算法发布 map->odom，Nav2 不能自己发，否则 TF 双父节点
        merged.setdefault("amcl", {"ros__parameters": {}})["ros__parameters"]["enabled"] = False
        # ---------------------------------------------------------------
        # 关键：把 slam_toolbox 的参数【注入到这份合成文件里】。
        #
        # nav2_bringup 的 slam_launch.py 用 HasNodeParams(params_file, 'slam_toolbox')
        # 决定走哪条路：
        #   有 slam_toolbox 段 -> 把【我们的】params_file 传给 slam_toolbox
        #   没有               -> 用它自带的 nav2_params.yaml（真机 2D 雷达调参）
        #
        # 不注入的话，slam_toolbox 会读到 minimum_laser_range=0.0，
        # 与 points_to_scan 的 range_min=0.6 冲突 —— 机器人周围被刷出一圈
        # 假的自由空间，近处墙体被"擦除"，这是占据召回率低的直接来源。
        #
        # 注入之后，这条链路完全闭环：仿真 → points_to_scan → slam_toolbox
        # 的雷达几何参数一致，不再有隐藏的不匹配。
        # ---------------------------------------------------------------
        st_cfg = _load_slam_toolbox_params()
        if st_cfg:
            merged["slam_toolbox"] = st_cfg
    else:
        merged.setdefault("amcl", {"ros__parameters": {}})["ros__parameters"]["enabled"] = True

    # ---- 5. 落盘 ----
    os.makedirs(os.path.dirname(os.path.abspath(args.out)), exist_ok=True)
    with open(args.out, "w", encoding="utf-8") as f:
        yaml.safe_dump(merged, f, allow_unicode=True, sort_keys=False)

    print(f"[compose_nav2_params] 已生成：{args.out}")
    print(f"  机器人        : {args.robot}")
    print(f"  全局规划器    : {args.global_planner}")
    print(f"  局部规划器    : {args.local_planner}")
    print(f"  里程计来源    : {args.odom_source}")
    print(f"  定位模式      : {args.localization}")
    d = robot_overlay["derived"]
    print(f"  footprint     : {d['footprint']}")
    print(f"  内切半径      : {d['inscribed_radius']:.3f} m")
    print(f"  膨胀半径      : {d['inflation_radius']:.3f} m  (内切 + 0.20 余量)")
    print(f"  代价衰减      : {d['cost_scaling_factor']}")

    if args.print_derived:
        print("\n--- 派生参数详情 ---")
        print(yaml.safe_dump(robot_overlay, allow_unicode=True, sort_keys=False))


if __name__ == "__main__":
    main()
