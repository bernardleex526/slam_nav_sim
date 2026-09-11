#!/usr/bin/env python3
"""
gen_rviz_configs
================
生成三套 rviz2 配置：mapping / navigation / evaluation。

为什么用生成器而不是手写三份 .rviz：
  一份 rviz 配置 300+ 行，三份就是 1000 行，而且 80% 是重复的。手写的话，
  以后想统一改个点云大小、加一个显示项，得改三遍，必然改漏。
  这里用"共享基础显示 + 各配置的增量"来组织，改一处三套一起变。

三套配置的分工：
  mapping    —— 建图阶段：重点看地图质量、点云稀疏度分层、轨迹漂移
  navigation —— 导航阶段：重点看 costmap、全局/局部路径、footprint
  evaluation —— 评测阶段：上面两者全开 + 轨迹对比强调，用于出截图/录屏

用法：
  python3 gen_rviz_configs.py            # 生成到 ../rviz/
  python3 gen_rviz_configs.py --out DIR
"""

import argparse
import os

# ---------------------------------------------------------------------------
# 显示项构造工具
# ---------------------------------------------------------------------------

# 配色：深色底 + 高对比。轨迹用"红=真值 / 青=估计"，
# 这样左边是红的、右边是青的，一眼能看出漂移方向。
C_GT = "255; 60; 60"
C_EST = "0; 200; 220"
C_MAP = "200; 200; 200"
C_COSTMAP = "90; 120; 200"
C_INFLATED = "120; 90; 200"
C_PATH_GLOBAL = "60; 220; 120"
C_PATH_LOCAL = "250; 200; 60"


def grid(cell=1.0, color="120; 120; 120", alpha=0.3):
    return {
        "Class": "rviz_default_plugins/Grid",
        "Name": "Grid",
        "Enabled": True,
        "Alpha": alpha,
        "Cell Size": cell,
        "Color": color,
        "Line Style": {"Line Width": 0.03, "Value": "Lines"},
        "Plane": "XY",
        "Plane Cell Count": 40,
        "Reference Frame": "<Fixed Frame>",
    }


def tf(show_names=False):
    return {
        "Class": "rviz_default_plugins/TF",
        "Name": "TF",
        "Enabled": False,
        "Show Arrows": True,
        "Show Axes": True,
        "Show Names": show_names,
        "Marker Scale": 0.4,
        "Frame Timeout": 15,
    }


def robot_model():
    return {
        "Class": "rviz_default_plugins/RobotModel",
        "Name": "RobotModel",
        "Enabled": True,
        "Description Topic": {"Value": "/robot_description"},
        "Visual Enabled": True,
        "Collision Enabled": False,
        "Alpha": 1.0,
    }


def occupancy_map(name, topic, color_scheme="map", alpha=0.7, enabled=True):
    return {
        "Class": "rviz_default_plugins/Map",
        "Name": name,
        "Enabled": enabled,
        "Topic": {"Value": topic, "Depth": 5, "Durability Policy": "Transient Local"},
        "Alpha": alpha,
        "Color Scheme": color_scheme,
        "Draw Behind": False,
        "Use Timestamp": False,
    }


def pointcloud(name, topic, size=0.03, color_mode="AxisColor", alpha=1.0,
               enabled=True, decay=0, style="Flat Squares", color=""):
    d = {
        "Class": "rviz_default_plugins/PointCloud2",
        "Name": name,
        "Enabled": enabled,
        "Topic": {"Value": topic, "Depth": 5},
        "Alpha": alpha,
        "Size (m)": size,
        "Size (Pixels)": 3,
        "Style": style,
        "Position Transformer": "XYZ",
        "Decay Time": decay,
        "Color Transformer": color_mode,
        "Use Fixed Frame": False,
        "Selectable": False,
    }
    if color:
        d["AxisColor"] = "Z"
        d["Color"] = color
    return d


def path(name, topic, color, alpha=1.0, width=0.04, enabled=True):
    return {
        "Class": "rviz_default_plugins/Path",
        "Name": name,
        "Enabled": enabled,
        "Topic": {"Value": topic, "Depth": 5},
        "Alpha": alpha,
        "Color": color,
        "Head Diameter": 0.2,
        "Head Length": 0.2,
        "Line Style": "Lines",
        "Line Width": width,
        "Pose Style": "None",
        "Radius": 0.03,
        "Shaft Diameter": 0.1,
        "Shaft Length": 0.1,
    }


def scan(name, topic, color, size=0.06, enabled=True):
    return {
        "Class": "rviz_default_plugins/LaserScan",
        "Name": name,
        "Enabled": enabled,
        "Topic": {"Value": topic, "Depth": 5},
        "Color": color,
        "Size (m)": size,
        "Style": "Points",
        "Alpha": 0.8,
        "Position Transformer": "XYZ",
    }


def polygon(name, topic, color, alpha=0.4, enabled=True):
    return {
        "Class": "rviz_default_plugins/Polygon",
        "Name": name,
        "Enabled": enabled,
        "Topic": {"Value": topic, "Depth": 5},
        "Alpha": alpha,
        "Color": color,
    }


def marker_array(name, topic, enabled=True):
    return {
        "Class": "rviz_default_plugins/MarkerArray",
        "Name": name,
        "Enabled": enabled,
        "Topic": {"Value": topic, "Depth": 5},
    }


# ---------------------------------------------------------------------------
# 共享基础显示块
# ---------------------------------------------------------------------------

def base_world():
    return [
        grid(),
        tf(),
        robot_model(),
        scan("Scan (虚拟 2D)", "/scan", "255; 255; 120"),
    ]


def sparsity_block():
    """需求 #4 的核心：不同稀疏程度的点云同屏对比。

    四层从原始到最稀疏，颜色和点大小递减 —— 这样在同一视角下能直接看出
    "砍到哪一档还保留着关键结构"。
    """
    return [
        pointcloud("原始点云 /points", "/points", size=0.015,
                   color_mode="AxisColor", alpha=0.5, enabled=False),
        pointcloud("滤波后 /points_filtered", "/points_filtered", size=0.02,
                   color_mode="AxisColor", alpha=0.7, enabled=True),
        pointcloud("稀疏 0.10m /points_light", "/points_light", size=0.03,
                   color_mode="Intensity", alpha=0.9, enabled=False),
        pointcloud("稀疏 0.25m /points_medium", "/points_medium", size=0.045,
                   color_mode="FlatColor", color=C_EST, alpha=1.0, enabled=True),
        pointcloud("稀疏 0.50m /points_heavy", "/points_heavy", size=0.07,
                   color_mode="FlatColor", color="255; 180; 60", alpha=1.0, enabled=False),
    ]


def map_block():
    return [
        occupancy_map("2D 地图 /map", "/map", "map", alpha=0.7),
        pointcloud("3D 地图点云 /map_cloud", "/map_cloud", size=0.03,
                   color_mode="AxisColor", alpha=0.9),
        pointcloud("3D 地图降采样 /map_cloud_downsampled", "/map_cloud_downsampled",
                   size=0.06, color_mode="FlatColor", color=C_MAP, alpha=1.0),
    ]


def trajectory_block():
    return [
        path("真值轨迹 /eval/gt_path", "/eval/gt_path", C_GT, width=0.05),
        path("估计轨迹 /eval/est_path", "/eval/est_path", C_EST, width=0.05),
    ]


def nav_block():
    return [
        occupancy_map("全局代价地图", "/global_costmap/costmap", "costmap", alpha=0.5, enabled=False),
        occupancy_map("局部代价地图", "/local_costmap/costmap", "costmap", alpha=0.6, enabled=True),
        occupancy_map("全局代价地图(膨胀)", "/global_costmap/costmap_raw", "costmap", alpha=0.35, enabled=False),
        occupancy_map("局部代价地图(膨胀)", "/local_costmap/costmap_raw", "costmap", alpha=0.45, enabled=False),
        path("全局路径 /plan", "/plan", C_PATH_GLOBAL, width=0.06),
        path("局部路径 /local_plan", "/local_plan", C_PATH_LOCAL, width=0.05),
        polygon("Footprint", "/local_costmap/published_footprint", "255; 80; 255", alpha=0.5),
        marker_array("导航目标 /waypoint_markers", "/waypoint_markers"),
    ]


# ---------------------------------------------------------------------------
# 生成
# ---------------------------------------------------------------------------

CONFIGS = {
    # 建图阶段：地图质量 + 稀疏度分层 + 轨迹漂移
    "mapping": {
        "fixed_frame": "map",
        "displays": base_world() + map_block() + sparsity_block() + trajectory_block(),
    },
    # 导航阶段：costmap + 路径 + footprint
    "navigation": {
        "fixed_frame": "map",
        "displays": base_world() + map_block() + nav_block(),
    },
    # 评测阶段：全开，轨迹对比强调，用于出图
    "evaluation": {
        "fixed_frame": "map",
        "displays": (
            base_world()
            + map_block()      # /map 默认开
            + sparsity_block() # 稀疏度分层
            + trajectory_block()
            + nav_block()
        ),
    },
}


def build_config(spec):
    return {
        "Panels": [
            {"Class": "rviz_common/Displays", "Name": "Displays"},
            {"Class": "rviz_common/Selection", "Name": "Selection"},
            {"Class": "rviz_common/Tool Properties", "Name": "Tool Properties"},
            {"Class": "rviz_common/Views", "Name": "Views"},
        ],
        "Visualization Manager": {
            "Class": "",
            "Displays": spec["displays"],
            "Global Options": {
                # 深色底 + 浅色文字，与 IDE 主题一致，长时间盯屏幕不刺眼
                "Background Color": "38; 40; 44",
                "Fixed Frame": spec["fixed_frame"],
                "Frame Rate": 30,
            },
            "Tools": [
                {"Class": "rviz_default_plugins/Interact"},
                {"Class": "rviz_default_plugins/MoveCamera"},
                {"Class": "rviz_default_plugins/Select"},
                {
                    "Class": "rviz_default_plugins/SetInitialPose",
                    "Topic": {"Value": "/initialpose"},
                },
                {
                    "Class": "rviz_default_plugins/SetGoal",
                    "Topic": {"Value": "/goal_pose"},
                },
                {"Class": "rviz_default_plugins/PublishPoint"},
            ],
            "Views": {
                "Current": {
                    "Class": "rviz_default_plugins/Orbit",
                    "Distance": 22.0,
                    "Focal Point": {"X": 0.0, "Y": 0.0, "Z": 0.0},
                    "Name": "Current View",
                    "Pitch": 1.2,
                    "Yaw": 0.0,
                    "Target Frame": "<Fixed Frame>",
                },
                "Saved": None,
            },
        },
        "Window Geometry": {
            "Height": 900,
            "Width": 1500,
            "X": 60,
            "Y": 60,
        },
    }


def dump_yaml(obj, indent=0):
    """极简 YAML 输出（不依赖 pyyaml，方便在任意环境重跑）。

    rviz 的配置就是普通 YAML，但字段名里大量出现 'Size (m)'、'::'、';' 这类
    字符。手写容易踩引号坑，这里统一对含特殊字符的 key/value 加引号。
    """
    SP = "  "
    lines = []
    pad = SP * indent

    if isinstance(obj, dict):
        if not obj:
            return pad + "{}\n"
        for k, v in obj.items():
            key = str(k)
            if any(c in key for c in "():;,[]#&*!|>'\"%@` ") or key[0].isdigit():
                key = f"'{key}'"
            if isinstance(v, (dict, list)) and v:
                lines.append(f"{pad}{key}:\n")
                lines.append(dump_yaml(v, indent + 1))
            elif isinstance(v, (dict, list)):
                lines.append(f"{pad}{key}: {'{}' if isinstance(v, dict) else '[]'}\n")
            else:
                lines.append(f"{pad}{key}: {fmt_scalar(v)}\n")
    elif isinstance(obj, list):
        for item in obj:
            if isinstance(item, dict):
                body = dump_yaml(item, indent + 1)
                first, rest = body.split("\n", 1)
                lines.append(f"{pad}- {first.strip()}\n")
                lines.append(rest)
            else:
                lines.append(f"{pad}- {fmt_scalar(item)}\n")
    return "".join(lines)


def fmt_scalar(v):
    if v is None:
        return "~"
    if isinstance(v, bool):
        return "true" if v else "false"
    if isinstance(v, (int, float)):
        return str(v)
    s = str(v)
    if s == "" or any(c in s for c in "():;,[]{}\"'#&*!|>%@`") or s.strip() != s:
        return f"'{s}'"
    return s


def main():
    here = os.path.dirname(os.path.abspath(__file__))
    pkg_root = os.path.dirname(here)

    ap = argparse.ArgumentParser()
    ap.add_argument("--out", default=os.path.join(pkg_root, "rviz"))
    args = ap.parse_args()
    os.makedirs(args.out, exist_ok=True)

    for name, spec in CONFIGS.items():
        cfg = build_config(spec)
        path_out = os.path.join(args.out, f"{name}.rviz")
        # newline="\n"：同上，避免 Windows 写出 CRLF 污染给 WSL 用的配置
        with open(path_out, "w", encoding="utf-8", newline="\n") as f:
            f.write("# 由 sim_viz/tools/gen_rviz_configs.py 生成，请勿手工编辑。\n")
            f.write("# 想改显示项，改生成器里的 CONFIGS，然后重跑脚本。\n")
            f.write(dump_yaml(cfg))
        n_on = sum(1 for d in spec["displays"] if d.get("Enabled"))
        print(f"[rviz] {name:11s} -> {path_out}  "
              f"({len(spec['displays'])} 个显示项，默认开 {n_on} 个)")

    print("\n提示：evaluation 这套把稀疏度分层、代价地图、双轨迹全开，")
    print("      适合出对比截图；平时调试用 mapping / navigation 更清爽。")


if __name__ == "__main__":
    main()
