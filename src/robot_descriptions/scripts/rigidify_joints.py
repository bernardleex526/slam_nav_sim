#!/usr/bin/env python3
"""
rigidify_joints.py
==================
把 URDF 中所有非固定关节（revolute / continuous / prismatic / floating）在
文本层面改写成 fixed 关节，输出一份"刚性化" URDF。

为什么需要这个
--------------
本项目默认用 gazebo_ros_planar_move 驱动底盘。planar_move 是**运动学**插件：
它直接写 x/y/yaw，不产生力矩，也完全不参与物理支撑。

而轮足/足式模型的腿关节是 revolute 且**没有任何力矩源**（没有控制器、没有
effort 指令）。URDF 里 effort=40 只是"允许的上限"，不是"实际施加的力矩"。
于是 Gazebo 里这些关节在重力下自由塌陷：

    轮子/足端着地 → 关节继续转动 → 车体一路下沉
    → base_link 落到接近地面甚至穿地（实测 z = -0.2275，应为 +0.435）

后果不只是"看起来倒了"：
  - 雷达高度错了，点云高度带 [−0.35, 1.2] 切出来的扫描几何是错的
  - planar_move 写的姿态与物理引擎的接触约束打架 → 底盘"推不动"
    → 探索状态机永远停在 TURN，割草机轨迹一步都走不出去

修法
----
把被动关节在 URDF 层锁成 fixed。车体成为单一刚体，planar_move 的运动学
驱动与物理引擎不再冲突，z 也不再下沉。

设计取舍（写清楚，避免以后误读）
--------------------------------
刚性化之后，我们**不再仿真腿式运动学**，测的就纯粹是"建图/导航算法在
理想平面底盘上的表现"。这与 common/gazebo_control.xacro 里声明的取舍一致。
要恢复真实腿动力学，就把 rigid_joints 关掉，并给腿挂上真正的步态控制器。

用法
----
    rigidify_joints.py <in.urdf> <out.urdf>
    rigidify_joints.py <in.urdf> <out.urdf> --keep FL_wheel_joint,FR_wheel_joint
"""

import argparse
import re
import sys
import xml.etree.ElementTree as ET

NON_FIXED = {"revolute", "continuous", "prismatic", "floating", "planar"}


def rigidify(text, keep):
    """把非固定关节改成 fixed。用 ElementTree 做结构化改写，避免正则误伤。"""
    # xacro 展开后的 URDF 里可能残留命名空间声明，ET 能处理
    try:
        root = ET.fromstring(text)
    except ET.ParseError as exc:
        print("PARSE_ERROR: %s" % exc, file=sys.stderr)
        return None, 0

    changed = []
    for joint in root.iter("joint"):
        jtype = joint.get("type", "")
        name = joint.get("name", "")
        if jtype not in NON_FIXED:
            continue
        if name in keep:
            continue
        joint.set("type", "fixed")
        # fixed 关节不允许 axis / limit / dynamics，留着会让 gz 报 schema 警告
        for tag in ("axis", "limit", "dynamics", "calibration", "safety_controller"):
            for child in list(joint):
                if child.tag == tag:
                    joint.remove(child)
        changed.append(name)

    if not changed:
        return text, 0

    out = ET.tostring(root, encoding="unicode")
    # ET 会把 <?xml ...?> 丢掉，补回来
    if not out.lstrip().startswith("<?xml"):
        out = '<?xml version="1.0"?>\n' + out
    return out, len(changed)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("src")
    ap.add_argument("dst")
    ap.add_argument("--keep", default="",
                    help="逗号分隔的关节名，保持原类型（例如轮子用来做视觉动画）")
    ap.add_argument("--quiet", action="store_true",
                    help="不输出任何信息。被 launch 的 Command 以管道方式调用时必须加："
                         "launch 把「命令有 stderr 输出」判定为失败，会直接中止整个 launch。")
    args = ap.parse_args()

    keep = {s.strip() for s in args.keep.split(",") if s.strip()}

    with open(args.src, "r") as f:
        text = f.read()

    out, n = rigidify(text, keep)
    if out is None:
        return 2

    with open(args.dst, "w") as f:
        f.write(out)

    # ⚠️ 默认什么都不打印，原因有两条，两条都很坑：
    #   1) stdout 是 URDF 数据流本身。往它写日志会被拼进 URDF 尾部，导致
    #      [spawn_entity] Invalid XML: Extra content at the end of the document
    #   2) launch 的 Command 认为「命令产生了 stderr 输出」= 命令失败，
    #      报 "executed command showed stderr output" 后中止整个 launch。
    #      也就是说往 stderr 写日志同样是致命的。
    # 手动调试需要看信息时显式加 --quiet 之外的方式：直接看目标文件。
    if not args.quiet:
        sys.stderr.write("RIGIDIFIED %d joints -> %s\n" % (n, args.dst))
    return 0


if __name__ == "__main__":
    sys.exit(main())
