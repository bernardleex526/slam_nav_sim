#!/usr/bin/env python3
"""
verify_odom_consistency.py —— 验证 /odom 与 /ground_truth/odom 是否一致

为什么需要：
  阶段A 的评测基准是 /ground_truth/odom（p3d 插件）。如果这个"真值"本身
  发的是错位姿，那么：
    · 所有 ATE / RPE 指标全部失真
    · 地图评测的"场内/场外"判定也会崩（拿错误的 GT 范围去裁剪 EST）
  实测踩过的坑：某轮探索的 /odom 轨迹完全在 20x20 场地内（机器人没出去），
  但 /ground_truth/odom 却有 x 到 30.9、y 到 37.7 的点 —— 真值自己跑出去了。
  怀疑是 Gazebo 里存在多个同名实体时 p3d 读错 model。

  本脚本把两个话题的轨迹并排记录，直接算逐点偏差，用来定性。

用法：
  python3 tools/verify_odom_consistency.py --duration 60 --out ~/.ros/odom_check
"""
import argparse
import csv
import math
import os
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from nav_msgs.msg import Odometry


class Checker(Node):
    def __init__(self, out_dir):
        super().__init__("odom_consistency_checker")
        self.out_dir = out_dir
        os.makedirs(out_dir, exist_ok=True)
        self.gt = []
        self.est = []
        qos = QoSProfile(depth=200,
                         reliability=ReliabilityPolicy.BEST_EFFORT,
                         durability=DurabilityPolicy.VOLATILE,
                         history=HistoryPolicy.KEEP_LAST)
        self.create_subscription(Odometry, "/ground_truth/odom", self._on_gt, qos)
        self.create_subscription(Odometry, "/odom", self._on_est, qos)
        self.get_logger().info("开始记录 /odom 与 /ground_truth/odom ...")

    def _on_gt(self, m):
        p = m.pose.pose.position
        self.gt.append((time.time(), p.x, p.y, p.z))

    def _on_est(self, m):
        p = m.pose.pose.position
        self.est.append((time.time(), p.x, p.y, p.z))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--duration", type=float, default=60.0)
    ap.add_argument("--out", default=os.path.expanduser("~/.ros/odom_check"))
    args = ap.parse_args()

    rclpy.init()
    node = Checker(args.out)
    t0 = time.time()
    while time.time() - t0 < args.duration:
        rclpy.spin_once(node, timeout_sec=0.1)
    node.destroy_node()
    rclpy.shutdown()

    def dump(name, rows):
        p = os.path.join(args.out, name)
        with open(p, "w", newline="") as f:
            w = csv.writer(f)
            w.writerow(["t", "x", "y", "z"])
            w.writerows(rows)
        return p

    pg = dump("gt.csv", node.gt)
    pe = dump("est.csv", node.est)

    print("=" * 74)
    print("/ground_truth/odom : %d 帧 -> %s" % (len(node.gt), pg))
    if node.gt:
        xs = [r[1] for r in node.gt]
        ys = [r[2] for r in node.gt]
        zs = [r[3] for r in node.gt]
        print("   x[%.3f, %.3f]  y[%.3f, %.3f]  z[%.3f, %.3f]"
              % (min(xs), max(xs), min(ys), max(ys), min(zs), max(zs)))

    print("/odom              : %d 帧 -> %s" % (len(node.est), pe))
    if node.est:
        xs = [r[1] for r in node.est]
        ys = [r[2] for r in node.est]
        zs = [r[3] for r in node.est]
        print("   x[%.3f, %.3f]  y[%.3f, %.3f]  z[%.3f, %.3f]"
              % (min(xs), max(xs), min(ys), max(ys), min(zs), max(zs)))

    print("-" * 74)
    if node.gt and node.est:
        print("⚠️  z 值解读：")
        print("   /odom 的 z 应恒等于 0（base_footprint 贴地，运动学模式）")
        print("   /ground_truth/odom 的 z 也应 ≈ 0")
        gz = [r[3] for r in node.gt]
        ez = [r[3] for r in node.est]
        print("   实测 z 均值: gt=%.4f  est=%.4f" % (sum(gz) / len(gz), sum(ez) / len(ez)))
        if abs(sum(gz) / len(gz)) > 1e-3:
            print("   ❌ 真值里程计 z 不为 0 ⇒ p3d 读到的不是贴地的根 link")
        if abs(sum(ez) / len(ez)) > 1e-3:
            print("   ❌ /odom 的 z 不为 0 ⇒ planar_move 或 TF 有问题")
    else:
        print("❌ 有一个话题没收到数据 —— 检查插件是否正常加载")
    print("=" * 74)


if __name__ == "__main__":
    main()
