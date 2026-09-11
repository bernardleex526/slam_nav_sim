#!/usr/bin/env python3
"""统计 /scan 的有效读数分布：多少在 range_min 附近、多少是 inf。

用来验证「盲区把墙擦掉」的假设：
如果大量读数落在 range_min（0.6m）附近，说明机器人经常贴墙，
那部分墙的观测会被 slam_toolbox 以 min_laser_range 丢掉。

用法：python3 scan_stats.py [--topic /scan] [--n 30]
"""
import argparse
import math
import sys

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan


class ScanStats(Node):
    def __init__(self, topic, n):
        super().__init__("scan_stats")
        self.n = n
        self.frames = []
        self.create_subscription(LaserScan, topic, self._cb, 10)

    def _cb(self, msg):
        self.frames.append(msg)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--topic", default="/scan")
    ap.add_argument("--n", type=int, default=30)
    args = ap.parse_args()

    rclpy.init()
    node = ScanStats(args.topic, args.n)

    import time
    t0 = time.time()
    while rclpy.ok() and len(node.frames) < args.n and time.time() - t0 < 30:
        rclpy.spin_once(node, timeout_sec=0.2)

    if not node.frames:
        print("没收到 %s" % args.topic)
        return 1

    m = node.frames[-1]
    print("frames collected: %d" % len(node.frames))
    print("angle: [%.1f°, %.1f°]  increment=%.4f°  n=%d" % (
        math.degrees(m.angle_min), math.degrees(m.angle_max),
        math.degrees(m.angle_increment), len(m.ranges)))
    print("range_min=%.3f  range_max=%.3f" % (m.range_min, m.range_max))

    # 累计所有帧
    near_min = 0
    near_max = 0
    valid = 0
    total = 0
    hist = [0] * 10   # 0.6~10m 分 10 桶
    for f in node.frames:
        for r in f.ranges:
            total += 1
            if not math.isfinite(r):
                near_max += 1
                continue
            if r < f.range_min or r > f.range_max:
                near_min += 1
                continue
            valid += 1
            d = r - f.range_min
            b = int(d * 10 / (f.range_max - f.range_min))
            if 0 <= b < 10:
                hist[b] += 1

    print("")
    print("总计 %d 个读数（%d 帧）" % (total, len(node.frames)))
    print("  有效            = %d (%.1f%%)" % (valid, 100.0 * valid / total))
    print("  越界(含<range_min) = %d (%.1f%%)" % (near_min, 100.0 * near_min / total))
    print("  非有限(inf/nan)  = %d (%.1f%%)" % (near_max, 100.0 * near_max / total))
    print("")
    print("有效读数距离分布（从 range_min 到 range_max 分 10 段）：")
    for i, c in enumerate(hist):
        lo = m.range_min + i * (m.range_max - m.range_min) / 10.0
        hi = m.range_min + (i + 1) * (m.range_max - m.range_min) / 10.0
        bar = "#" * int(60.0 * c / max(1, max(hist)))
        print("  %6.1f-%-6.1f m: %8d  %s" % (lo, hi, c, bar))

    node.destroy_node()
    rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
