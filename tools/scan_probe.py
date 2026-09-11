#!/usr/bin/env python3
"""scan_probe.py —— 直接订阅 /scan 与 /points，打印 header 与统计

为什么不用 `ros2 topic echo`：
  本机 ROS2 daemon 时常坏掉，`ros2 topic list/echo/info` 会静默返回空，
  看起来像"话题没有数据"，会严重误导排查。自己起订阅者最可靠。

用法：
    python3 tools/scan_probe.py [秒数]
"""
import math
import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy
from sensor_msgs.msg import LaserScan, PointCloud2


class Probe(Node):
    def __init__(self):
        super().__init__("scan_probe")
        self.scan = None
        self.cloud = None
        self.n_scan = 0
        self.n_cloud = 0
        q = QoSProfile(depth=5, history=QoSHistoryPolicy.KEEP_LAST,
                       reliability=QoSReliabilityPolicy.BEST_EFFORT)
        self.create_subscription(LaserScan, "/scan", self._on_scan, q)
        self.create_subscription(PointCloud2, "/points", self._on_cloud, q)

    def _on_scan(self, m):
        self.scan = m
        self.n_scan += 1

    def _on_cloud(self, m):
        self.cloud = m
        self.n_cloud += 1


def main():
    dur = float(sys.argv[1]) if len(sys.argv) > 1 else 12.0
    rclpy.init()
    n = Probe()
    t0 = time.time()
    while time.time() - t0 < dur:
        rclpy.spin_once(n, timeout_sec=0.2)

    print(f"=== /scan 收到 {n.n_scan} 帧 ===")
    if n.scan:
        s = n.scan
        finite = [r for r in s.ranges if r > 0 and math.isfinite(r)]
        print(f"  frame_id   : {s.header.frame_id}")
        print(f"  stamp      : {s.header.stamp.sec}.{s.header.stamp.nanosec:09d}")
        print(f"  ranges     : {len(s.ranges)} 点，有效 {len(finite)}")
        if finite:
            print(f"  距离范围   : [{min(finite):.2f}, {max(finite):.2f}] m")
        print(f"  angle_min/max: {s.angle_min:.3f} / {s.angle_max:.3f}")
        print(f"  range_min/max: {s.range_min:.2f} / {s.range_max:.2f}")
        print(f"  scan_time  : {s.scan_time}   time_increment: {s.time_increment}")
    else:
        print("  ❌ 没有收到任何 scan")

    print(f"\n=== /points 收到 {n.n_cloud} 帧 ===")
    if n.cloud:
        c = n.cloud
        print(f"  frame_id   : {c.header.frame_id}")
        print(f"  stamp      : {c.header.stamp.sec}.{c.header.stamp.nanosec:09d}")
        print(f"  点云尺寸   : {c.width} x {c.height} = {c.width*c.height}")
    else:
        print("  ❌ 没有收到任何点云")

    print(f"\n=== 当前时钟 ===")
    print(f"  now: {n.get_clock().now().nanoseconds/1e9:.3f}")
    if n.scan:
        print(f"  scan stamp: {n.scan.header.stamp.sec}.{n.scan.header.stamp.nanosec/1e9:.3f}")

    n.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
