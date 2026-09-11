#!/usr/bin/env python3
"""
诊断：为什么 DWB 报 "BaseObstacle/Trajectory Hits Obstacle"
一次性读取 /scan、/odom、/local_costmap/costmap，判断代价地图在机器人
自身位置上到底是什么值，以及 /scan 里有没有近场回波（自身车体）。
"""
import math
import sys
import time

import numpy as np
import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import OccupancyGrid, Odometry
from rclpy.node import Node
from rclpy.qos import QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy
from sensor_msgs.msg import LaserScan
from tf2_ros import Buffer, TransformListener


class Diag(Node):
    def __init__(self):
        super().__init__("nav_diag")
        qos = QoSProfile(depth=1, history=QoSHistoryPolicy.KEEP_LAST,
                         reliability=QoSReliabilityPolicy.RELIABLE)
        self.scan = None
        self.odom = None
        self.cost = None
        self.create_subscription(LaserScan, "/scan", self._s, qos)
        self.create_subscription(Odometry, "/odom", self._o, qos)
        self.create_subscription(OccupancyGrid, "/local_costmap/costmap", self._c, qos)
        self.cmd_pub = self.create_publisher(Twist, "/cmd_vel", 10)
        self.buffer = Buffer()
        self.listener = TransformListener(self.buffer, self)

    def _s(self, m): self.scan = m
    def _o(self, m): self.odom = m
    def _c(self, m): self.cost = m


def main():
    rclpy.init()
    n = Diag()
    t0 = time.time()
    while time.time() - t0 < 25 and not (n.scan and n.odom and n.cost):
        rclpy.spin_once(n, timeout_sec=0.5)

    print("=" * 64)
    print("【1】/scan 内容")
    if n.scan:
        s = n.scan
        r = np.asarray(s.ranges, dtype=np.float64)
        finite = r[np.isfinite(r)]
        print(f"  frame_id     : {s.header.frame_id}")
        print(f"  angle_min/max: {s.angle_min:.3f} / {s.angle_max:.3f}")
        print(f"  range_min/max: {s.range_min:.3f} / {s.range_max:.3f}")
        print(f"  有效点数     : {finite.size} / {r.size}")
        if finite.size:
            print(f"  最小距离     : {finite.min():.3f} m   <-- 若在 0.3~0.6 说明很可能打到自身车体")
            for th in (0.3, 0.4, 0.5, 0.6, 0.8, 1.0):
                print(f"    < {th:.1f}m 的点数: {int((finite < th).sum())}")
    else:
        print("  !! 未收到 /scan")

    print()
    print("【2】/local_costmap/costmap 在机器人自身位置的值")
    if n.cost and n.odom:
        c = n.cost
        res = c.info.resolution
        ox, oy = c.info.origin.position.x, c.info.origin.position.y
        px = n.odom.pose.pose.position.x
        py = n.odom.pose.pose.position.y
        ix = int((px - ox) / res)
        iy = int((py - oy) / res)
        arr = np.asarray(c.data, dtype=np.int16).reshape(c.info.height, c.info.width)
        print(f"  代价地图: {c.info.width}x{c.info.height} @ {res} m, origin=({ox:.2f},{oy:.2f})")
        print(f"  机器人位姿(/odom): ({px:.2f}, {py:.2f})  -> 栅格 ({ix},{iy})")
        if 0 <= ix < c.info.width and 0 <= iy < c.info.height:
            v = int(arr[iy, ix])
            print(f"  >>> 机器人所在格代价: {v}   "
                  f"({'致命(>=253): 起点即碰撞' if v >= 253 else '致命(>=90)' if v >= 90 else '正常'})")
            # 前向 0.5m 处
            fx = int((px + 0.5 - ox) / res)
            if 0 <= fx < c.info.width:
                print(f"  正前方 0.5m 处代价: {int(arr[iy, fx])}")
        else:
            print("  !! 机器人不在代价地图范围内（rolling window 未覆盖）")
        lethal = arr >= 253
        print(f"  致命格总数: {int(lethal.sum())} / {arr.size}")
    else:
        print("  !! 未收到 costmap 或 odom")

    print()
    print("【3】计程：给 0.2 m/s 前进指令 3 秒，看是否真的动了")
    if n.odom:
        p0 = (n.odom.pose.pose.position.x, n.odom.pose.pose.position.y)
        tw = Twist()
        tw.linear.x = 0.2
        t0 = time.time()
        while time.time() - t0 < 3.0:
            n.cmd_pub.publish(tw)
            rclpy.spin_once(n, timeout_sec=0.1)
        for _ in range(10):
            n.cmd_pub.publish(Twist())
            rclpy.spin_once(n, timeout_sec=0.05)
        if n.odom:
            p1 = (n.odom.pose.pose.position.x, n.odom.pose.pose.position.y)
            d = math.hypot(p1[0] - p0[0], p1[1] - p0[1])
            print(f"  位移: {d:.3f} m  ({'动了' if d > 0.1 else '没动 或 被墙挡住'})")

    print()
    print("【4】TF: map -> odom 是否存在（SLAM 是否在发）")
    for a, b in (("map", "odom"), ("odom", "base_footprint"), ("map", "base_footprint")):
        try:
            tf = n.buffer.lookup_transform(a, b, rclpy.time.Time())
            t = tf.transform.translation
            print(f"  {a} -> {b}: OK  ({t.x:.3f}, {t.y:.3f}, {t.z:.3f})")
        except Exception as e:
            print(f"  {a} -> {b}: 缺失 ({type(e).__name__})")

    n.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
