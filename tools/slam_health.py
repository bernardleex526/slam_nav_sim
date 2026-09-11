#!/usr/bin/env python3
"""slam_health.py —— 一次性体检 slam_toolbox 是否真的在工作

回答：扫描进来了吗？地图在更新吗？map->odom 在发吗？

关键点：本节点必须 use_sim_time=True，否则它自己的时钟是墙钟，
和仿真时间（几百秒）差 1.7e9，所有基于时间的判断都会错。
上一轮排查就是被这个绊了一次（打印出 now=1.789e9 而 stamp=684，
一度误以为时戳错乱，其实是探针自己没设 use_sim_time）。

用法：
    python3 tools/slam_health.py [秒数]
"""
import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSHistoryPolicy, \
    QoSReliabilityPolicy
from nav_msgs.msg import OccupancyGrid, Odometry, Path
from geometry_msgs.msg import PoseWithCovarianceStamped
from sensor_msgs.msg import LaserScan
from tf2_msgs.msg import TFMessage


class Health(Node):
    def __init__(self):
        super().__init__("slam_health")
        self.set_parameters([rclpy.parameter.Parameter(
            "use_sim_time", rclpy.Parameter.Type.BOOL, True)])
        self.n = {"scan": 0, "map": 0, "map_tf": 0, "pose": 0}
        self.last_map = None
        self.map_shapes = []

        q = QoSProfile(depth=10, history=QoSHistoryPolicy.KEEP_LAST,
                       reliability=QoSReliabilityPolicy.RELIABLE)
        sq = QoSProfile(depth=5, history=QoSHistoryPolicy.KEEP_LAST,
                        reliability=QoSReliabilityPolicy.BEST_EFFORT)
        mq = QoSProfile(depth=1, history=QoSHistoryPolicy.KEEP_LAST,
                        reliability=QoSReliabilityPolicy.RELIABLE,
                        durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)

        self.create_subscription(LaserScan, "/scan", self._scan, sq)
        self.create_subscription(OccupancyGrid, "/map", self._map, mq)
        self.create_subscription(TFMessage, "/tf", self._tf, q)
        for t in ("/pose", "/slam_toolbox/pose"):
            try:
                self.create_subscription(PoseWithCovarianceStamped, t,
                                         self._pose, q)
            except Exception:
                pass
        self.create_subscription(Odometry, "/odom", self._odom, q)
        self.n_odom = 0

    def _scan(self, m):
        self.n["scan"] += 1

    def _map(self, m):
        self.n["map"] += 1
        self.last_map = m
        info = m.info
        self.map_shapes.append((info.width, info.height))

    def _pose(self, m):
        self.n["pose"] += 1

    def _odom(self, m):
        self.n_odom += 1

    def _tf(self, msg):
        for t in msg.transforms:
            if t.header.frame_id == "map" or t.child_frame_id == "map":
                self.n["map_tf"] += 1


def main():
    dur = float(sys.argv[1]) if len(sys.argv) > 1 else 20.0
    rclpy.init()
    h = Health()
    t0 = time.time()
    while time.time() - t0 < dur:
        rclpy.spin_once(h, timeout_sec=0.1)

    print(f"采样 {dur:.0f}s")
    sim_now = h.get_clock().now().nanoseconds / 1e9
    print(f"仿真时钟 now = {sim_now:.3f}")
    print()
    print(f"  /scan    : {h.n['scan']:5d} 帧  ({h.n['scan']/dur:.1f} Hz)")
    print(f"  /odom    : {h.n_odom:5d} 帧  ({h.n_odom/dur:.1f} Hz)")
    print(f"  /map     : {h.n['map']:5d} 帧")
    print(f"  /pose    : {h.n['pose']:5d} 帧")
    print(f"  map 相关 TF : {h.n['map_tf']:5d} 条")
    print()
    if h.n["map"] == 0:
        print("  ❌ /map 完全没有发布 —— slam_toolbox 没有在建图")
    else:
        w, hgt = h.last_map.info.width, h.last_map.info.height
        r = h.last_map.info.resolution
        ox, oy = h.last_map.info.origin.position.x, \
            h.last_map.info.origin.position.y
        print(f"  ✅ /map 尺寸 {w}x{hgt} @ {r}m  "
              f"origin ({ox:.2f},{oy:.2f})  "
              f"世界 x[{ox:.1f},{ox+w*r:.1f}] y[{oy:.1f},{oy+h*r:.1f}]")
    if h.n["map_tf"] == 0:
        print("  ❌ map->odom TF 从未发布 —— 这是幽灵墙的直接原因")
    else:
        print(f"  ✅ map 相关 TF {h.n['map_tf']} 条")

    h.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
