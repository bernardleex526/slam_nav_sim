#!/usr/bin/env python3
"""订阅 /map（TRANSIENT_LOCAL），统计占据/空闲/未知占比。

为什么不用 `ros2 topic echo`：
  /map 是 402x402 = 16 万格，YAML 逐行输出有近百 MB，
  `timeout` 一定在消息收完前把它掐掉，解析出来的数字毫无意义
  （实测只拿到 128 个值，却算出"unknown 100%"，属于典型的假数据陷阱）。

用法：python3 map_stats.py [--topic /map] [--timeout 15]
"""
import argparse
import sys

import rclpy
from nav_msgs.msg import OccupancyGrid
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy


class MapStats(Node):
    def __init__(self, topic):
        super().__init__("map_stats_one_shot")
        qos = QoSProfile(
            depth=1,
            history=QoSHistoryPolicy.KEEP_LAST,
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.got = None
        self.create_subscription(OccupancyGrid, topic, self._cb, qos)

    def _cb(self, msg):
        self.got = msg


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--topic", default="/map")
    ap.add_argument("--timeout", type=float, default=15.0)
    args = ap.parse_args()

    rclpy.init()
    node = MapStats(args.topic)

    waited = 0.0
    while rclpy.ok() and node.got is None and waited < args.timeout:
        rclpy.spin_once(node, timeout_sec=0.2)
        waited += 0.2

    msg = node.got
    if msg is None:
        print("TIMEOUT: %ss 内没收到 %s（地图是否已发布？）" % (args.timeout, args.topic))
        node.destroy_node()
        rclpy.shutdown()
        return 1

    data = list(msg.data)
    tot = len(data)
    occ = sum(1 for v in data if v >= 100)
    free = sum(1 for v in data if 0 <= v < 100)
    unk = sum(1 for v in data if v < 0)

    print("size: %dx%d = %d" % (msg.info.width, msg.info.height, tot))
    print("res: %.3f m/cell   origin: (%.2f, %.2f)" % (
        msg.info.resolution, msg.info.origin.position.x, msg.info.origin.position.y))
    print("occupancy: %.3f%%   free: %.3f%%   unknown: %.3f%%" % (
        100 * occ / tot, 100 * free / tot, 100 * unk / tot))
    print("cells: occupied=%d  free=%d  unknown=%d" % (occ, free, unk))

    node.destroy_node()
    rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
