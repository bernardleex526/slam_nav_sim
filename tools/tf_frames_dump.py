#!/usr/bin/env python3
"""tf_frames_dump.py —— 抓取 /tf 与 /tf_static，列出所有出现过的 frame 对

用途（定位 map->odom 缺失）：
  ros2 run tf2_ros tf2_echo map odom 报 "frame does not exist"，
  但地图确实被建出来了。必须直接看 /tf 里到底有哪些 parent->child，
  才能判断是"根本没发"还是"frame 名不匹配"（例如发了 map->odom 但
  child 实际是 odom 的某个子 frame，或 parent 用了命名空间前缀）。
"""
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSHistoryPolicy, \
    QoSReliabilityPolicy
from tf2_msgs.msg import TFMessage
from collections import Counter


class Dump(Node):
    def __init__(self):
        super().__init__("tf_frames_dump")
        self.pairs = Counter()
        self.n = 0
        q = QoSProfile(depth=50, history=QoSHistoryPolicy.KEEP_LAST,
                       reliability=QoSReliabilityPolicy.RELIABLE)
        self.create_subscription(TFMessage, "/tf", self._cb, q)
        sq = QoSProfile(depth=100, history=QoSHistoryPolicy.KEEP_LAST,
                        reliability=QoSReliabilityPolicy.RELIABLE,
                        durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(TFMessage, "/tf_static", self._cb_static, sq)

    def _cb(self, msg):
        self.n += 1
        for t in msg.transforms:
            self.pairs[(t.header.frame_id, t.child_frame_id)] += 1

    def _cb_static(self, msg):
        for t in msg.transforms:
            self.pairs[("STATIC:" + t.header.frame_id, t.child_frame_id)] += 1


def main():
    rclpy.init()
    n = Dump()
    import time
    t0 = time.time()
    while time.time() - t0 < 20.0:
        rclpy.spin_once(n, timeout_sec=0.2)
    print(f"收到 /tf 消息 {n.n} 条")
    print("\nparent -> child  （出现次数）")
    for (p, c), k in sorted(n.pairs.items()):
        print(f"  {p:28s} -> {c:28s}  {k}")
    print("\n是否出现 map 相关 frame：")
    has = [k for k in n.pairs if "map" in k[0] or "map" in k[1]]
    if has:
        for k in has:
            print("  ", k)
    else:
        print("   ❌ 完全没有 map frame！")
    n.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
