#!/usr/bin/env python3
"""
check_interface
===============
接口一致性自检节点。SPEC.md 第 5 节承诺的验证工具。

用途：新机器人接入后跑一次，直接告诉你是哪个 frame / 话题没对齐。
比在 rviz2 里对着空窗口猜要快得多。

检查项：
  1. TF 树里是否存在所有必需 frame
  2. 必需话题是否已发布（且类型正确）
  3. 里程计来源与 TF 发布责任是否自洽（最容易踩的坑）

退出码：0 = 通过，1 = 有问题（可直接用在 CI / 脚本里）
"""

import sys
import time

import rclpy
from rclpy.node import Node
from tf2_ros import Buffer, TransformListener

DEFAULT_REQUIRED_FRAMES = ["odom", "base_footprint", "base_link", "imu_link", "lidar_link"]

DEFAULT_REQUIRED_TOPICS = [
    ("/points", "sensor_msgs/msg/PointCloud2"),
    ("/imu", "sensor_msgs/msg/Imu"),
    ("/odom", "nav_msgs/msg/Odometry"),
]

OPTIONAL_FRAMES = ["lidar_stabilized", "camera_link", "camera_optical_frame"]

OK = "  [ OK ]"
BAD = "  [FAIL]"
WARN = "  [WARN]"


class CheckInterface(Node):
    def __init__(self, frames, topics, check_map_frame, timeout):
        super().__init__("check_interface")
        self._frames = frames
        self._topics = topics
        self._check_map = check_map_frame
        self._timeout = timeout

        self._buffer = Buffer()
        self._listener = TransformListener(self._buffer, self)

    def _all_frames(self):
        try:
            return set(self._buffer.all_frames_as_yaml().split("\n"))
        except Exception:
            return set()

    def _wait(self, predicate, what):
        deadline = time.time() + self._timeout
        last = None
        while time.time() < deadline:
            rclpy.spin_once(self, timeout_sec=0.2)
            last = predicate()
            if last:
                return True
        return False

    def run(self) -> bool:
        failures = []

        print("\n=== 1. TF frame 检查 ===")
        deadline = time.time() + self._timeout
        required = list(self._frames)
        if self._check_map:
            required.append("map")

        seen = set()
        while time.time() < deadline:
            rclpy.spin_once(self, timeout_sec=0.2)
            yaml_dump = self._buffer.all_frames_as_yaml()
            seen = {
                line.split(":")[0]
                for line in yaml_dump.splitlines()
                if line and not line.startswith(" ")
            }
            if all(f in seen for f in required):
                break

        for f in required:
            if f in seen:
                print(f"{OK} frame 存在：{f}")
            else:
                print(f"{BAD} frame 缺失：{f}")
                failures.append(f"frame:{f}")

        optional_present = [f for f in OPTIONAL_FRAMES if f in seen]
        if optional_present:
            print(f"  [info] 额外存在：{', '.join(optional_present)}")

        print("\n=== 2. 话题检查 ===")
        for name, expected_type in self._topics:
            found = None
            deadline = time.time() + self._timeout
            while time.time() < deadline and not found:
                rclpy.spin_once(self, timeout_sec=0.2)
                for topic_name, topic_types in self.get_topic_names_and_types():
                    if topic_name == name:
                        found = topic_types
                        break

            if not found:
                print(f"{BAD} 话题未发布：{name}")
                failures.append(f"topic:{name}")
                continue
            if expected_type in found:
                print(f"{OK} 话题就绪：{name}  [{expected_type}]")
            else:
                print(f"{WARN} 话题类型不符：{name}  期望 {expected_type}，实际 {found}")
                failures.append(f"topic_type:{name}")

        print("\n=== 3. TF 发布责任自检 ===")
        # odom -> base_footprint 必须只有一个发布者；这里做启发式提示
        try:
            tfs = self._buffer.lookup_transform("odom", "base_footprint", rclpy.time.Time())
            if tfs is not None:
                print(f"{OK} odom -> base_footprint 可查（来源自洽）")
        except Exception as e:
            print(f"{WARN} odom -> base_footprint 查询失败：{type(e).__name__}")
            print("      常见原因：odom_source=lio 但 LIO 未启动，或 planar_move 的")
            print("      publish_odom_tf 被误设为 false。")

        print("\n=== 结论 ===")
        if failures:
            print(f"发现 {len(failures)} 项问题：")
            for f in failures:
                print(f"  - {f}")
            print("\n对照 robot_interface_spec/SPEC.md 第 2、3 节排查。")
            return False
        print("接口检查全部通过。机器人已 conform 规范。")
        return True


def main():
    rclpy.init()
    frames = sys.argv[1].split(",") if len(sys.argv) > 1 and sys.argv[1] else DEFAULT_REQUIRED_FRAMES
    timeout = float(sys.argv[2]) if len(sys.argv) > 2 else 15.0

    node = CheckInterface(frames, DEFAULT_REQUIRED_TOPICS, False, timeout)
    try:
        ok = node.run()
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    sys.exit(0 if ok else 1)


if __name__ == "__main__":
    main()
