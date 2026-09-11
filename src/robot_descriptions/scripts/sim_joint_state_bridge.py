#!/usr/bin/env python3
"""
sim_joint_state_bridge
======================
为仿真中的腿式/轮足平台发布零位关节状态。

为什么需要这个节点：
  robot_state_publisher 必须有 /joint_states 才能算出传感器 frame 的 TF。
  本环境默认用 gazebo_ros_planar_move 驱动底盘，不跑步态控制器，
  所以没有任何东西发布关节状态 —— 腿式机器人的传感器 frame 就会永久缺失，
  表现为 "TF 树里找不到 lidar_link"，rviz2 什么都看不见。

为什么不用 gazebo_ros_joint_state_publisher 插件：
  那个插件要求显式列出每个关节名，换机器人就得改 xacro，违背"换机器人不改代码"。
  这里改为自动从 /robot_description 解析，零配置。

零位角 = URDF 里的完全伸展站姿，与 stand_height 的几何推导一致，
所以不会出现脚穿地。
"""

import re

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy
from sensor_msgs.msg import JointState
from std_msgs.msg import String

_JOINT_RE = re.compile(r"<joint\b([^>]*)>(.*?)</joint>", re.DOTALL)
_ATTR_RE = re.compile(r'(\w+)\s*=\s*"([^"]*)"')

# 这些关节类型在 /joint_states 中需要位置反馈；fixed 不需要
_MOVABLE_TYPES = {"revolute", "continuous", "prismatic", "planar", "floating"}


def parse_movable_joints(urdf_xml: str):
    """从 URDF 文本里抽出所有非 fixed 关节名（保持出现顺序）。"""
    names = []
    for m in _JOINT_RE.finditer(urdf_xml):
        attrs = dict(_ATTR_RE.findall(m.group(1)))
        if attrs.get("type", "fixed") in _MOVABLE_TYPES and "name" in attrs:
            names.append(attrs["name"])
    return names


class SimJointStateBridge(Node):
    def __init__(self):
        super().__init__("sim_joint_state_bridge")

        self.declare_parameter("publish_rate", 50.0)
        self.declare_parameter("robot_description_topic", "/robot_description")
        self.declare_parameter("joint_states_topic", "/joint_states")

        self._rate = float(self.get_parameter("publish_rate").value)
        self._joint_names = []
        self._warned = False
        # 关节被全部刚性化时为 True —— 此时本节点主动静默（见 _on_description）
        self._rigid = False

        qos = QoSProfile(
            depth=1,
            history=QoSHistoryPolicy.KEEP_LAST,
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
        )

        self._pub = self.create_publisher(
            JointState, self.get_parameter("joint_states_topic").value, 10
        )
        self._sub = self.create_subscription(
            String,
            self.get_parameter("robot_description_topic").value,
            self._on_description,
            qos,
        )
        self._timer = self.create_timer(1.0 / max(self._rate, 1.0), self._tick)

        self.get_logger().info("sim_joint_state_bridge 已启动，等待 /robot_description …")

    def _on_description(self, msg: String):
        names = parse_movable_joints(msg.data)
        if not names:
            # 不是错误，是刚性化模式下的正常情况。
            #
            # 管线里 rigidify_joints.py 会把腿/轮的被动关节全锁成 fixed
            # （planar_move 无力矩源，不锁会塌陷下沉，见该脚本注释）。
            # 全 fixed 时 robot_state_publisher 直接静态发布整棵 TF，
            # 不需要 /joint_states，本节点自然无事可做。
            self._rigid = True
            self.get_logger().info(
                "robot_description 中无 revolute/continuous 关节 —— 关节已全部刚性化"
                "（fixed）。TF 由 robot_state_publisher 静态发布，本节点不发布 /joint_states。"
            )
            return
        if names != self._joint_names:
            self._joint_names = names
            self.get_logger().info(
                f"已解析到 {len(names)} 个可动关节：{', '.join(names[:6])}"
                + (" …" if len(names) > 6 else "")
            )

    def _tick(self):
        if self._rigid:
            return
        if not self._joint_names:
            if not self._warned:
                self.get_logger().warn(
                    "尚未拿到 robot_description，暂不发布 /joint_states。"
                    "若长时间无输出，检查 robot_state_publisher 是否已启动。"
                )
                self._warned = True
            return

        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = list(self._joint_names)
        msg.position = [0.0] * len(self._joint_names)
        msg.velocity = [0.0] * len(self._joint_names)
        msg.effort = [0.0] * len(self._joint_names)
        self._pub.publish(msg)


def main():
    rclpy.init()
    node = SimJointStateBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
