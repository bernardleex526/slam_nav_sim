"""Sensor adapter and explicit fault injection; never subscribes to simulator truth."""

import argparse
from copy import deepcopy
import math
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TwistStamped, TransformStamped
from sensor_msgs.msg import LaserScan
from tf2_ros import TransformBroadcaster


class SensorAdapter(Node):
    def __init__(self, mode, fault):
        super().__init__(
            "validation_sensor_adapter",
            parameter_overrides=[rclpy.parameter.Parameter("use_sim_time", value=True)],
        )
        self.mode, self.fault = mode, fault
        self.odom_out = self.create_publisher(Odometry, "/odom", 10)
        self.tf = TransformBroadcaster(self)
        self.previous = None
        self.x = self.y = self.angle = 0.0
        self.subs = []
        if mode == "bag":
            self.subs.append(
                self.create_subscription(TwistStamped, "/wheel_twist", self.wheel, 100)
            )
        else:
            self.scan_out = self.create_publisher(LaserScan, "/scan", qos_profile_sensor_data)
            self.subs.append(
                self.create_subscription(LaserScan, "/scan_raw", self.scan, qos_profile_sensor_data)
            )
            self.subs.append(
                self.create_subscription(
                    Odometry, "/odom_raw", self.odometry, qos_profile_sensor_data
                )
            )

    def scan(self, msg):
        ts = msg.header.stamp.sec + msg.header.stamp.nanosec / 1e9
        if self.fault == "scan-gap" and 15 <= ts <= 23:
            return
        msg.header.frame_id = "laser"
        self.scan_out.publish(msg)

    def publish(self, msg):
        self.odom_out.publish(msg)
        tf = TransformStamped()
        tf.header = msg.header
        tf.child_frame_id = "base_link"
        tf.transform.translation.x = msg.pose.pose.position.x
        tf.transform.translation.y = msg.pose.pose.position.y
        tf.transform.rotation = msg.pose.pose.orientation
        self.tf.sendTransform(tf)

    def odometry(self, msg):
        out = deepcopy(msg)
        if self.fault == "odometry-bias":
            out.pose.pose.position.x *= 3
            out.pose.pose.position.y *= 3
            out.twist.twist.linear.x *= 3
        out.header.frame_id, out.child_frame_id = "odom", "base_link"
        self.publish(out)

    def wheel(self, msg):
        ts = msg.header.stamp.sec + msg.header.stamp.nanosec / 1e9
        dt = 0 if self.previous is None else ts - self.previous
        if dt < 0:
            raise ValueError("Wheel timestamp went backward")
        self.previous = ts
        distance = msg.twist.linear.x * dt * (3 if self.fault == "odometry-bias" else 1)
        self.x += distance * math.cos(self.angle)
        self.y += distance * math.sin(self.angle)
        self.angle += msg.twist.angular.z * dt
        out = Odometry()
        out.header.stamp, out.header.frame_id = msg.header.stamp, "odom"
        out.child_frame_id = "base_link"
        out.pose.pose.position.x, out.pose.pose.position.y = self.x, self.y
        out.pose.pose.orientation.z, out.pose.pose.orientation.w = (
            math.sin(self.angle / 2),
            math.cos(self.angle / 2),
        )
        out.twist.twist = msg.twist
        self.publish(out)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("mode", choices=("bag", "sim"))
    parser.add_argument("--fault", default="none")
    args = parser.parse_args()
    rclpy.init()
    node = SensorAdapter(args.mode, args.fault)
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
