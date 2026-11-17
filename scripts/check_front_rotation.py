#!/usr/bin/env python3
"""Run the installed Nav2 collision monitor at the rotation-blocking boundary."""

import argparse
import math
from pathlib import Path
import time

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, TransformStamped, PolygonStamped, Point32
from sensor_msgs.msg import LaserScan
from lifecycle_msgs.srv import ChangeState
from lifecycle_msgs.msg import Transition
from tf2_ros import StaticTransformBroadcaster
from ament_index_python.packages import get_package_share_directory

from robot_validate.model import dump_json
from robot_validate.process import ProcessScope
from robot_validate.ros_assets import nav_config
from robot_validate.ros_backend import isolated_domain


class Harness(Node):
    def __init__(self):
        super().__init__("front_rotation_contract")
        self.command = (0.0, 0.0)
        self.wall_present = False
        self.samples = []
        self.scan_pub = self.create_publisher(LaserScan, "/scan", 10)
        self.cmd_pub = self.create_publisher(Twist, "/cmd_vel_smoothed", 10)
        self.foot_pub = self.create_publisher(
            PolygonStamped, "/local_costmap/published_footprint", 10
        )
        self.subscription = self.create_subscription(Twist, "/cmd_vel", self.collect, 10)
        self.tf = StaticTransformBroadcaster(self)
        transforms = []
        for parent, child in [("odom", "base_link"), ("base_link", "laser")]:
            msg = TransformStamped()
            msg.header.frame_id, msg.child_frame_id = parent, child
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.transform.rotation.w = 1.0
            transforms.append(msg)
        self.tf.sendTransform(transforms)
        self.timer = self.create_timer(0.02, self.publish)

    def collect(self, msg):
        self.samples.append([time.monotonic(), msg.linear.x, msg.angular.z])

    def publish(self):
        scan = LaserScan()
        scan.header.frame_id = "laser"
        scan.header.stamp = self.get_clock().now().to_msg()
        scan.angle_min, scan.angle_max = -2.094, 2.094
        scan.angle_increment = (scan.angle_max - scan.angle_min) / 360
        scan.range_min, scan.range_max = 0.05, 12.0
        scan.ranges = []
        for i in range(361):
            cosine = math.cos(scan.angle_min + i * scan.angle_increment)
            distance = 0.45 / cosine if cosine > 0 else float("inf")
            scan.ranges.append(distance if self.wall_present and distance < 12 else float("inf"))
        self.scan_pub.publish(scan)
        footprint = PolygonStamped()
        footprint.header = scan.header
        footprint.header.frame_id = "base_link"
        for i in range(24):
            footprint.polygon.points.append(
                Point32(x=0.26 * math.cos(i * math.pi / 12), y=0.26 * math.sin(i * math.pi / 12))
            )
        self.foot_pub.publish(footprint)
        msg = Twist()
        msg.linear.x, msg.angular.z = self.command
        self.cmd_pub.publish(msg)

    def spin_for(self, seconds):
        until = time.monotonic() + seconds
        while time.monotonic() < until:
            rclpy.spin_once(self, timeout_sec=0.02)

    def transition(self, transition):
        client = self.create_client(ChangeState, "/collision_monitor/change_state")
        try:
            deadline = time.monotonic() + 15
            while not client.service_is_ready() and time.monotonic() < deadline:
                self.spin_for(0.05)
            if not client.service_is_ready():
                raise TimeoutError("collision monitor lifecycle service")
            request = ChangeState.Request()
            request.transition.id = transition
            future = client.call_async(request)
            while not future.done() and time.monotonic() < deadline:
                self.spin_for(0.05)
            assert future.done() and future.result().success
        finally:
            self.destroy_client(client)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--params", type=Path)
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=False)
    params = args.params
    if params is None:
        params = args.output / "nav.yaml"
        nav_config(
            Path(get_package_share_directory("nav2_bringup")) / "params/nav2_params.yaml", params
        )
    record = dict(status="fail", stages=[])
    with isolated_domain():
        rclpy.init()
        try:
            with ProcessScope(args.output / "logs") as scope:
                scope.start(
                    "monitor",
                    [
                        "ros2",
                        "run",
                        "nav2_collision_monitor",
                        "collision_monitor",
                        "--ros-args",
                        "--params-file",
                        str(params.resolve()),
                        "-p",
                        "use_sim_time:=false",
                    ],
                )
                harness = Harness()
                try:
                    harness.transition(Transition.TRANSITION_CONFIGURE)
                    harness.transition(Transition.TRANSITION_ACTIVATE)
                    harness.spin_for(1)
                    for name, wall, command in [
                        ("blocked_translation", True, (0.35, 0.6)),
                        ("safe_rotation", True, (0.0, 0.6)),
                        ("cleared_translation", False, (0.35, 0.0)),
                    ]:
                        harness.wall_present, harness.command = wall, command
                        start = time.monotonic()
                        harness.spin_for(1.5)
                        samples = [row[1:] for row in harness.samples if row[0] > start + 0.5]
                        if len(samples) < 10:
                            passed = False
                        elif name == "blocked_translation":
                            passed = all(abs(v) < 1e-6 for v, w in samples)
                        elif name == "safe_rotation":
                            passed = all(abs(v) < 1e-6 and 0.05 < w <= 0.201 for v, w in samples)
                        else:
                            passed = all(v > 0.34 for v, w in samples)
                        record["stages"].append(dict(name=name, passed=passed, samples=samples))
                    record["status"] = (
                        "pass" if all(s["passed"] for s in record["stages"]) else "fail"
                    )
                finally:
                    harness.command = (0.0, 0.0)
                    harness.spin_for(0.1)
                    harness.destroy_node()
        except BaseException as error:
            record.update(status="fail", error=f"{type(error).__name__}: {error}")
        finally:
            rclpy.try_shutdown()
    dump_json(args.output / "result.json", record)
    print(record["status"], [(s["name"], s["passed"]) for s in record["stages"]])
    return 0 if record["status"] == "pass" else 1


if __name__ == "__main__":
    raise SystemExit(main())
