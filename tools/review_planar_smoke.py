#!/usr/bin/env python3
"""Inject a roll/pitch disturbance and verify the ideal planar model recovers."""
import argparse
import json
import math
from pathlib import Path
import subprocess
import sys
import time

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src/sim_benchmark/scripts"))
from stage_a_runner import kill_group
import rclpy
from gazebo_msgs.srv import GetEntityState, SetEntityState
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--robot", default="wheeled_quadruped")
    args = parser.parse_args()
    out = Path.home() / ".ros/sim_review_20260911" / ("planar_" + args.robot)
    out.mkdir(parents=True, exist_ok=True)
    log = (out / "launch.log").open("w")
    proc = subprocess.Popen(["ros2", "launch", "sim_bringup", "sim.launch.py",
        "robot:="+args.robot, "world:=simple", "mapping:=none", "navigation:=none",
        "use_gimbal:=false", "headless:=true", "viz:=false"],
        stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
    rclpy.init()
    node = Node("planar_regression")
    get = node.create_client(GetEntityState, "/gazebo/get_entity_state")
    set_state = node.create_client(SetEntityState, "/gazebo/set_entity_state")
    pub = node.create_publisher(Twist, "/cmd_vel", 10)
    odometry = []
    node.create_subscription(Odometry, "/odom", lambda msg: odometry.append(msg), 1)
    def call(client, req):
        future = client.call_async(req)
        rclpy.spin_until_future_complete(node, future, timeout_sec=3)
        if not future.done() or not future.result().success:
            return None
        return future.result()
    result = {"robot": args.robot, "passed": False}
    try:
        deadline = time.monotonic()+100
        request = GetEntityState.Request()
        request.name = args.robot + "_sim"
        request.reference_frame = "world"
        initial = None
        while time.monotonic() < deadline:
            if get.wait_for_service(timeout_sec=.5) and set_state.service_is_ready():
                initial = call(get, request)
                if initial and odometry:
                    break
            time.sleep(.1)
        if initial is None or not odometry:
            raise RuntimeError("Robot did not spawn")
        disturbance = SetEntityState.Request()
        disturbance.state = initial.state
        disturbance.state.name = request.name
        disturbance.state.reference_frame = "world"
        disturbance.state.pose.position.z = .4
        disturbance.state.pose.orientation.x = math.sin(.4/2)
        disturbance.state.pose.orientation.y = 0.
        disturbance.state.pose.orientation.z = 0.
        disturbance.state.pose.orientation.w = math.cos(.4/2)
        if call(set_state, disturbance) is None:
            raise RuntimeError("Could not inject disturbance")
        samples = []
        for _ in range(30):
            command = Twist()
            command.linear.x = .2
            command.angular.z = .3
            pub.publish(command)
            time.sleep(.1)
            state = call(get, request)
            if state:
                p = state.state.pose
                samples.append(dict(z=p.position.z, qx=p.orientation.x, qy=p.orientation.y,
                                    x=p.position.x, y=p.position.y))
        result.update(samples=samples, max_z=max(abs(s['z']) for s in samples),
                      max_tilt=max(math.hypot(s['qx'], s['qy']) for s in samples))
        result['passed'] = len(samples) >= 20 and result['max_z'] < .01 and result['max_tilt'] < .01
        print(json.dumps({k: v for k, v in result.items() if k != 'samples'}))
        return 0 if result['passed'] else 1
    finally:
        pub.publish(Twist())
        node.destroy_node()
        rclpy.shutdown()
        kill_group(proc)
        log.close()
        (out / 'result.json').write_text(json.dumps(result, indent=2))


if __name__ == '__main__':
    sys.exit(main())
