#!/usr/bin/env python3
"""Exercise contact reporting using a deliberate wall overlap in simulation."""
import json
from pathlib import Path
import subprocess
import sys
import time

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'src/sim_benchmark/scripts'))
from stage_a_runner import kill_group
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
from gazebo_msgs.srv import SetEntityState
from nav_msgs.msg import Odometry


def main():
    out = Path.home() / '.ros/sim_delivery_checks/contact'
    out.mkdir(parents=True, exist_ok=True)
    log = (out / 'sim.log').open('w')
    process = subprocess.Popen(['ros2', 'launch', 'sim_bringup', 'sim.launch.py',
        'world:=simple', 'mapping:=none', 'navigation:=none', 'headless:=true',
        'use_gimbal:=false', 'viz:=false'], stdout=log, stderr=subprocess.STDOUT,
        start_new_session=True)
    rclpy.init()
    node = Node('contact_acceptance')
    values, odom = [], []
    node.create_subscription(Bool, '/eval/contact', lambda m: values.append(m.data), 10)
    node.create_subscription(Odometry, '/odom', lambda m: odom.append(m), 1)
    client = node.create_client(SetEntityState, '/gazebo/set_entity_state')
    def spin(seconds):
        end = time.monotonic()+seconds
        while time.monotonic() < end:
            rclpy.spin_once(node, timeout_sec=.1)
    try:
        deadline = time.monotonic()+120
        while time.monotonic() < deadline and not (values and odom and client.service_is_ready()):
            spin(.2)
        if not values or not odom:
            raise RuntimeError('No contact heartbeat or odometry')
        spin(1)
        free = values[-10:]
        values.clear()
        for _ in range(20):
            req = SetEntityState.Request()
            req.state.name = 'wheeled_quadruped_sim'
            req.state.reference_frame = 'world'
            req.state.pose.position.x = 9.85
            req.state.pose.orientation.w = 1.
            future = client.call_async(req)
            rclpy.spin_until_future_complete(node, future, timeout_sec=2)
            if not future.done() or not future.result().success:
                raise RuntimeError('Cannot place test robot at wall')
            spin(.1)
        result = dict(free_contact=any(free), wall_contact=any(values), samples=len(values))
        result['passed'] = not result['free_contact'] and result['wall_contact']
        (out / 'result.json').write_text(json.dumps(result, indent=2))
        print(json.dumps(result))
        return 0 if result['passed'] else 1
    finally:
        node.destroy_node()
        rclpy.shutdown()
        kill_group(process)
        log.close()


if __name__ == '__main__':
    sys.exit(main())
