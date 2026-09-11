#!/usr/bin/env python3
"""Verify actual obstacle motion via Gazebo state, not publisher counts."""
import json
import math
from pathlib import Path
import subprocess
import sys
import time

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'src/sim_benchmark/scripts'))
from stage_a_runner import kill_group
import rclpy
from rclpy.node import Node
from gazebo_msgs.srv import GetEntityState


def main():
    out = Path.home() / '.ros/sim_review_20260911/dynamic_obstacles'
    out.mkdir(parents=True, exist_ok=True)
    log = (out / 'launch.log').open('w')
    proc = subprocess.Popen(['ros2', 'launch', 'sim_bringup', 'sim.launch.py',
        'world:=medium', 'mapping:=none', 'navigation:=none', 'headless:=true',
        'viz:=false', 'use_gimbal:=false'], stdout=log, stderr=subprocess.STDOUT,
        start_new_session=True)
    rclpy.init()
    node = Node('dynamic_obstacle_regression')
    client = node.create_client(GetEntityState, '/gazebo/get_entity_state')
    samples = []
    try:
        if not client.wait_for_service(timeout_sec=90):
            raise RuntimeError('Gazebo state service not available')
        time.sleep(8)
        for _ in range(12):
            row = []
            for index in range(2):
                request = GetEntityState.Request()
                request.name = f'dyn_obstacle_{index}'
                request.reference_frame = 'world'
                future = client.call_async(request)
                rclpy.spin_until_future_complete(node, future, timeout_sec=3)
                if not future.done() or not future.result().success:
                    raise RuntimeError('Obstacle state unavailable')
                p = future.result().state.pose.position
                row.append([p.x, p.y])
            samples.append(row)
            time.sleep(.5)
        origins = json.loads((ROOT / 'src/sim_worlds/maps/medium_meta.json').read_text())['dynamic_obstacle_poses']
        spans = [max(row[i][0] for row in samples)-min(row[i][0] for row in samples) for i in range(2)]
        valid = all(span > .2 for span in spans) and all(
            math.dist(row[i], [origins[i]['x'], origins[i]['y']]) <= 1.25
            for row in samples for i in range(2))
        result = dict(passed=valid, x_spans=spans, samples=samples)
        (out / 'result.json').write_text(json.dumps(result, indent=2))
        print(json.dumps(dict(passed=valid, x_spans=spans)))
        return 0 if valid else 1
    finally:
        node.destroy_node()
        rclpy.shutdown()
        kill_group(proc)
        log.close()


if __name__ == '__main__':
    sys.exit(main())
