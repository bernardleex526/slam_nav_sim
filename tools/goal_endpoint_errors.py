#!/usr/bin/env python3
"""Compare action completion with independent GT position at completion time."""
import csv
import json
import math
from pathlib import Path
import numpy as np


def endpoint_errors(run):
    run = Path(run)
    manifest = json.loads((run / 'run_result.json').read_text())
    metrics = json.loads((run / 'metrics.json').read_text())
    with (run / 'trajectory_gt.csv').open() as stream:
        trajectory = np.asarray([(float(r['t']), float(r['x']), float(r['y'])) for r in csv.DictReader(stream)])
    rows, index = [], 0
    for goal, status in zip(manifest['goals'], manifest.get('goal_statuses', [])):
        row = dict(goal=goal, action_status=status, gt_error_m=None)
        if status != 'REJECTED' and index < len(metrics.get('goals', [])):
            ended = metrics['goals'][index]['end']
            index += 1
            if trajectory.size:
                nearest = int(np.argmin(np.abs(trajectory[:, 0]-ended)))
                if abs(trajectory[nearest, 0]-ended) < .5:
                    row['gt_error_m'] = round(math.dist(trajectory[nearest, 1:], goal), 4)
        rows.append(row)
    return rows


if __name__ == '__main__':
    for run in sorted((Path.home()/'.ros/sim_delivery_stage_b').glob('B_*')):
        if (run/'run_result.json').exists():
            print(run.name, json.dumps(endpoint_errors(run)))
