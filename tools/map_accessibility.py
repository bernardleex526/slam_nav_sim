#!/usr/bin/env python3
"""Independent footprint-clearance coverage diagnostic; retains global coverage."""
import argparse
import json
import math
from pathlib import Path
import sys
import numpy as np
from scipy.ndimage import distance_transform_edt, label
import yaml

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'src/sim_metrics/scripts'))
from evaluate_vs_gt import load_occupancy, align_and_crop


def accessible_coverage(world, map_yaml, robot='wheeled_quadruped'):
    gt, meta = load_occupancy(str(ROOT / f'src/sim_worlds/maps/{world}.yaml'))
    est, est_meta = load_occupancy(str(map_yaml))
    profile = yaml.safe_load((ROOT / 'src/robot_descriptions/config/robot_params.yaml').read_text())['robots'][robot]
    footprint = json.loads(profile['footprint']['vertices'])
    radius = max(math.hypot(x,y) for x,y in footprint)
    # Conservative circular envelope permits turning without wall contact.
    margin = .035
    distance = distance_transform_edt(np.pad(gt == 0, 1))[1:-1,1:-1]*meta['resolution']
    safe = (gt == 0) & (distance >= radius+margin)
    components, _ = label(safe)
    spawn = json.loads((ROOT / f'src/sim_worlds/maps/{world}_meta.json').read_text())['recommended_spawn']
    iy = gt.shape[0]-1-int((spawn['y']-meta['origin'][1])/meta['resolution'])
    ix = int((spawn['x']-meta['origin'][0])/meta['resolution'])
    if components[iy,ix] == 0:
        raise ValueError('Spawn lacks footprint clearance')
    reachable = components == components[iy,ix]
    mask = np.where(reachable, 0, 100).astype(np.int8)
    aligned_mask, aligned_est, _ = align_and_crop(mask, est, meta, est_meta)
    selected = aligned_mask == 0
    known = int(np.count_nonzero(selected & (aligned_est >= 0)))
    count = int(np.count_nonzero(selected))
    return dict(world=world, circumscribed_radius_m=radius, clearance_margin_m=margin,
                reachable_center_cells=count, observed_reachable_center_cells=known,
                reachable_center_coverage=known/count,
                gt_free_area_m2=int(np.count_nonzero(gt == 0))*meta['resolution']**2,
                reachable_center_area_m2=int(reachable.sum())*meta['resolution']**2,
                interpretation='Conservative in-place-turning envelope; not a proof of SE2 unreachability')


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('world')
    parser.add_argument('map_yaml')
    args = parser.parse_args()
    print(json.dumps(accessible_coverage(args.world, args.map_yaml), indent=2))
