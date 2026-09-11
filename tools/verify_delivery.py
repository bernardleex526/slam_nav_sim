#!/usr/bin/env python3
"""Fail-closed acceptance of real Stage A/B artifacts; never launch simulations."""
import argparse
import hashlib
import json
import math
from pathlib import Path
import sys
import yaml

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'src/sim_benchmark/scripts'))
from stage_a_runner import valid_map
from map_accessibility import accessible_coverage
from goal_endpoint_errors import endpoint_errors


def audit(a_root, b_root, config):
    a_root, b_root = Path(a_root).expanduser(), Path(b_root).expanduser()
    errors, maps, runs, coverage_limits = [], {}, [], []
    selection_file = a_root / 'stage_a_selected.yaml'
    selection = yaml.safe_load(selection_file.read_text()).get('selected', {}) if selection_file.exists() else {}
    for world in ('simple', 'medium', 'complex'):
        chosen = selection.get(world)
        if not chosen:
            errors.append(f'{world}: no qualified map')
            continue
        run = a_root / chosen['run_name']
        try:
            result = json.loads((run / 'run_result.json').read_text())
            report = json.loads((run / 'report.json').read_text())
            mp, trajectory = report['map'], report['trajectory']
            if result['status'] != 'DONE' or (run / 'exploration.done').read_text().splitlines()[0] != 'done':
                errors.append(f'{world}: exploration not fully completed')
            if not valid_map(chosen['map_yaml']):
                errors.append(f'{world}: invalid map artifact')
            accessibility = accessible_coverage(world, chosen['map_yaml'])
            if accessibility['reachable_center_coverage'] < .95:
                errors.append(f'{world}: less than 95% of the declared turning-safe envelope observed')
            if mp.get('coverage_ratio', 0) < .85:
                coverage_limits.append(f"{world}: global GT coverage {mp.get('coverage_ratio')} < .85; acceptance is limited to the turning-safe connected operating envelope")
            for key, threshold, direction in (('precision_tol2', .7, 'min'),
                                               ('ghost_wall_ratio', .3, 'max')):
                value = mp.get(key)
                if not isinstance(value, (int, float)) or not math.isfinite(value) or (value < threshold if direction == 'min' else value >= threshold):
                    errors.append(f'{world}: {key}={value} violates {direction} {threshold}')
            if trajectory.get('matched_pairs', 0) < 10:
                errors.append(f'{world}: no matched trajectory')
            if result.get('postprocessing') != 'none':
                errors.append(f'{world}: delivery requires an untrimmed map')
            maps[world] = dict(map=mp, trajectory=trajectory, accessibility=accessibility, run_name=chosen['run_name'])
        except (OSError, KeyError, ValueError, TypeError) as exc:
            errors.append(f'{world}: unreadable artifacts: {exc}')
    fairness, successes = {}, {}
    for exp in config['experiments']:
        for repeat in range(int(exp.get('repeats', 1))):
            name = f"B_{exp['name']}_r{repeat+1}"
            run = b_root / name
            try:
                result = json.loads((run / 'run_result.json').read_text())
                metrics = json.loads((run / 'metrics.json').read_text())['navigation']
                count = int(config['runner']['goals_per_run'])
                statuses = result.get('goal_statuses', [])
                if result['status'] != 'DONE' or len(statuses) != count:
                    errors.append(f'{name}: missing completed goal attempts')
                if any(s not in ('SUCCEEDED', 'ABORTED', 'CANCELED', 'TIMEOUT', 'REJECTED') for s in statuses):
                    errors.append(f'{name}: unknown action outcome')
                terminal = sum(s in ('SUCCEEDED', 'ABORTED', 'CANCELED', 'TIMEOUT') for s in statuses)
                if metrics.get('goals_total') != terminal:
                    errors.append(f'{name}: action and metrics goal counts disagree')
                source = Path(result['map_yaml'])
                meta = yaml.safe_load(source.read_text())
                digest = hashlib.sha256((source.parent / meta['image']).read_bytes()).hexdigest()
                signature = (result['goals'], digest, result.get('rmw_implementation'), exp['robot'])
                key = (exp['world'], repeat)
                if key in fairness and fairness[key] != signature:
                    errors.append(f'{name}: comparison used different goals or map pixels')
                fairness[key] = signature
                for metric in ('collision_count', 'control_smoothness', 'min_obstacle_dist'):
                    if metrics.get(metric) is None:
                        errors.append(f'{name}: missing {metric}')
                successes[exp['world']] = successes.get(exp['world'], 0) + statuses.count('SUCCEEDED')
                endpoints = endpoint_errors(run)
                for endpoint in endpoints:
                    if endpoint['action_status'] == 'SUCCEEDED' and (endpoint['gt_error_m'] is None or endpoint['gt_error_m'] > .5):
                        errors.append(f"{name}: nominal success lacks GT arrival within 0.5m")
                runs.append(dict(result=result, metrics=metrics, map_sha256=digest, gt_endpoints=endpoints))
            except (OSError, KeyError, ValueError, TypeError) as exc:
                errors.append(f'{name}: unreadable artifacts: {exc}')
    for world in ('simple', 'medium', 'complex'):
        if not successes.get(world):
            errors.append(f'{world}: no navigation success')
    return dict(passed=not errors, errors=errors, maps=maps, runs=runs,
                coverage_limits=coverage_limits, full_environment_coverage_passed=not coverage_limits and len(maps)==3,
                scope='Ideal-planar SLAM Toolbox + AMCL/Nav2 benchmark within the declared turning-safe operating envelope; not full narrow-maze coverage',
                thresholds=dict(min_reachable_center_coverage=.95, full_environment_coverage=.85,
                                min_precision_tol2=.7, max_ghost_exclusive=.3, successful_goal_gt_tolerance_m=.5))


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--stage-a', default='~/.ros/sim_delivery_stage_a')
    parser.add_argument('--stage-b', default='~/.ros/sim_delivery_stage_b')
    parser.add_argument('--out', default='~/.ros/sim_delivery_acceptance.json')
    args = parser.parse_args()
    cfg = yaml.safe_load((ROOT / 'src/sim_benchmark/config/delivery_stage_b.yaml').read_text())
    result = audit(args.stage_a, args.stage_b, cfg)
    Path(args.out).expanduser().write_text(json.dumps(result, ensure_ascii=False, indent=2))
    print(json.dumps(dict(passed=result['passed'], errors=result['errors']), ensure_ascii=False, indent=2))
    return 0 if result['passed'] else 1


if __name__ == '__main__':
    sys.exit(main())
