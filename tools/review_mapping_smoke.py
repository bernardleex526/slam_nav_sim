#!/usr/bin/env python3
"""Bounded integration check. Partial maps are never selected for Stage B."""
import json
from pathlib import Path
import sys
import yaml

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'src/sim_benchmark/scripts'))
import stage_a_runner as a


def main():
    cfg = yaml.safe_load((ROOT / 'src/sim_benchmark/config/stage_a_matrix.yaml').read_text())
    cfg['runner'].update(max_explore_duration=45., warmup=2., cooldown=1.,
                         output_dir='~/.ros/sim_review_mapping')
    exp = dict(name='review_complex', robot='wheeled_quadruped', world='complex',
               mapping='slam_toolbox', use_gimbal=False)
    result = a.run_one(exp, cfg, 0, 1)
    run = Path(cfg['runner']['output_dir']).expanduser() / result['run_name']
    gt = str(ROOT / 'src/sim_worlds/maps/complex.yaml')
    result['report'] = a.evaluate(str(run), 'complex', gt)
    result['raw_report'] = a.evaluate(str(run), 'complex', gt, raw=True)
    result['passed'] = (result['status'] == 'EXPLORATION_INCOMPLETE'
                        and a.valid_map(run / 'map.yaml')
                        and bool(result['raw_report'])
                        and result['report']['trajectory'].get('matched_pairs', 0) >= 10)
    a.atomic_write(run / 'smoke_result.json', json.dumps(result, indent=2))
    print(json.dumps(result, indent=2))
    return 0 if result['passed'] else 1


if __name__ == '__main__':
    sys.exit(main())
