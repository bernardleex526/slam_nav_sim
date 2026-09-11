#!/usr/bin/env python3
"""Cross-model interface acceptance; not part of planner ranking."""
import json
from pathlib import Path
import sys
import yaml

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'src/sim_benchmark/scripts'))
import stage_b_runner as b


def main():
    cfg = yaml.safe_load((ROOT / 'src/sim_benchmark/config/delivery_stage_b.yaml').read_text())
    cfg['runner'].update(output_dir='~/.ros/sim_delivery_checks/navigation', goals_per_run=1,
                         goal_timeout=90, cooldown=2)
    chosen = yaml.safe_load(Path(cfg['runner']['stage_a_selected']).expanduser().read_text())['selected']['simple']
    results = []
    for index, robot in enumerate(('agv', 'quadruped', 'go2w')):
        exp = dict(name='model_'+robot, robot=robot, world='simple',
                   global_planner='smac_2d', local_planner='dwb', use_gimbal=False)
        result = b.run_one(exp, cfg, index, 3, chosen['map_yaml'])
        results.append(result)
    output = Path(cfg['runner']['output_dir']).expanduser() / 'model_acceptance.json'
    output.write_text(json.dumps(results, indent=2))
    return 0 if all(r and r['status']=='DONE' and r['goal_statuses']==['SUCCEEDED'] for r in results) else 1


if __name__ == '__main__':
    sys.exit(main())
