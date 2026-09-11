#!/usr/bin/env python3
"""Render measured artifacts into a portable report directory."""
import argparse
import csv
import json
from pathlib import Path
import shutil
import sys

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from PIL import Image
import numpy as np
import yaml

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'src/sim_metrics/scripts'))
from evaluate_vs_gt import read_traj
from verify_delivery import audit


def map_panel(ax, path, title):
    meta = yaml.safe_load(path.read_text())
    with Image.open(path.parent / meta['image']) as source:
        image = np.asarray(source)
    x, y = meta['origin'][:2]
    res = meta['resolution']
    ax.imshow(image, cmap='gray', vmin=0, vmax=255,
              extent=[x, x+image.shape[1]*res, y, y+image.shape[0]*res])
    ax.set_title(title, fontsize=11)
    ax.set_xlabel('x (m)')
    ax.set_ylabel('y (m)')
    ax.set_aspect('equal')


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--output', required=True)
    args = parser.parse_args()
    out = Path(args.output).resolve()
    out.mkdir(parents=True, exist_ok=True)
    a_root = Path.home() / '.ros/sim_delivery_stage_a'
    b_root = Path.home() / '.ros/sim_delivery_stage_b'
    config = yaml.safe_load((ROOT / 'src/sim_benchmark/config/delivery_stage_b.yaml').read_text())
    result = audit(a_root, b_root, config)
    (out / 'acceptance.json').write_text(json.dumps(result, ensure_ascii=False, indent=2))
    lines = ['# Simulation Platform Delivery', '',
             '**Operating-envelope acceptance: '+('PASS' if result['passed'] else 'NOT COMPLETE')+'**', '',
             '**Full-environment coverage: '+('PASS' if result['full_environment_coverage_passed'] else 'NOT PASS')+'**', '',
             'Scope: ideal-planar SLAM Toolbox mapping, AMCL localization and eight Nav2 combinations.',
             'No map cropping was used in this delivery experiment. All values below come from saved run artifacts.', '',
             '## Mapping', '',
             '| World | Coverage | Precision @10cm | Ghost ratio | Raw ATE (m) | RPE (m) |',
             '|---|---:|---:|---:|---:|---:|']
    for world, row in result['maps'].items():
        mp, tp = row['map'], row['trajectory']
        lines.append(f"| {world} | {mp['coverage_ratio']:.4f} | {mp['precision_tol2']:.4f} | {mp['ghost_wall_ratio']:.4f} | {tp.get('ate_rmse_raw')} | {tp.get('rpe_trans_rmse')} |")
        run = a_root / row['run_name']
        target = out / 'maps' / world
        target.mkdir(parents=True, exist_ok=True)
        for name in ('map.yaml', 'map.pgm', 'report.json', 'report_raw.json', 'run_result.json',
                     'coverage_plan.json', 'exploration.done', 'trajectory_gt.csv', 'trajectory_est.csv'):
            if (run / name).is_file():
                shutil.copy2(run / name, target / name)
        fig, axes = plt.subplots(1, 3, figsize=(14, 4.5), layout='constrained')
        map_panel(axes[0], ROOT / f'src/sim_worlds/maps/{world}.yaml', world+' / ground truth')
        map_panel(axes[1], run / 'map.yaml', world+' / untrimmed SLAM')
        for name, label, color in [('trajectory_gt.csv', 'Ground truth', '#207a61'),
                                    ('trajectory_est.csv', 'SLAM estimate', '#c34b58')]:
            trajectory = read_traj(str(run / name))
            if trajectory is not None:
                axes[2].plot(trajectory[:, 1], trajectory[:, 2], lw=.8, label=label, color=color)
        axes[2].set_title(world+' / trajectories')
        axes[2].set_aspect('equal')
        axes[2].legend(fontsize=8)
        axes[2].set_xlabel('x (m)')
        axes[2].set_ylabel('y (m)')
        fig.savefig(out / (world+'_mapping.png'), dpi=150)
        plt.close(fig)
    lines += ['', '## Coverage Limits', '']
    lines += ['- '+limit for limit in result['coverage_limits']] or ['None.']
    for world, row in result['maps'].items():
        a = row['accessibility']
        lines.append(f"- {world}: turning-safe center coverage {a['reachable_center_coverage']:.1%}; circumscribed radius {a['circumscribed_radius_m']:.3f}m + margin {a['clearance_margin_m']:.3f}m. This is a conservative operating envelope, not a proof of SE2 unreachability.")
    lines += ['', '## Navigation', '',
              '| Experiment | Succeeded / attempted | Goal states | Mean time (s) | Tracking RMSE (m) | Contact episodes |',
              '|---|---:|---|---:|---:|---:|']
    table = []
    for entry in result['runs']:
        run, metrics = entry['result'], entry['metrics']
        states = run.get('goal_statuses', [])
        lines.append(f"| {run['experiment']} | {states.count('SUCCEEDED')} / {len(states)} | {', '.join(states)} | {metrics.get('time_to_goal_mean')} | {metrics.get('tracking_error_rmse')} | {metrics.get('collision_count')} |")
        table.append(dict(experiment=run['experiment'], world=run['world'], success=states.count('SUCCEEDED'),
                          attempts=len(states), success_rate=states.count('SUCCEEDED')/max(1,len(states)),
                          mean_time=metrics.get('time_to_goal_mean'), tracking_rmse=metrics.get('tracking_error_rmse'),
                          contacts=metrics.get('collision_count'), map_sha256=entry['map_sha256']))
        target = out / 'navigation' / run['run_name']
        target.mkdir(parents=True, exist_ok=True)
        (target / 'gt_endpoint_errors.json').write_text(json.dumps(entry['gt_endpoints'], indent=2))
        for name in ('run_result.json', 'metrics.json', 'trajectory_gt.csv', 'trajectory_est.csv', 'launch.log'):
            source = b_root / run['run_name'] / name
            if source.is_file():
                shutil.copy2(source, target / name)
    if table:
        with (out / 'navigation_summary.csv').open('w', newline='') as stream:
            writer = csv.DictWriter(stream, fieldnames=list(table[0]))
            writer.writeheader()
            writer.writerows(table)
        fig, ax = plt.subplots(figsize=(10, 4.5), layout='constrained')
        ax.barh([r['experiment'] for r in table], [r['success_rate'] for r in table], color='#247f77')
        ax.set_xlim(0, 1)
        ax.set_xlabel('Success rate (five fixed goals, one repeat)')
        ax.invert_yaxis()
        fig.savefig(out / 'navigation_success.png', dpi=150)
        plt.close(fig)
    lines += ['', '## Interpretation Limits', '',
              '- One repeat per combination; no statistical ranking or confidence claims.',
              '- Goals are sequential within each run; an earlier failure can change the starting pose of later attempts.',
              '- Mean time includes both successful and failed attempts; compare it jointly with success rate.',
              '- Dynamic obstacles move during runs; contact episodes exclude ground/self contacts.',
              '- GT-assisted coverage is a controlled benchmark, not autonomous unknown-space exploration.',
              '- GT solid obstacle interiors and observed surfaces differ; recall alone is not a geometry diagnosis.',
              '- Ideal planar motion is not a validated legged gait, tire-slip or M20 Pro dynamics model.',
              '', '## Acceptance Errors', '']
    lines += ['- '+error for error in result['errors']] or ['None.']
    (out / 'DELIVERY.md').write_text('\n'.join(lines)+'\n', encoding='utf-8')
    shutil.copy2(ROOT / 'src/sim_benchmark/config/delivery_stage_a.yaml', out / 'delivery_stage_a.yaml')
    shutil.copy2(ROOT / 'src/sim_benchmark/config/delivery_stage_b.yaml', out / 'delivery_stage_b.yaml')
    print(out / 'DELIVERY.md')
    return 0 if result['passed'] else 1


if __name__ == '__main__':
    sys.exit(main())
