#!/usr/bin/env python3
"""Fast regression checks; no Gazebo process is started."""
import json
from pathlib import Path
import sys
import tempfile
import unittest
from unittest.mock import patch
import yaml

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'src/sim_benchmark/scripts'))
import stage_a_runner as a
import stage_b_runner as b


class BenchmarkTests(unittest.TestCase):
    def test_navigation_goals_stay_in_spawn_component(self):
        import numpy as np
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / 'map.yaml'
            path.write_text('image: map.pgm\nresolution: 1.0\norigin: [0,0,0]\n')
            image = np.full((30, 60), 254, dtype=np.uint8)
            image[:, 20] = 0
            (path.parent / 'map.pgm').write_bytes(b'P5\n60 30\n255\n'+image.tobytes())
            goals = b.sample_goals(str(path), 5, (5., 15.), seed=1000, clearance=1.)
            self.assertTrue(all(x < 20 for x, y in goals))

    def test_map_server_has_rewrite_target(self):
        config = yaml.safe_load((ROOT / 'src/nav2_config/config/base/nav2_params.yaml').read_text())
        self.assertIn('yaml_filename', config['map_server']['ros__parameters'])

    def test_missing_metrics_do_not_crash_flatten(self):
        row = a.flatten(dict(name='test', world='simple', mapping='slam_toolbox', robot='test'),
                        dict(run_name='A_test'), {'map': {'coverage_ratio': 1.0}})
        self.assertIsNone(row['map_score'])

    def test_zero_score_stays_zero(self):
        self.assertEqual(a.map_score(dict(coverage_ratio=1, occupied_recall=0,
                                         occupied_iou=1, occupied_precision=1)), 0)

    def test_unusable_map_is_not_selected(self):
        rows = [dict(world='simple', map_score=.9, ghost_wall_ratio=.31),
                dict(world='simple', map_score=.8, ghost_wall_ratio=.02)]
        self.assertEqual(a.pick_best_map(rows)['simple']['map_score'], .8)

    def test_missing_ghost_metric_is_not_selected(self):
        self.assertEqual(a.pick_best_map([dict(world='simple', map_score=.9)]), {})

    def test_startup_does_not_succeed_without_evidence(self):
        with patch.object(a, 'sh', return_value=a._Result(0, '', '')), \
             patch.object(a.time, 'time', side_effect=[0, 1, 2, 3, 100, 101]), \
             patch.object(a.time, 'sleep'):
            self.assertFalse(a.wait_for_topic('/scan', 10, settle=0)[0])

    def test_navigation_timeout_counts_all_planned_goals(self):
        row = b.flatten(dict(name='test', world='simple', global_planner='navfn', local_planner='dwb'),
                        dict(run_name='B_test', goals_requested=5,
                             goal_statuses=['SUCCEEDED', 'TIMEOUT']), {})
        self.assertEqual(row['success_rate'], .2)

    def test_navigation_gimbal_matches_mapping_default(self):
        cmd = b.build_cmd(dict(robot='test', world='simple', global_planner='navfn',
                               local_planner='dwb'), {}, 'test', '/tmp/map.yaml')
        self.assertIn('use_gimbal:=false', cmd)

    def test_frozen_maps_do_not_change_with_stage_a(self):
        with tempfile.TemporaryDirectory() as temp:
            root = Path(temp)
            source = root / 'map.yaml'
            source.write_text('image: map.pgm\nresolution: 0.05\norigin: [0,0,0]\n')
            image = root / 'map.pgm'
            original = b'P5\n1 1\n255\n\x00'
            image.write_bytes(original)
            frozen = b.freeze_maps({'simple': {'map_yaml': str(source)}}, root / 'frozen')
            image.write_bytes(b'changed')
            target = Path(frozen['simple']['map_yaml'])
            meta = yaml.safe_load(target.read_text())
            self.assertEqual((target.parent / meta['image']).read_bytes(), original)

    def test_all_worlds_survive_resummary(self):
        with tempfile.TemporaryDirectory() as temp:
            root = Path(temp)
            exps = []
            for world in ('simple', 'medium', 'complex'):
                exp = dict(name='test_' + world, world=world, robot='test', mapping='slam_toolbox')
                exps.append(exp)
                run = root / ('A_' + exp['name'])
                run.mkdir()
                (run / 'map.pgm').write_bytes(b'P5\n1 1\n255\n\x00')
                (run / 'map.yaml').write_text('image: map.pgm\nresolution: 0.05\norigin: [0,0,0]\n')
                (run / 'exploration.done').write_text('done\n10 points\n')
                (run / 'report.json').write_text(json.dumps({'map': dict(
                    coverage_ratio=1, occupied_recall=.7, occupied_iou=.5,
                    occupied_precision=.8, ghost_wall_ratio=.01)}))
            a.resummarize(root, {'experiments': exps})
            a.resummarize(root, {'experiments': exps})
            selected = yaml.safe_load((root / 'stage_a_selected.yaml').read_text())['selected']
            self.assertEqual(set(selected), {'simple', 'medium', 'complex'})

    def test_timeout_sentinel_is_not_completion(self):
        with tempfile.TemporaryDirectory() as temp:
            path = Path(temp) / 'exploration.done'
            path.write_text('timeout\n100 points\n')
            self.assertFalse(a.exploration_complete(path))
            path.write_text('done_with_stuck_skips\n100 points\n')
            self.assertFalse(a.exploration_complete(path))
            path.write_text('done\n100 points\n')
            self.assertTrue(a.exploration_complete(path))


if __name__ == '__main__':
    unittest.main()
