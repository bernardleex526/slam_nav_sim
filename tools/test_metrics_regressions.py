import importlib.util
from pathlib import Path
from types import SimpleNamespace
import unittest
import tempfile
from unittest.mock import Mock
import numpy as np

ROOT = Path(__file__).resolve().parents[1]

def load(name, path):
    spec = importlib.util.spec_from_file_location(name, ROOT / path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module

metrics = load('metrics', 'src/sim_metrics/nodes/metrics_collector.py')
evaluate = load('evaluate', 'src/sim_metrics/scripts/evaluate_vs_gt.py')


class MetricsTests(unittest.TestCase):
    def test_contact_episode_is_counted_once(self):
        from std_msgs.msg import Bool
        node = object.__new__(metrics.MetricsCollector)
        node._contact_seen = False
        node._contact_active = False
        node._col_streak = 0
        node._coll_persist = 3
        node._col_count = 0
        for value in [True]*10+[False]+[True]*3:
            node._on_contact(Bool(data=value))
        self.assertEqual(node._col_count, 2)

    def test_interrupted_csv_write_preserves_previous_complete_file(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / 'trajectory.csv'
            path.write_text('previous complete data')
            def interrupted_rows():
                yield [1., 2.]
                raise KeyboardInterrupt()
            with self.assertRaises(KeyboardInterrupt):
                metrics.MetricsCollector._write_csv(str(path), ['t', 'x'], interrupted_rows())
            self.assertEqual(path.read_text(), 'previous complete data')

    def test_control_variation_handles_duplicate_sim_timestamps(self):
        node = object.__new__(metrics.MetricsCollector)
        node._goal_state = 'IDLE'
        node._goals = []
        node._min_obs = float('inf')
        node._gt = node._est = node._track_err = []
        node._cmd = [(1., 0., 0.), (1., 1., 0.), (2., 0., 0.)]
        self.assertEqual(node._nav_metrics()['control_smoothness'], 2.)

    def test_ground_truth_does_not_follow_slam_correction(self):
        from nav_msgs.msg import Odometry
        node = object.__new__(metrics.MetricsCollector)
        node._gt = []
        node._use_frame = None
        node._target = 'map'
        node._fallback = 'odom'
        node._buffer = Mock()
        node._buffer.lookup_transform.return_value = SimpleNamespace(transform=SimpleNamespace(
            translation=SimpleNamespace(x=10., y=20.),
            rotation=SimpleNamespace(x=0., y=0., z=0., w=1.)))
        msg = Odometry()
        msg.header.frame_id = 'odom'
        msg.pose.pose.position.x = 2.
        msg.pose.pose.position.y = 3.
        msg.pose.pose.orientation.w = 1.
        node._on_gt(msg)
        self.assertEqual(node._gt[0][1:3], (2., 3.))
        node._buffer.lookup_transform.assert_not_called()

    def test_nearest_time_uses_left_neighbor(self):
        gt = np.array([(float(i), float(i), 0., 0.) for i in range(15)])
        est = gt.copy()
        est[:, 0] += .01
        matched = evaluate.match_by_time(gt, est)
        self.assertIsNotNone(matched)
        self.assertEqual(len(matched[0]), 14)


if __name__ == '__main__':
    unittest.main()
