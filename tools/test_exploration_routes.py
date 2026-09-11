import sys
from pathlib import Path
import unittest
import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'src/sim_benchmark/scripts'))
from exploration_routes import coverage_cells, segment_safe


class RoutesTests(unittest.TestCase):
    def test_coverage_routes_around_internal_wall(self):
        safe = np.ones((20, 30), dtype=bool)
        safe[:, 14] = False
        safe[2:5, 14] = True
        route, reachable = coverage_cells(safe, (10, 3), 4)
        self.assertTrue(any(x > 14 for y, x in route))
        self.assertEqual(reachable, int(safe.sum()))
        for left, right in zip(route, route[1:]):
            delta = np.asarray(right) - left
            self.assertTrue(segment_safe(left, right, safe))
            count = int(abs(delta).max())
            for point in np.linspace(left, right, count + 1).astype(int):
                self.assertTrue(safe[tuple(point)])

    def test_simplification_preserves_every_sweep_row(self):
        safe = np.ones((20, 30), dtype=bool)
        route, _ = coverage_cells(safe, (10, 3), 4)
        for y in (0, 4, 8, 12, 16, 19):
            self.assertIn((y, 0), route)
            self.assertIn((y, 29), route)

    def test_disconnected_free_space_is_not_claimed(self):
        safe = np.ones((12, 20), dtype=bool)
        safe[:, 10] = False
        route, reachable = coverage_cells(safe, (4, 3), 3)
        self.assertEqual(reachable, 120)
        self.assertTrue(all(x < 10 for y, x in route))


if __name__ == '__main__':
    unittest.main()
