import importlib.util
from pathlib import Path
import unittest
from geometry_msgs.msg import Pose

ROOT = Path(__file__).resolve().parents[1]
spec = importlib.util.spec_from_file_location('driver', ROOT / 'src/sim_worlds/scripts/dynamic_obstacle_driver.py')
driver = importlib.util.module_from_spec(spec)
spec.loader.exec_module(driver)


class DynamicObstacleTests(unittest.TestCase):
    def test_motion_is_relative_to_world_spawn(self):
        base = Pose()
        base.position.x = 12.
        base.position.y = -7.
        base.position.z = .6
        base.orientation.w = 1.
        state = driver.obstacle_state('dyn_obstacle_0', base, 0, 2, 3., 1.2, 12.)
        self.assertEqual(state.name, 'dyn_obstacle_0')
        self.assertAlmostEqual(state.pose.position.x, 13.2)
        self.assertAlmostEqual(state.pose.position.y, -7.)
        self.assertAlmostEqual(state.pose.position.z, .6)
        self.assertAlmostEqual(base.position.x, 12.)


if __name__ == '__main__':
    unittest.main()
