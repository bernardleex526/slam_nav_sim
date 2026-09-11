import importlib.util
from pathlib import Path
import unittest
import tempfile

ROOT = Path(__file__).resolve().parents[1]
spec = importlib.util.spec_from_file_location('compose', ROOT / 'src/nav2_config/tools/compose_nav2_params.py')
compose = importlib.util.module_from_spec(spec)
spec.loader.exec_module(compose)


class NavConfigTests(unittest.TestCase):
    def test_launch_parameter_files_are_unique(self):
        spec = importlib.util.spec_from_file_location('nav_launch', ROOT / 'src/nav2_config/launch/nav2.launch.py')
        launch = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(launch)
        with tempfile.TemporaryDirectory() as directory:
            first = launch.unique_params_path(directory, 'same_profile')
            second = launch.unique_params_path(directory, 'same_profile')
            self.assertNotEqual(first, second)
            self.assertTrue(Path(first).is_file())

    def test_footprint_is_data_not_python(self):
        cfg = {'footprint': {'shape': 'polygon', 'vertices': '[[1/2,1/4],[-1/2,-1/4]]'}}
        with self.assertRaises(ValueError):
            compose.build_robot_overlay(cfg, 'gt')


if __name__ == '__main__':
    unittest.main()
