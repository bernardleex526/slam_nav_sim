import importlib.util
from pathlib import Path
import unittest
from unittest.mock import patch
from launch import LaunchContext

ROOT = Path(__file__).resolve().parents[1]
spec = importlib.util.spec_from_file_location('mapping_common', ROOT / 'src/mapping_adapters/launch/mapping_common.py')
module = importlib.util.module_from_spec(spec)
spec.loader.exec_module(module)


class MappingLaunchTests(unittest.TestCase):
    def test_missing_algorithm_is_not_silent_adapter_only_success(self):
        context = LaunchContext()
        context.launch_configurations.update(external_launch='', lidar_mode='points',
            publish_2d_map='true', publish_downsample_levels='true',
            start_points_to_scan='true', start_external_algorithm='true')
        with patch.object(module, 'find_external_launch', return_value=None):
            with self.assertRaises(RuntimeError):
                module.setup_mapping(context, 'fast_lio')


if __name__ == '__main__':
    unittest.main()
