from pathlib import Path
import subprocess
import unittest
import xml.etree.ElementTree as ET

ROOT = Path(__file__).resolve().parents[1]


class RobotDescriptionTests(unittest.TestCase):
    def test_agv_profile_has_independent_model(self):
        import yaml
        cfg = yaml.safe_load((ROOT / 'src/robot_descriptions/config/robot_params.yaml').read_text())
        profile = cfg['robots']['agv']
        result = subprocess.run(['xacro', str(ROOT / 'src/robot_descriptions' / profile['description_xacro'])],
                                text=True, capture_output=True, check=True)
        robot = ET.fromstring(result.stdout)
        names = {item.get('name') for item in robot.findall('link')}
        self.assertTrue({'base_footprint', 'base_link', 'left_wheel', 'right_wheel', 'front_caster'} <= names)

    def test_go2w_names_are_unique(self):
        path = ROOT / 'src/robot_descriptions/xacro/external/go2w/robot.urdf.xacro'
        result = subprocess.run(['xacro', str(path), 'use_gimbal:=false'],
                                text=True, capture_output=True, check=True)
        robot = ET.fromstring(result.stdout)
        for kind in ('joint', 'link'):
            names = [item.get('name') for item in robot.findall(kind)]
            self.assertEqual(len(names), len(set(names)), kind + ' names must be unique')


if __name__ == '__main__':
    unittest.main()
