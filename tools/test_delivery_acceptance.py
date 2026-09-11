import json
from pathlib import Path
import tempfile
import unittest
from PIL import Image
import yaml
from map_accessibility import accessible_coverage, ROOT
from verify_delivery import audit


class DeliveryTests(unittest.TestCase):
    def test_gt_map_covers_accessible_region(self):
        result = accessible_coverage('simple', ROOT / 'src/sim_worlds/maps/simple.yaml')
        self.assertEqual(result['reachable_center_coverage'], 1.)

    def test_unknown_map_cannot_pass_accessible_coverage(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory)
            meta = yaml.safe_load((ROOT / 'src/sim_worlds/maps/simple.yaml').read_text())
            meta['image'] = 'map.pgm'
            (path / 'map.yaml').write_text(yaml.safe_dump(meta))
            Image.new('L', (200,200), 205).save(path / 'map.pgm')
            self.assertEqual(accessible_coverage('simple', path / 'map.yaml')['reachable_center_coverage'], 0.)

    def test_empty_artifacts_fail(self):
        with tempfile.TemporaryDirectory() as directory:
            self.assertFalse(audit(directory, directory, {'experiments': []})['passed'])


if __name__ == '__main__':
    unittest.main()
