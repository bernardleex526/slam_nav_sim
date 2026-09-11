import importlib.util
from pathlib import Path
import struct
import unittest
from unittest.mock import Mock
import numpy as np
from sensor_msgs.msg import PointCloud2, PointField

ROOT = Path(__file__).resolve().parents[1]
spec = importlib.util.spec_from_file_location('scan', ROOT / 'src/mapping_adapters/nodes/points_to_scan.py')
scan = importlib.util.module_from_spec(spec)
spec.loader.exec_module(scan)


def cloud(padded=False, big=False):
    msg = PointCloud2()
    msg.width = 1
    msg.height = 2 if padded else 1
    msg.point_step = 12
    msg.row_step = 16 if padded else 12
    msg.is_bigendian = big
    msg.fields = [PointField(name=name, offset=i*4, datatype=PointField.FLOAT32, count=1)
                  for i, name in enumerate(('x', 'y', 'z'))]
    msg.data = struct.pack(('>' if big else '<')+'fff', 2., 0., 0.)
    if padded:
        msg.data.extend(b'XXXX'+struct.pack('<fff', 3., 0., 0.)+b'XXXX')
    return msg


class ScanTests(unittest.TestCase):
    def test_organized_row_padding(self):
        np.testing.assert_allclose(scan.cloud_to_xyz(cloud(padded=True)), [[2,0,0], [3,0,0]])

    def test_big_endian(self):
        np.testing.assert_allclose(scan.cloud_to_xyz(cloud(big=True)), [[2,0,0]])

    def test_angle_count_matches_scan_metadata(self):
        node = object.__new__(scan.PointsToScan)
        node._hmin, node._hmax = -.35, 1.2
        node._rmin, node._rmax, node._blind = .3, 40., .6
        node._bins, node._min_pts, node._miss = 720, 1, 0
        node._pub = Mock()
        node._on_cloud(cloud())
        msg = node._pub.publish.call_args.args[0]
        self.assertAlmostEqual(msg.angle_max, msg.angle_min+(len(msg.ranges)-1)*msg.angle_increment, places=5)


if __name__ == '__main__':
    unittest.main()
