import numpy as np
import pytest
from robot_validate.data import synthetic_frames, write_bag, read_bag, read_tum, write_tum
from robot_validate.slam import Grid


@pytest.mark.parametrize("ros2", [False, True])
def test_real_bag_roundtrip_and_no_truth(ros2, tmp_path):
    frames, spec, truth = synthetic_frames()
    path = tmp_path / ("bag2" if ros2 else "sample.bag")
    write_bag(path, frames[:20], spec, missing=range(5, 10), ros2=ros2)
    loaded = read_bag(path, spec)
    assert len(loaded) == 15
    assert loaded[5].distance == pytest.approx(sum(f.distance for f in frames[5:11]), abs=1e-6)
    write_tum(tmp_path / "truth.tum", truth)
    np.testing.assert_allclose(read_tum(tmp_path / "truth.tum")[:, :3], truth[:, :3], atol=1e-8)


def test_map_origin_unknown_and_orientation_roundtrip(tmp_path):
    grid = Grid(np.array([[0, -1], [100, 0]], np.int8), 0.1, (-2.5, 4.1, 0.3))
    restored = Grid.load(grid.save(tmp_path))
    np.testing.assert_array_equal(restored.cells, grid.cells)
    assert restored.origin == grid.origin
