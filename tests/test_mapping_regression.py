"""Regression at the sensor-bag -> estimated map seam, with reference only in evaluation."""

import numpy as np
import pytest
from robot_validate.data import synthetic_frames, write_bag, read_bag
from robot_validate.metrics import map_checks, trajectory_checks
from robot_validate.model import DEFAULT_THRESHOLDS, sha256
from robot_validate.runner import reference_grid
from robot_validate.slam import run_breezy


@pytest.mark.parametrize("sensor_seed", [391, 614, 1009])
def test_original_and_heldout_inputs_meet_unchanged_map_gates(tmp_path, sensor_seed):
    frames, spec, reference = synthetic_frames(seed=sensor_seed)
    bag = tmp_path / "input.bag"
    write_bag(bag, frames, spec)
    if sensor_seed == 391:
        assert sha256(bag) == "f76e2220228ad0a802b6d818a73ebc499e290b371d71365ec45135b9d2978f31"
    frames = read_bag(bag, spec)
    poses, grid = run_breezy(frames, spec, 9999, tmp_path, (2, 2, 0))
    checks, metrics, _ = map_checks(grid, reference_grid(), DEFAULT_THRESHOLDS)
    assert all(c.status == "pass" for c in checks), metrics
    assert trajectory_checks(poses, reference, DEFAULT_THRESHOLDS)[0][0].status == "pass"


def test_initial_frame_rotation_applies_to_both_map_and_poses(tmp_path):
    from robot_validate.scan_matcher import rotate

    frames, spec, _ = synthetic_frames()
    one = tmp_path / "one"
    two = tmp_path / "two"
    one.mkdir()
    two.mkdir()
    poses_a, grid_a = run_breezy(frames[:50], spec, 1, one)
    poses_b, grid_b = run_breezy(frames[:50], spec, 1, two, (4, -3, 0.7))
    np.testing.assert_array_equal(grid_a.cells, grid_b.cells)
    np.testing.assert_allclose(poses_b[:, 1:3], rotate(poses_a[:, 1:3], 0.7) + [4, -3])
    np.testing.assert_allclose(
        grid_b.origin[:2], rotate(np.array([grid_a.origin[:2]]), 0.7)[0] + [4, -3]
    )


def test_map_gray_conversion_matches_declared_yaml_thresholds():
    from robot_validate.slam import grayscale_to_cells

    values = np.array([[0, 65, 89, 90, 127, 196, 205, 206, 255]], np.uint8)
    assert grayscale_to_cells(values).tolist() == [[100, 100, 100, -1, -1, -1, -1, 0, 0]]


def test_rolling_scan_is_preserved_and_rejected_without_deskew(tmp_path):
    from dataclasses import replace

    frames, spec, _ = synthetic_frames()
    frames = [replace(f, beam_dt_s=0.0001) for f in frames[:10]]
    write_bag(tmp_path / "rolling.bag", frames, spec)
    restored = read_bag(tmp_path / "rolling.bag", spec)
    assert restored[0].beam_dt_s == pytest.approx(0.0001)
    with pytest.raises(ValueError, match="deskewed"):
        run_breezy(restored, spec, 1, tmp_path)


def test_degenerate_geometry_retains_odometry_and_reports_uncertainty():
    from robot_validate.data import Scan, LaserSpec
    from robot_validate.scan_matcher import ScanMatcher

    spec = LaserSpec(size=121, fov=120, margin=0)
    angles = np.linspace(-np.pi / 3, np.pi / 3, 121)
    ranges = 2 / np.cos(angles)
    matcher = ScanMatcher(spec)
    matcher.update(Scan(0, ranges, 0, 0, 0.1))
    pose, diagnostic = matcher.update(Scan(0.1, ranges, 0.01, 0, 0.1))
    assert diagnostic["state"] == "degenerate_geometry"
    np.testing.assert_allclose(pose, [0.01, 0, 0])


def test_lost_matches_do_not_insert_an_odometry_fallback_keyframe():
    from dataclasses import replace
    from robot_validate.scan_matcher import ScanMatcher

    frames, spec, _ = synthetic_frames()
    matcher = ScanMatcher(spec)
    matcher.update(frames[0])
    tree = matcher.tree
    points = matcher.points.copy()

    class DropAfterFirstQuery:
        calls = 0

        def query(self, values):
            self.calls += 1
            distances, indices = tree.query(values)
            if self.calls > 1:
                distances[:] = 100
            return distances, indices

    matcher.tree = DropAfterFirstQuery()
    pose, diagnostic = matcher.update(replace(frames[1], distance=0.5))
    assert matcher.tree.calls > 1
    assert diagnostic["state"] == "insufficient_geometry"
    assert diagnostic["matches"] == 0
    assert diagnostic["residual_rmse_m"] is None
    np.testing.assert_allclose(pose, [0.5, 0, frames[1].yaw])
    np.testing.assert_array_equal(matcher.points, points)


def test_excluded_edge_beams_cannot_affect_localization():
    from dataclasses import replace
    from robot_validate.scan_matcher import ScanMatcher

    frames, spec, _ = synthetic_frames()
    a, b = ScanMatcher(spec), ScanMatcher(spec)
    for frame in frames[:20]:
        altered = frame.ranges.copy()
        altered[: spec.margin + 1] = 1.0
        altered[spec.size - spec.margin :] = 2.0
        pose_a, _ = a.update(frame)
        pose_b, _ = b.update(replace(frame, ranges=altered))
        np.testing.assert_array_equal(pose_a, pose_b)
        np.testing.assert_array_equal(a.points, b.points)


@pytest.mark.parametrize("invalid", [np.nan, -np.inf])
def test_invalid_scans_do_not_create_free_space(tmp_path, invalid):
    from dataclasses import replace

    frames, spec, _ = synthetic_frames()
    invalid_frames = [replace(f, ranges=np.full(spec.size, invalid)) for f in frames[:10]]
    write_bag(tmp_path / "invalid.bag", invalid_frames, spec)
    restored = read_bag(tmp_path / "invalid.bag", spec)
    assert np.isnan(restored[0].ranges).all()
    _, grid = run_breezy(restored, spec, 1, tmp_path)
    assert np.all(grid.cells == -1)


def test_only_explicit_positive_infinity_is_a_free_ray():
    from robot_validate.slam import native_ranges

    result = native_ranges(np.array([np.nan, -np.inf, np.inf, -1, 0, 12, 13]), 12)
    assert result == [1, 1, 0, 1, 1, 12000, 1]
