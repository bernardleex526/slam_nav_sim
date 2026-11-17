import numpy as np
import pytest
from robot_validate.metrics import trajectory_checks, map_checks, sample_grid, navigation_checks
from robot_validate.model import DEFAULT_THRESHOLDS as T
from robot_validate.slam import Grid


def test_truth_missing_and_fused_reference_never_pass():
    poses = np.array([[0, 0, 0, 0], [1, 1, 0, 0], [2, 1, 1, 1]])
    assert trajectory_checks(poses, None, T)[0][0].status == "insufficient"
    assert trajectory_checks(poses, poses, T, independent=False)[0][0].status == "insufficient"


def test_scale_error_is_retained():
    truth = np.array([[0, 0, 0, 0], [1, 2, 0, 0], [2, 2, 2, 1], [3, 0, 2, 2]], float)
    est = truth.copy()
    est[:, 1:3] *= 3
    checks, stats, _ = trajectory_checks(est, truth, T)
    assert checks[0].status == "fail"
    assert stats["scale_correction"] is False
    assert stats["reference_coverage"] == 1


def test_insufficient_pair_coverage():
    truth = np.array([[0, 0, 0, 0], [1, 1, 0, 0], [2, 1, 1, 0], [3, 0, 1, 0]], float)
    assert trajectory_checks(truth[:3], truth, T)[0][0].status == "insufficient"


def test_map_geometry_and_unknown_semantics():
    ref = Grid(np.array([[100, 0], [100, 0]], np.int8), 1, (0, 0, 0))
    unknown = Grid(np.full((2, 2), -1, np.int8), 1, (0, 0, 0))
    checks, scores, _ = map_checks(unknown, ref, T)
    assert scores["false_free_rate"] == 0
    assert scores["coverage"] == 0
    assert checks[0].status == "fail"
    rotated = Grid(np.array([[100, 0]], np.int8), 1, (2, 3, np.pi / 2))
    assert sample_grid(rotated, np.array([[1.5, 3.5], [1.5, 4.5]])).tolist() == [100, 0]


@pytest.mark.parametrize(
    "change, expected",
    [
        ({}, "pass"),
        ({"contacts": 1}, "fail"),
        ({"contact_samples": 0}, "insufficient"),
        ({"final_pose": [2, 2, 0]}, "fail"),
        ({"action_status": "aborted"}, "fail"),
        ({"pose_age_s": 2}, "insufficient"),
    ],
)
def test_navigation_requires_independent_evidence(change, expected):
    record = dict(
        name="healthy",
        action_status="succeeded",
        final_pose=[1, 1, 0],
        goal=[1, 1, 0],
        contacts=0,
        contact_samples=10,
        duration_s=10,
        timeout_s=30,
        final_speed=0,
        pose_age_s=0.1,
        map_sha256="abc",
    )
    record.update(change)
    record.update(
        contact_coverage=1.0,
        contact_max_gap_s=0.02,
        contact_age_s=0.02,
        collision_geometries_observed=4,
    )
    assert navigation_checks(record, T, "navigation")[0].status == expected


@pytest.mark.parametrize(
    "field,value",
    [
        ("contact_coverage", 0.1),
        ("contact_max_gap_s", 2),
        ("contact_age_s", 2),
        ("collision_geometries_observed", 1),
    ],
)
def test_partial_collision_evidence_cannot_pass(field, value):
    record = dict(
        name="healthy",
        action_status="succeeded",
        final_pose=[1, 1, 0],
        goal=[1, 1, 0],
        contacts=0,
        contact_samples=10,
        duration_s=10,
        timeout_s=30,
        final_speed=0,
        pose_age_s=0.1,
        map_sha256="abc",
        contact_coverage=1.0,
        contact_max_gap_s=0.02,
        contact_age_s=0.02,
        collision_geometries_observed=4,
    )
    record[field] = value
    assert navigation_checks(record, T, "navigation")[0].status == "insufficient"
