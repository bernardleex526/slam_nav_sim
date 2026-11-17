import numpy as np
from robot_validate.evaluation_inputs import sample_reference, timer_phase
from robot_validate.metrics import trajectory_checks
from robot_validate.model import DEFAULT_THRESHOLDS


def truth():
    t = np.arange(0.02, 10.01, 0.02)
    return np.column_stack((t, np.sin(t), np.cos(t), np.zeros(len(t))))


def test_reference_phase_matches_timer_without_relaxing_pair_tolerance():
    raw = truth()
    ref, metadata = sample_reference(raw, 0.68, 0.03)
    estimated = ref[ref[:, 0] >= 0.68].copy()
    checks, stats, _ = trajectory_checks(estimated, ref, DEFAULT_THRESHOLDS)
    assert checks[0].status == "pass"
    assert stats["reference_coverage"] >= 0.95
    assert metadata["clock_offset_applied_s"] == 0
    np.testing.assert_allclose(ref[:, 1], np.interp(ref[:, 0], raw[:, 0], raw[:, 1]))


def test_late_estimator_does_not_crop_reference_coverage():
    ref, _ = sample_reference(truth(), 3.68, 0.03)
    estimated = ref[ref[:, 0] >= 3.68].copy()
    assert ref[0, 0] < 0.3
    assert trajectory_checks(estimated, ref, DEFAULT_THRESHOLDS)[0][0].status == "insufficient"


def test_reference_dropout_is_not_interpolated_away():
    raw = truth()
    raw = raw[(raw[:, 0] < 1) | (raw[:, 0] > 3)]
    ref, metadata = sample_reference(raw, 0.68, 0.03)
    assert metadata["reference_availability"] < 0.95
    assert not np.any((ref[:, 0] > 1) & (ref[:, 0] < 3))


def test_timer_phase_does_not_depend_on_first_startup_sample():
    timestamps = np.r_[0.64, np.arange(0.78, 10, 0.1)]
    phase = timer_phase(timestamps)
    assert abs(((phase - 0.08 + 0.05) % 0.1) - 0.05) < 0.001
