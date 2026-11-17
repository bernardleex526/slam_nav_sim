import numpy as np

from robot_validate.navigation_scenarios import crossed_narrow_corridor


def trajectory(x=5.55):
    t = np.arange(100) * 0.1
    return np.column_stack((t, np.full(len(t), x), 1.5 + t * 0.25, np.zeros(len(t))))


def test_goal_reached_by_detour_is_not_narrow_coverage():
    assert crossed_narrow_corridor(trajectory(), 0, 10)
    assert not crossed_narrow_corridor(trajectory(x=7.2), 0, 10)


def test_partial_visit_or_truth_dropout_cannot_claim_crossing():
    data = trajectory()
    assert not crossed_narrow_corridor(data, 0, 4)
    assert not crossed_narrow_corridor(data, 3, 10)
    assert not crossed_narrow_corridor(data[(data[:, 0] < 4) | (data[:, 0] > 6)], 0, 10)


def test_leaving_corridor_side_invalidates_crossing():
    data = trajectory()
    data[40:50, 1] = 6.2
    assert not crossed_narrow_corridor(data, 0, 10)
