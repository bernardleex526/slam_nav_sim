from robot_validate.route_control import waypoint_command


def test_map_heading_corrects_the_observed_turn_overshoot():
    v, w, reached = waypoint_command([10, 8, 0, 1.739], [8, 5], 10.02)
    assert v > 0
    assert w < -0.2
    assert not reached


def test_stale_or_missing_map_pose_stops_motion():
    assert waypoint_command([10, 8, 0, 1.739], [8, 5], 11) == (0, 0, False)
    assert waypoint_command(None, [8, 5], 11) == (0, 0, False)


def test_arrival_uses_map_coordinates():
    assert waypoint_command([10, 7.98, 0.01, 0], [8, 0], 10.01) == (0, 0, True)
