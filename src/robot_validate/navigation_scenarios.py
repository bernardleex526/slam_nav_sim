"""Fixed waypoints and independent corridor coverage for the bundled world."""

import numpy as np

# Inner wall faces between the two fixed boxes, after the world's (-2, -2) shift.
NARROW_CORRIDOR = dict(x_min=5.15, x_max=5.95, y_entry=2.0, y_exit=3.2)


def navigation_waypoints(scenario, goal):
    if scenario == "narrow":
        # A final goal alone permits Nav2 to bypass the corridor on its right.
        return [[5.55, 1.6, 1.57], [5.55, 3.6, 1.57], goal]
    return [goal]


def crossed_narrow_corridor(truth, begin, end):
    """Require a continuous south-to-north crossing, not just touching the region."""
    data = np.asarray(truth)
    if data.ndim != 2 or len(data) < 2:
        return False
    data = data[(data[:, 0] >= begin) & (data[:, 0] <= end)]
    entered = False
    c = NARROW_CORRIDOR
    for previous, current in zip(data, data[1:]):
        if current[0] - previous[0] > 0.5:
            entered = False
            continue
        # Require the whole sampled segment to stay between the wall faces.
        inside_x = all(c["x_min"] < p[1] < c["x_max"] for p in (previous, current))
        if not inside_x or current[2] < c["y_entry"]:
            entered = False
        if inside_x and previous[2] <= c["y_entry"] < current[2]:
            entered = True
        if entered and previous[2] < c["y_exit"] <= current[2]:
            return True
    return False
