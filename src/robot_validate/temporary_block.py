"""Temporary gate scheduling and independent evidence for the bundled recovery case."""

import math
import numpy as np

# Close the entire room at x=3; a shorter obstacle permits a north-side bypass.
GATE_X, GATE_Y = 3.0, 2.5
GATE_SIZE = (0.5, 9.2, 1.5)
STOP_SPEED = 0.05  # Original blocked/resumed speed boundary, in m/s.
MAX_SAMPLE_GAP = 0.5


def insertion_ready(map_pose, now_s):
    return bool(
        map_pose is not None
        and all(math.isfinite(v) for v in map_pose)
        and 0 <= now_s - map_pose[0] <= MAX_SAMPLE_GAP
        and 2.1 <= map_pose[1] <= 2.4
        and abs(map_pose[2]) < 0.5
    )


class StopWindow:
    def __init__(self, duration_s):
        self.duration_s = duration_s
        self.start = self.end = None

    def update(self, stamp_s, x, y, speed, now_s):
        valid = (
            all(math.isfinite(v) for v in (stamp_s, x, y, speed, now_s))
            and 0 <= now_s - stamp_s <= MAX_SAMPLE_GAP
            and abs(x - 3) < 1.0
            and abs(y) < 1.8
            and 0 <= speed < STOP_SPEED
        )
        if not valid or (self.end is not None and stamp_s < self.end):
            self.start = self.end = None
            return False
        if self.end is None or stamp_s - self.end > MAX_SAMPLE_GAP:
            self.start = stamp_s
        self.end = stamp_s
        return self.end - self.start >= self.duration_s


def recovery_evidence(motion, inserted_s, removal_requested_s, removed_s):
    """Use truth only after execution; never use it to schedule or drive the robot."""
    proof = dict(observed=False, resumed=False, stop_interval_s=[], resumed_at_s=None)
    if None in (inserted_s, removal_requested_s, removed_s):
        return proof
    if not (
        all(math.isfinite(t) for t in (inserted_s, removal_requested_s, removed_s))
        and inserted_s < removal_requested_s <= removed_s
    ):
        return proof
    data = np.asarray(motion, dtype=float)
    if data.ndim != 2 or len(data) < 2 or not np.isfinite(data).all():
        return proof
    if np.any(np.diff(data[:, 0]) <= 0):
        return proof
    stopped = StopWindow(2.0)
    before = data[(data[:, 0] >= inserted_s) & (data[:, 0] <= removal_requested_s)]
    for t, x, y, _, vx, vy, _ in before:
        stopped.update(t, x, y, math.hypot(vx, vy), t)
    if stopped.end is None:
        return proof
    proof["stop_interval_s"] = [stopped.start, stopped.end]
    proof["observed"] = bool(
        stopped.end - stopped.start >= stopped.duration_s
        and 0 <= removal_requested_s - stopped.end <= MAX_SAMPLE_GAP
    )
    if not proof["observed"]:
        return proof
    proof["stopped_pose"] = before[-1, 1:3].tolist()
    after = data[data[:, 0] > removed_s]
    crossings = after[after[:, 1] > GATE_X + GATE_SIZE[0] / 2 + 0.26]
    proof["passed_gate_at_s"] = float(crossings[0, 0]) if len(crossings) else None
    previous = removed_s
    for t, x, _, _, vx, _, _ in after:
        if t - previous > MAX_SAMPLE_GAP:
            return proof
        previous = t
        if vx > STOP_SPEED and x - before[-1, 1] >= 0.1:
            proof.update(resumed=True, resumed_at_s=float(t), resumed_x=float(x))
            break
    return proof
