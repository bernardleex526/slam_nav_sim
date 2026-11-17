"""Temporary obstruction must exercise recovery, rather than permit a bypass."""

import xml.etree.ElementTree as ET
import numpy as np
from scipy.ndimage import binary_dilation, label

from robot_validate.ros_assets import build_world
from robot_validate.runner import reference_grid
from robot_validate.temporary_block import insertion_ready, StopWindow, recovery_evidence


def test_active_gate_separates_start_and_goal_but_hidden_gate_does_not(tmp_path):
    world = tmp_path / "world.sdf"
    build_world(world)
    block = ET.parse(world).find("./world/model[@name='temporary_block']")
    x, y = map(float, block.findtext("pose").split()[:2])
    sx, sy = map(float, block.findtext("link/collision/geometry/box/size").split()[:2])
    grid = reference_grid()
    grid.origin = (-3, -3, 0)
    rows, cols = np.indices(grid.cells.shape)
    wx = grid.origin[0] + (cols + 0.5) * grid.resolution
    wy = grid.origin[1] + (rows + 0.5) * grid.resolution
    gate = (abs(wx - x) <= sx / 2) & (abs(wy - y) <= sy / 2)
    radius = int(np.ceil(0.26 / grid.resolution))
    dy, dx = np.mgrid[-radius : radius + 1, -radius : radius + 1]
    footprint = (dx * grid.resolution) ** 2 + (dy * grid.resolution) ** 2 <= 0.26**2

    def reachable(active):
        occupied = grid.cells != 0
        if active:
            occupied |= gate
        regions, _ = label(~binary_dilation(occupied, structure=footprint))
        start = (int(3 / grid.resolution), int(3 / grid.resolution))
        finish = (int(3 / grid.resolution), int(11 / grid.resolution))
        return regions[start] != 0 and regions[start] == regions[finish]

    assert reachable(False)
    assert not reachable(True), "Active temporary gate still permits a bypass"


def test_gate_waits_for_approach_and_a_sustained_fresh_odometry_stop():
    assert not insertion_ready([1, 1, 0, 0], 1)
    assert not insertion_ready([1, 2.2, 0, 0], 2)
    assert not insertion_ready([1, 2.5, 0, 0], 1)  # too close to teleport the gate safely
    assert insertion_ready([1, 2.2, 0, 0], 1)
    stop = StopWindow(3)
    for t in np.arange(1, 4, 0.1):
        assert not stop.update(t, 2.3, 0, 0, t)
    assert stop.update(4.01, 2.3, 0, 0, 4.01)
    assert not stop.update(4.01, 2.3, 0, 0, 5)  # a stale repeated sample cannot release
    assert not stop.update(5, 2.3, 0, 0.08, 5)


def episode():
    t = np.arange(0, 8.01, 0.1)
    speed = np.where(t > 5.2, 0.2, 0.0)
    x = 2.3 + np.maximum(0, t - 5.2) * 0.2
    return np.column_stack((t, x, np.zeros((len(t), 2)), speed, np.zeros((len(t), 2))))


def test_independent_evidence_requires_stop_then_removal_then_real_forward_motion():
    data = episode()
    proof = recovery_evidence(data, 1, 5, 5.1)
    assert proof["observed"] and proof["resumed"]
    assert proof["stop_interval_s"][1] <= 5 < 5.1 < proof["resumed_at_s"]
    assert proof["resumed_x"] - proof["stopped_pose"][0] >= 0.1
    data[:, 4] = 0.1  # a moving bypass is not a blocked episode
    assert not recovery_evidence(data, 1, 5, 5.1)["observed"]


def test_brief_stop_stale_truth_and_motion_before_removal_do_not_pass():
    data = episode()
    data[data[:, 0] < 4.5, 4] = 0.2
    assert not recovery_evidence(data, 1, 5, 5.1)["observed"]
    data = episode()
    data = data[(data[:, 0] < 3.5) | (data[:, 0] > 4.5)]
    assert not recovery_evidence(data, 1, 5, 5.1)["observed"]
    data = episode()
    data[data[:, 0] > 5.1, 4] = 0.0
    assert not recovery_evidence(data, 1, 5, 5.1)["resumed"]
    assert not recovery_evidence(episode(), None, 5, 5.1)["observed"]
