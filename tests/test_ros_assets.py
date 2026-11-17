import xml.etree.ElementTree as ET
import yaml
import numpy as np
from robot_validate.ros_assets import build_world, bridge_config, slam_config
from robot_validate.ros_backend import same_map
from robot_validate.slam import Grid


def test_generated_world_has_independent_truth_and_contacts(tmp_path):
    path = tmp_path / "world.sdf"
    build_world(path)
    world = ET.parse(path).getroot().find("world")
    robot = world.find("model[@name='robot']")
    plugins = {p.get("name"): p for p in robot.findall("plugin")}
    truth = plugins["gz::sim::systems::OdometryPublisher"]
    assert truth.findtext("odom_topic") == "/ground_truth"
    assert truth.findtext("tf_topic") != "/tf"
    assert world.find("plugin[@name='robot_validate::ContactsMonitor']") is not None
    assert len(robot.findall("link/collision")) == 4
    assert world.find("model[@name='temporary_block']") is not None
    assert not world.findall(".//uri")  # world is independent of remote model downloads


def test_bridge_direction_and_loop_switch(tmp_path):
    bridge_config(tmp_path / "bridge.yaml")
    bridge = yaml.safe_load((tmp_path / "bridge.yaml").read_text())
    assert next(b for b in bridge if b["topic_name"] == "/ground_truth")["direction"] == "GZ_TO_ROS"
    slam_config(tmp_path / "slam.yaml", False)
    assert (
        yaml.safe_load((tmp_path / "slam.yaml").read_text())["slam_toolbox"]["ros__parameters"][
            "do_loop_closing"
        ]
        is False
    )


def test_map_receipt_requires_geometry_and_content():
    expected = Grid(np.array([[0, 100]], np.int8), 0.05, (0, 0, 0))
    wrong = Grid(expected.cells.copy(), 0.05, (0.1, 0, 0))
    assert not same_map(wrong, expected)
    wrong.origin = (0, 0, 0)
    assert same_map(wrong, expected)
    wrong.cells[0, 1] = 0
    assert not same_map(wrong, expected)
