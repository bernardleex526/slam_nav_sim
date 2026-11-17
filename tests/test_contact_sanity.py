"""The contact diagnostic must not report success if its evidence cannot be saved."""

import importlib.util
import json
from pathlib import Path
import sys
from types import SimpleNamespace

import pytest


@pytest.mark.parametrize("failure", ["export", "cleanup", None])
def test_contact_sanity_fails_closed_after_positive_observation(tmp_path, monkeypatch, failure):
    class Probe:
        contacts = 0
        collision_geometries = 4
        contact_samples = 25
        events = []

        def __init__(self, output):
            self.output = output

        def ready(self):
            pass

        def run_for(self, seconds):
            pass

        def wait(self, condition, timeout, description):
            if "physical contact" in description:
                self.contacts = 80
                self.events = [{"objects": ["temporary_block", "robot"]}]
            assert condition()

        def velocity(self):
            pass

        def export(self):
            if failure == "export":
                raise OSError("evidence disk failure")
            (self.output / "events.json").write_text(json.dumps(self.events))

        def destroy_node(self):
            pass

    class Scope:
        def __init__(self, directory):
            self.directory = directory
            directory.mkdir()

        def __enter__(self):
            return self

        def __exit__(self, *args):
            if failure == "cleanup":
                raise RuntimeError("cleanup failure")

        def start(self, name, argv):
            (self.directory / f"{name}.log").write_text("data: true\n")

        def wait(self, process, timeout):
            return 0

    monkeypatch.setitem(
        sys.modules, "rclpy", SimpleNamespace(init=lambda: None, try_shutdown=lambda: None)
    )
    monkeypatch.setitem(sys.modules, "robot_validate.ros_probe", SimpleNamespace(Probe=Probe))
    path = Path(__file__).resolve().parents[1] / "scripts/check_ros_contacts.py"
    spec = importlib.util.spec_from_file_location("contact_sanity_test_subject", path)
    diagnostic = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(diagnostic)
    monkeypatch.setattr(diagnostic, "ProcessScope", Scope)
    monkeypatch.setattr(diagnostic, "start_sim", lambda *args: None)
    assets = tmp_path / "assets"
    assets.mkdir()
    (assets / "world.sdf").write_text("test world")
    output = tmp_path / "result"
    monkeypatch.setattr(sys, "argv", [str(path), "--assets", str(assets), "--output", str(output)])
    assert diagnostic.main() == (1 if failure else 0)
    record = json.loads((output / "contact-sanity.json").read_text())
    assert record["status"] == ("fail" if failure else "pass")
    if failure:
        assert "error" in record
