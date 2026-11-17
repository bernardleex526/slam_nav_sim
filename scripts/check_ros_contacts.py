#!/usr/bin/env python3
"""Prove a quiet contact stream can detect a known physical contact in an isolated world."""

import argparse
from pathlib import Path
import re
import sys
import traceback

import rclpy

from robot_validate.model import dump_json, sha256
from robot_validate.process import ProcessScope
from robot_validate.ros_backend import isolated_domain, start_sim
from robot_validate.ros_probe import Probe


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--assets", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    output = args.output.resolve()
    output.mkdir(parents=True, exist_ok=False)
    record = dict(status="fail", world_sha256=sha256(args.assets / "world.sdf"))
    with isolated_domain() as env:
        record["isolation"] = env
        rclpy.init()
        try:
            with ProcessScope(output / "logs") as scope:
                probe = Probe(output)
                try:
                    start_sim(scope, args.assets.resolve(), 9999, "none")
                    probe.ready()
                    probe.wait(
                        lambda: probe.collision_geometries == 4 and probe.contact_samples >= 20,
                        30,
                        "continuous observation of all robot collision geometries",
                    )
                    probe.run_for(1)
                    record.update(
                        baseline_contacts=probe.contacts,
                        observed_geometries=probe.collision_geometries,
                        baseline_samples=probe.contact_samples,
                    )
                    if probe.contacts:
                        raise AssertionError("Baseline contains unexpected non-floor contacts")
                    process = scope.start(
                        "known-contact",
                        [
                            "gz",
                            "service",
                            "-s",
                            "/world/validation/set_pose",
                            "--reqtype",
                            "gz.msgs.Pose",
                            "--reptype",
                            "gz.msgs.Boolean",
                            "--timeout",
                            "5000",
                            "--req",
                            'name: "temporary_block" position {x: 0 y: 0 z: 0.75} orientation {w: 1}',
                        ],
                    )
                    if scope.wait(process, 8) != 0 or not re.search(
                        r"\bdata:\s*true\b", (output / "logs/known-contact.log").read_text()
                    ):
                        raise RuntimeError("Known-contact injection service failed")
                    probe.wait(
                        lambda: any(
                            "temporary_block" in " ".join(e.get("objects", []))
                            for e in probe.events
                        ),
                        15,
                        "physical contact with the injected blocker",
                    )
                    record.update(
                        status="pass",
                        injected_contact_count=probe.contacts,
                        event_evidence="events.json",
                        final_samples=probe.contact_samples,
                    )
                finally:
                    probe.velocity()
                    probe.export()
                    probe.destroy_node()
        except BaseException as error:
            record["status"] = "fail"
            record["error"] = f"{type(error).__name__}: {error}"
            (output / "failure.log").write_text(traceback.format_exc())
        finally:
            rclpy.try_shutdown()
    dump_json(output / "contact-sanity.json", record)
    print(record)
    return 0 if record["status"] == "pass" else 1


if __name__ == "__main__":
    sys.exit(main())
