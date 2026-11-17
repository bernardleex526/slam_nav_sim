from contextlib import contextmanager
import os
import re
from pathlib import Path
import shutil
import subprocess
import sys
import tempfile
import uuid

import numpy as np
import yaml
from PIL import Image

from .data import public_frames, write_bag, write_tum, read_tum
from .metrics import health_checks, trajectory_checks, map_checks, navigation_checks
from .model import Check, sha256, dump_json
from .process import ProcessScope
from .report import map_figure
from .ros_assets import build_world, bridge_config, nav_config, slam_config
from .runner import project_root, reference_grid
from .slam import Grid
from .evaluation_inputs import sample_reference, timer_phase
from .temporary_block import GATE_X, GATE_Y


@contextmanager
def isolated_domain():
    import fcntl

    locks = []
    chosen = None
    for domain in range(40, 100):
        lock = open(Path(tempfile.gettempdir()) / f"robot-validate-domain-{domain}.lock", "a")
        try:
            fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
        except BlockingIOError:
            lock.close()
            continue
        chosen = domain
        locks.append(lock)
        break
    if chosen is None:
        raise RuntimeError("No free validation ROS domain")
    env = dict(
        ROS_DOMAIN_ID=str(chosen),
        ROS_LOCALHOST_ONLY="1",
        GZ_PARTITION=f"validation-{uuid.uuid4().hex}",
    )
    previous = {key: os.environ.get(key) for key in env}
    os.environ.update(env)
    try:
        yield env
    finally:
        for key, value in previous.items():
            if value is None:
                os.environ.pop(key, None)
            else:
                os.environ[key] = value
        for lock in locks:
            lock.close()


def normalized(grid):
    cells = np.full(grid.cells.shape, -1, np.int8)
    cells[(grid.cells >= 0) & (grid.cells <= 19)] = 0
    cells[grid.cells >= 65] = 100
    return Grid(cells, grid.resolution, grid.origin)


def same_map(actual, expected):
    return (
        actual.cells.shape == expected.cells.shape
        and np.array_equal(actual.cells, expected.cells)
        and np.allclose(
            [actual.resolution, *actual.origin], [expected.resolution, *expected.origin], atol=1e-6
        )
    )


def reuse_generated_map(source, output):
    """Copy map semantics without evaluating or changing the source algorithm result."""
    source, output = Path(source).resolve(), Path(output)
    if source.parent == output.resolve():
        raise ValueError("Reused map must be copied into a separate result directory")
    metadata = yaml.safe_load(source.read_text())
    source_image = source.parent / metadata["image"]
    if metadata.get("mode", "trinary") != "trinary":
        raise ValueError("Reused maps require trinary mode; raw/scale semantics are unsupported")
    with Image.open(source_image) as image:
        if source_image.suffix.lower() != ".pgm" or image.format != "PPM" or image.mode != "L":
            raise ValueError("Reused maps require an 8-bit grayscale PGM image")
    grid = Grid.load(source)
    grid.save(output)
    if not same_map(Grid.load(output / "map.yaml"), grid):
        raise RuntimeError("Reused map changed geometry or occupancy")
    return dict(
        source_yaml=str(source),
        source_yaml_sha256=sha256(source),
        source_image_sha256=sha256(source_image),
        copied_image_sha256=sha256(output / "map.pgm"),
        geometry_and_cells_identical=True,
        mapping_executed_this_run=False,
    )


def run_ros(execution, request):
    result, output, cfg = execution.result, execution.output, request["config"]
    with execution.stage("environment_preflight"):
        missing = [name for name in ("ros2", "gz") if not shutil.which(name)]
        if missing:
            raise FileNotFoundError(
                f"缺少 {', '.join(missing)}；在 Ubuntu 24.04 / ROS 2 Jazzy 容器中运行 deploy/run.sh"
            )
        if os.environ.get("ROS_DISTRO") != "jazzy":
            raise RuntimeError("This deployment baseline requires ROS_DISTRO=jazzy")
        import rclpy
        from rclpy.utilities import get_rmw_implementation_identifier
        from ament_index_python.packages import get_package_share_directory
        from .ros_probe import Probe

        for package in ("nav2_bringup", "slam_toolbox", "ros_gz_bridge", "nav2_map_server"):
            get_package_share_directory(package)
        build_world(output / "world.sdf")
        bridge_config(output / "bridge.yaml")
        bridge = yaml.safe_load((output / "bridge.yaml").read_text())
        bridge = [entry for entry in bridge if entry["topic_name"] != "/tf"]
        for entry in bridge:
            if entry["topic_name"] in ("/scan", "/odom"):
                topic = entry.pop("topic_name")
                entry.update(gz_topic_name=topic, ros_topic_name=f"{topic}_raw")
        (output / "bridge.yaml").write_text(yaml.safe_dump(bridge))
        nav_config(
            Path(get_package_share_directory("nav2_bringup")) / "params/nav2_params.yaml",
            output / "nav.yaml",
        )
        slam_config(output / "slam.yaml", cfg.get("ros", {}).get("loop_closing", True))
        result.provenance.update(
            world_sha256=sha256(output / "world.sdf"),
            nav_params_sha256=sha256(output / "nav.yaml"),
            slam_params_sha256=sha256(output / "slam.yaml"),
            ROS_DISTRO="jazzy",
            RMW_IMPLEMENTATION=get_rmw_implementation_identifier(),
            validation_level="unverified until this execution completes",
        )
        packages = subprocess.run(
            ["dpkg-query", "-W", "ros-jazzy-*", "gz-*"], capture_output=True, text=True, timeout=20
        )
        (output / "ros-package-versions.log").write_text(packages.stdout + packages.stderr)

    with isolated_domain() as env:
        result.provenance["isolation"] = env
        rclpy.init()
        try:
            if request.get("mode") == "bag":
                run_ros_bag(execution, request, Probe)
            else:
                run_ros_sim(execution, request, Probe)
        finally:
            rclpy.try_shutdown()


def start_sim(scope, output, seed, fault):
    scope.start(
        "gazebo",
        [
            "gz",
            "sim",
            "-s",
            "-r",
            "--headless-rendering",
            "--seed",
            str(seed),
            str(output / "world.sdf"),
        ],
    )
    scope.start(
        "bridge",
        [
            "ros2",
            "run",
            "ros_gz_bridge",
            "parameter_bridge",
            "--ros-args",
            "-p",
            f"config_file:={output / 'bridge.yaml'}",
        ],
    )
    scope.start("sensors", [sys.executable, "-m", "robot_validate.ros_io", "sim", "--fault", fault])
    scope.start(
        "laser-tf",
        [
            "ros2",
            "run",
            "tf2_ros",
            "static_transform_publisher",
            "--z",
            "0.16",
            "--frame-id",
            "base_link",
            "--child-frame-id",
            "laser",
        ],
    )


def start_slam(scope, output, cfg):
    # argv is explicit, no shell. User algorithms may override only the algorithm launch.
    argv = cfg.get("algorithm", {}).get("launch_argv") or [
        "ros2",
        "launch",
        "slam_toolbox",
        "online_sync_launch.py",
        "use_sim_time:=true",
        f"slam_params_file:={output / 'slam.yaml'}",
    ]
    if not isinstance(argv, list) or not all(isinstance(s, str) for s in argv):
        raise ValueError("algorithm.launch_argv must be a list of strings")
    scope.start("algorithm", argv)


def assess_mapping(execution, probe, cfg, reference=None, ref_map=None):
    result, output = execution.result, execution.output
    estimates = np.asarray(probe.estimates)
    if len(estimates) < 3:
        raise RuntimeError("No usable map→base_link algorithm trajectory")
    generated = Grid.load(output / "map.yaml")
    result.checks.extend(health_checks(estimates, generated, cfg["thresholds"]))
    if len(probe.scan_times) >= 2:
        gaps = np.diff(probe.scan_times)
        idx = int(np.argmax(gaps))
        result.checks.append(
            Check(
                "INPUT_SCAN_GAP",
                "mapping",
                "pass" if gaps[idx] <= cfg["thresholds"]["max_scan_gap_s"] else "fail",
                float(gaps[idx]),
                cfg["thresholds"]["max_scan_gap_s"],
                window_s=probe.scan_times[idx : idx + 2],
                evidence=["scan-times.json"],
            )
        )
    checks, stats, _ = trajectory_checks(
        estimates, reference, cfg["thresholds"], independent=reference is not None
    )
    result.checks.extend(checks)
    result.metrics["trajectory"] = stats
    if reference is not None:
        write_tum(output / "reference.tum", reference)
    if ref_map is not None:
        checks, stats, _ = map_checks(generated, ref_map, cfg["thresholds"])
        result.checks.extend(checks)
        result.metrics["map"] = stats
    result.checks.append(
        Check(
            "LOOP_EVIDENCE",
            "mapping",
            "insufficient",
            value={"configured": cfg.get("ros", {}).get("loop_closing", True)},
            reason="已记录回环开关；候选、拒绝、接受事件尚无适配，不能归因",
            required=False,
        )
    )
    result.artifacts.update(map="map.yaml", estimated="estimated.tum")
    result.provenance["map_sha256"] = sha256(output / "map.pgm")
    execution.figure = map_figure(generated, ref_map, estimates, reference)


def run_ros_sim(execution, request, Probe):
    result, output, cfg = execution.result, execution.output, request["config"]
    reference = reference_grid()
    reference.origin = (-3, -3, 0)
    reference.save(output, "reference-map")
    saved_map = request.get("generated_map")
    if saved_map and not request.get("navigation_only"):
        raise ValueError("generated_map requires navigation_only")
    if not request.get("navigation_only"):
        with execution.stage("mapping"):
            with ProcessScope(output / "logs/mapping") as scope:
                probe = Probe(output)
                try:
                    start_sim(scope, output, cfg["seed"], request["fault"])
                    start_slam(scope, output, cfg)
                    probe.ready(mapping=True)
                    probe.wait(
                        lambda: probe.truth is not None, 30, "independent simulator ground truth"
                    )
                    p = probe.truth.pose.pose.position
                    if abs(p.x) > 0.05 or abs(p.y) > 0.05:
                        raise RuntimeError("Simulator initial pose does not match world/map origin")
                    probe.drive_mapping_route()
                    probe.run_for(cfg.get("ros", {}).get("settle_s", 3))
                    probe.export()
                    with execution.stage("map_export"):
                        if request["fault"] == "map-export":
                            raise RuntimeError("Injected map export failure")
                        normalized(probe.grid()).save(output)
                    # Re-sample independent high-rate truth at a fixed 10 Hz, never at algorithm output timestamps.
                    truth = np.asarray(probe.truths)
                    ref, timing = sample_reference(
                        truth,
                        timer_phase(np.asarray(probe.estimates)[:, 0]),
                        cfg["thresholds"]["max_pair_dt_s"],
                    )
                    result.metrics["reference_sampling"] = timing
                    result.checks.append(
                        Check(
                            "REFERENCE_AVAILABILITY",
                            "mapping",
                            "pass"
                            if timing["reference_availability"]
                            >= cfg["thresholds"]["min_pair_coverage"]
                            else "insufficient",
                            timing["reference_availability"],
                            cfg["thresholds"]["min_pair_coverage"],
                            "保留完整参考时段；仅调整取样网格相位，不移动参考时钟或拟合位姿",
                            evidence=["ground-truth-raw.tum"],
                        )
                    )
                    if request["fault"] == "no-truth" or len(ref) < 3:
                        ref = None
                    assess_mapping(execution, probe, cfg, ref, reference)
                finally:
                    probe.velocity()
                    probe.export()
                    probe.destroy_node()
    else:
        result.checks.append(
            Check(
                "MAPPING_NOT_REQUESTED",
                "mapping",
                "insufficient",
                reason="本轮仅导航；不重新执行或重新判定历史建图",
                required=False,
            )
        )
        if saved_map:
            with execution.stage("reuse_generated_map"):
                result.provenance["reused_map"] = reuse_generated_map(saved_map, output)
                result.provenance["map_sha256"] = sha256(output / "map.pgm")
                result.artifacts["map"] = "map.yaml"
                execution.figure = map_figure(Grid.load(output / "map.yaml"), reference)
        else:
            result.checks.append(
                Check(
                    "INTEGRATION_NOT_REQUESTED",
                    "integration",
                    "insufficient",
                    reason="独立参考地图导航模式",
                    required=False,
                )
            )
    generated_path = output / "map.yaml"
    scenarios = (
        ["healthy", "temporary-block", "unreachable", "narrow"]
        if request["scenario"] == "demo"
        else [request["scenario"]]
    )
    navigation_records = []
    comparisons = [("reference", output / "reference-map.yaml", "navigation")]
    if not request.get("navigation_only") or saved_map:
        comparisons.append(("generated", generated_path, "integration"))
    for label, map_path, section in comparisons:
        for scenario in scenarios:
            with execution.stage(f"navigation_{label}_{scenario}"):
                phase = output / label / scenario
                expected = Grid.load(map_path)
                with ProcessScope(phase / "logs") as scope:
                    probe = Probe(phase)
                    try:
                        # Fresh world + identical seed/start/parameters for every comparison.
                        start_sim(scope, output, cfg["seed"], "none")
                        scope.start(
                            "nav2",
                            [
                                "ros2",
                                "launch",
                                "nav2_bringup",
                                "bringup_launch.py",
                                "use_sim_time:=True",
                                "use_composition:=False",
                                "autostart:=True",
                                "slam:=False",
                                f"map:={map_path}",
                                f"params_file:={output / 'nav.yaml'}",
                            ],
                        )
                        probe.ready()
                        probe.wait(lambda: probe.map is not None, 60, "loaded navigation map")
                        probe.wait(
                            lambda: same_map(normalized(probe.grid()), expected),
                            30,
                            "map_server content equals requested map",
                        )
                        probe.wait(lambda: probe.truth is not None, 30, "reset truth")
                        initial = probe.truth.pose.pose.position
                        if abs(initial.x) > 0.05 or abs(initial.y) > 0.05:
                            raise RuntimeError("World reset mismatch")
                        probe.initialize_localization()

                        def block(enabled):
                            command = [
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
                                f'name: "temporary_block" position {{x: {GATE_X} y: {GATE_Y} z: {0.75 if enabled else -3}}} orientation {{w: 1}}',
                            ]
                            child = scope.start(f"block-{enabled}", command)
                            returncode = scope.wait(child, 8)
                            reply = (phase / "logs" / f"block-{enabled}.log").read_text(
                                errors="replace"
                            )
                            if returncode != 0 or not re.search(r"\bdata:\s*true\b", reply):
                                raise RuntimeError("Gazebo blocker pose service failed")

                        record = probe.navigate(
                            scenario, sha256(map_path.with_suffix(".pgm")), block
                        )
                        record.update(
                            map_source=label,
                            map_yaml=str(map_path),
                            world_sha256=sha256(output / "world.sdf"),
                            initial_pose=[initial.x, initial.y, 0],
                            seed=cfg["seed"],
                            nav_params_sha256=sha256(output / "nav.yaml"),
                        )
                        record["evidence"] = f"{label}/{scenario}/navigation.json"
                        dump_json(phase / "navigation.json", record)
                        checks = navigation_checks(record, cfg["thresholds"], section)
                        for check in checks:
                            check.id = f"{label.upper()}_{scenario.upper()}"
                        result.checks.extend(checks)
                        if scenario == "narrow":
                            result.checks.append(
                                Check(
                                    f"{label.upper()}_NARROW_COVERAGE",
                                    section,
                                    "pass" if record["corridor_crossed"] else "fail",
                                    record["corridor_crossed"],
                                    True,
                                    "独立实际轨迹须连续穿过两箱体之间的 0.8 米通道",
                                    record["window_s"],
                                    [f"{label}/{scenario}/ground-truth-raw.tum"],
                                )
                            )
                        navigation_records.append(record)
                        dump_json(output / "navigation-comparison.json", navigation_records)
                        execution.save()
                    finally:
                        probe.velocity()
                        probe.export()
                        probe.destroy_node()
    result.provenance["validation_level"] = "executed ROS 2 / Gazebo; see individual checks"


def run_ros_bag(execution, request, Probe):
    result, output, cfg = execution.result, execution.output, request["config"]
    with execution.stage("input_preflight"):
        if request.get("bag"):
            bag = Path(request["bag"])
        else:
            frames, spec = public_frames(
                project_root() / "third_party/breezyslam/examples/exp2.dat"
            )
            bag = output / "bag2"
            write_bag(
                bag,
                frames,
                spec,
                missing=range(220, 300) if request["fault"] == "scan-gap" else (),
                ros2=True,
            )
        if not (bag / "metadata.yaml").exists():
            raise ValueError("ROS backend requires a rosbag2 directory with metadata.yaml")
        result.provenance["input_bag"] = str(bag)
        result.provenance["bag_metadata_sha256"] = sha256(bag / "metadata.yaml")
    with ProcessScope(output / "logs/replay") as scope:
        probe = Probe(output)
        try:
            scope.start(
                "wheel-adapter",
                [sys.executable, "-m", "robot_validate.ros_io", "bag", "--fault", request["fault"]],
            )
            offset = cfg.get("bag", {}).get("laser", {}).get("offset", 0.145)
            scope.start(
                "laser-tf",
                [
                    "ros2",
                    "run",
                    "tf2_ros",
                    "static_transform_publisher",
                    "--x",
                    str(offset),
                    "--frame-id",
                    "base_link",
                    "--child-frame-id",
                    "laser",
                ],
            )
            start_slam(scope, output, cfg)
            with execution.stage("subscriber_readiness"):
                probe.wait(
                    lambda: probe.count_subscribers("/scan") >= 2
                    and probe.count_subscribers("/wheel_twist") >= 1,
                    60,
                    "algorithm scan subscription and wheel adapter before playback",
                )
            with execution.stage("replay"):
                player = scope.start(
                    "rosbag2",
                    [
                        "ros2",
                        "bag",
                        "play",
                        str(bag),
                        "--clock",
                        "100",
                        "--topics",
                        "/scan",
                        "/wheel_twist",
                    ],
                )
                probe.wait(
                    lambda: player.poll() is not None,
                    cfg.get("ros", {}).get("replay_timeout_s", 600),
                    "bag playback completion",
                )
                if player.returncode != 0:
                    raise RuntimeError(f"rosbag2 exited {player.returncode}")
                probe.run_for(cfg.get("ros", {}).get("settle_s", 3))
                probe.export()
            with execution.stage("map_export"):
                if request["fault"] == "map-export":
                    raise RuntimeError("Injected export failure")
                if probe.map is None:
                    raise RuntimeError("Algorithm did not publish a map")
                normalized(probe.grid()).save(output)
            with execution.stage("evaluation"):
                ref_cfg = cfg.get("reference", {})
                reference = (
                    read_tum(ref_cfg["trajectory"])
                    if ref_cfg.get("trajectory") and ref_cfg.get("independent")
                    else None
                )
                assess_mapping(execution, probe, cfg, reference)
                result.checks.append(
                    Check(
                        "BAG_NAVIGATION",
                        "navigation",
                        "insufficient",
                        reason="录包回放不验收新导航决策",
                        required=False,
                    )
                )
        finally:
            probe.export()
            probe.destroy_node()
