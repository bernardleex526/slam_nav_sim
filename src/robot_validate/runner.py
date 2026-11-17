from contextlib import contextmanager
from datetime import datetime, timezone
from importlib.metadata import version
import json
import os
from pathlib import Path
import platform
import signal
import subprocess
import sys
import time
import traceback

import numpy as np

from .data import (
    LaserSpec,
    WALLS,
    BOXES,
    public_frames,
    synthetic_frames,
    write_bag,
    read_bag,
    read_tum,
    write_tum,
)
from .metrics import health_checks, trajectory_checks, map_checks
from .model import Result, Check, EXIT_CODES, dump_json, sha256
from .process import ProcessScope, cleanup_ledger
from .report import write_report, map_figure
from .slam import Grid, run_breezy


def project_root():
    root = Path(os.environ.get("ROBOT_VALIDATE_ROOT", Path(__file__).resolve().parents[2]))
    if not (root / "pyproject.toml").exists():
        raise FileNotFoundError("Set ROBOT_VALIDATE_ROOT to the unpacked plugin source directory")
    return root


def reference_grid():
    resolution = 0.04
    xx, yy = np.meshgrid(
        (np.arange(350) + 0.5) * resolution - 1, (np.arange(275) + 0.5) * resolution - 1
    )
    cells = np.full(xx.shape, -1, np.int8)
    cells[(xx >= 0) & (xx <= 12) & (yy >= 0) & (yy <= 9)] = 0
    for x0, y0, x1, y1 in BOXES[1:]:
        # Only boundary surfaces are observable; solid interiors are outside the scoring mask.
        cells[(xx > x0) & (xx < x1) & (yy > y0) & (yy < y1)] = -1
    for axis, at, lo, hi in WALLS:
        a, b = (xx, yy) if axis == 0 else (yy, xx)
        cells[(abs(a - at) <= 0.06) & (b >= lo) & (b <= hi)] = 100
    return Grid(cells, resolution, (-1, -1, 0))


class Execution:
    def __init__(self, request, output):
        self.output = Path(output)
        self.output.mkdir(parents=True, exist_ok=True)
        self.result = Result(request["scenario"], request["backend"], request["config"]["seed"])
        self.figure = ""
        self.start = time.monotonic()
        self.result.provenance = dict(
            started_utc=datetime.now(timezone.utc).isoformat(),
            platform=platform.platform(),
            python=sys.version,
            config=request["config"],
            fault=request.get("fault", "none"),
            software={p: version(p) for p in ["numpy", "rosbags", "evo", "pytest"]},
        )
        dump_json(self.output / "request.json", request)

    def save(self):
        dump_json(self.output / "checkpoint.json", self.result.as_dict())

    @contextmanager
    def stage(self, name):
        stage = dict(
            name=name, status="running", elapsed_s=0.0, start_wall_s=time.monotonic() - self.start
        )
        self.result.stages.append(stage)
        self.save()
        start = time.monotonic()
        try:
            yield
        except BaseException:
            stage["status"] = "failed"
            raise
        else:
            stage["status"] = "completed"
        finally:
            stage["elapsed_s"] = time.monotonic() - start
            self.save()


def run_local(execution, request):
    result, output = execution.result, execution.output
    cfg, fault = request["config"], request.get("fault", "none")
    reference = ref_map = None
    with execution.stage("input_preflight"):
        if request["scenario"] == "public" or request.get("bag"):
            if request.get("bag"):
                bag = Path(request["bag"]).resolve()
                spec = LaserSpec(**cfg.get("bag", {}).get("laser", {}))
            else:
                source = project_root() / "third_party/breezyslam/examples/exp2.dat"
                expected = (
                    (project_root() / "third_party/breezyslam/exp2.sha256").read_text().split()[0]
                )
                if sha256(source) != expected:
                    raise ValueError("Public dataset SHA256 mismatch")
                frames, spec = public_frames(source)
                bag = output / "input.bag"
                write_bag(bag, frames, spec, missing=range(220, 300) if fault == "scan-gap" else ())
                result.provenance["dataset"] = dict(
                    name="BreezySLAM Mines exp2",
                    sha256=sha256(source),
                    source="https://github.com/simondlevy/BreezySLAM/tree/1170285cd42a88300eddd275fc3df36e642e3a2e/examples",
                    independent_truth=False,
                    note="真实激光日志转为 ROS 1 bag；轮速不是独立真值",
                )
            reference_cfg = cfg.get("reference", {})
            if reference_cfg.get("trajectory"):
                reference = read_tum(reference_cfg["trajectory"])
                result.provenance["reference"] = reference_cfg
            initial = (0, 0, 0)
        else:
            frames, spec, truth = synthetic_frames(wheel_scale=3 if fault == "odometry-bias" else 1)
            bag = output / "input.bag"
            write_bag(bag, frames, spec, missing=range(100, 180) if fault == "scan-gap" else ())
            reference = None if fault == "no-truth" else truth
            ref_map = reference_grid()
            ref_map.save(output, "reference-map")
            initial = (2, 2, 0)
            result.provenance["dataset"] = dict(
                name="deterministic 2D raycast",
                generator_seed=391,
                truth_source="independent geometric raycast simulator",
                independent_truth=reference is not None,
                scope="建图工具本地回归；不是 Gazebo/Nav2 闭环验证",
            )
        if not bag.exists():
            raise FileNotFoundError(bag)
        if bag.is_file():
            result.provenance["input_sha256"] = sha256(bag)
        result.provenance["input_bag"] = str(bag)
        frames = read_bag(
            bag,
            spec,
            cfg.get("bag", {}).get("scan_topic", "/scan"),
            cfg.get("bag", {}).get("wheel_topic", "/wheel_twist"),
        )
    with execution.stage("mapping"):
        matcher = cfg.get("algorithm", {}).get("local_matcher", "point_to_line")
        estimated, generated = run_breezy(frames, spec, cfg["seed"], output, initial, matcher)
        result.provenance["local_algorithm"] = dict(
            pose_estimator=matcher, map_engine="BreezySLAM CoreSLAM", reference_inputs=False
        )
        result.artifacts["mapping_diagnostics"] = "mapping-diagnostics.json"
        result.artifacts["trajectory"] = "estimated.tum"
    with execution.stage("map_export"):
        if fault == "map-export":
            raise RuntimeError(
                "Injected map export failure after SLAM; partial trajectory retained"
            )
        generated.save(output)
        loaded = Grid.load(output / "map.yaml")
        result.provenance["map_sha256"] = sha256(output / "map.pgm")
        result.artifacts.update(map="map.yaml", trajectory="estimated.tum", input=str(bag))
    with execution.stage("evaluation"):
        result.checks.extend(health_checks(estimated, loaded, cfg["thresholds"]))
        independent = (
            cfg.get("reference", {}).get("independent", False)
            if request.get("bag") or request["scenario"] == "public"
            else True
        )
        checks, metrics, _ = trajectory_checks(
            estimated, reference, cfg["thresholds"], independent=independent
        )
        result.checks.extend(checks)
        result.metrics["trajectory"] = metrics
        if reference is not None:
            write_tum(output / "reference.tum", reference)
        if ref_map is not None:
            checks, metrics, _ = map_checks(generated, ref_map, cfg["thresholds"])
            result.checks.extend(checks)
            result.metrics["map"] = metrics
        else:
            result.checks.append(
                Check(
                    "MAP_REFERENCE",
                    "mapping",
                    "insufficient",
                    reason="公开日志没有测绘参考地图",
                    required=False,
                )
            )
        result.checks.extend(
            [
                Check(
                    "LOOP_EVIDENCE",
                    "mapping",
                    "insufficient",
                    reason="BreezySLAM 不提供回环候选／接受事件；不能据最终轨迹推断原因",
                    required=False,
                ),
                Check(
                    "NAV2_RUNTIME",
                    "navigation",
                    "insufficient",
                    reason="本地回归不运行 Nav2；需使用 ROS 2 后端",
                    required=False,
                ),
                Check(
                    "GENERATED_MAP_NAVIGATION",
                    "integration",
                    "insufficient",
                    reason="待 ROS 2 / Gazebo 环境验收",
                    required=False,
                ),
            ]
        )
        execution.figure = map_figure(generated, ref_map, estimated, reference)


def worker(request, output):
    execution = Execution(request, output)
    try:
        if request["backend"] == "local":
            run_local(execution, request)
        else:
            from .ros_backend import run_ros

            run_ros(execution, request)
    except BaseException as error:
        trace = Path(output) / "failure.log"
        trace.write_text(traceback.format_exc())
        stage = execution.result.stages[-1]["name"] if execution.result.stages else "startup"
        state = (
            "environment_error" if isinstance(error, (ImportError, FileNotFoundError)) else "fail"
        )
        execution.result.checks.append(
            Check(
                f"{stage.upper()}_ERROR",
                "mapping"
                if stage in ("mapping", "map_export", "input_preflight")
                else "integration",
                state,
                reason=f"{type(error).__name__}: {error}",
                window_s=[time.monotonic() - execution.start],
                evidence=["failure.log"],
            )
        )
    execution.save()
    write_report(execution.result, output, execution.figure)
    return EXIT_CODES[execution.result.status]


def supervised_run(request, output, timeout=300):
    output = Path(output).resolve()
    if output.exists() and any(output.iterdir()):
        raise ValueError(f"Result directory must be empty: {output}")
    output.mkdir(parents=True, exist_ok=True)
    dump_json(output / "request.json", request)
    started = time.monotonic()
    interrupted = False
    previous = signal.getsignal(signal.SIGTERM)

    def terminate(_sig, _frame):
        raise KeyboardInterrupt()

    signal.signal(signal.SIGTERM, terminate)
    try:
        with ProcessScope(
            output / "logs", {"ROBOT_VALIDATE_PROCESS_LEDGER": str(output / "processes.jsonl")}
        ) as scope:
            process = scope.start(
                "worker",
                [
                    sys.executable,
                    "-m",
                    "robot_validate.worker",
                    str(output / "request.json"),
                    str(output),
                ],
            )
            try:
                code = scope.wait(process, timeout)
            except (subprocess.TimeoutExpired, KeyboardInterrupt) as error:
                interrupted = isinstance(error, KeyboardInterrupt)
                code = 130 if interrupted else 124
    finally:
        cleanup_ledger(output / "processes.jsonl")
        signal.signal(signal.SIGTERM, previous)
    if code in (124, 130) or not (output / "result.json").exists():
        result = Result(request["scenario"], request["backend"], request["config"]["seed"])
        checkpoint = output / "checkpoint.json"
        if checkpoint.exists():
            data = json.loads(checkpoint.read_text())
            for key in ("provenance", "stages", "artifacts", "metrics"):
                setattr(result, key, data[key])
            result.checks = [Check(**c) for c in data["checks"]]
        name = "INTERRUPTED" if interrupted else ("TIMEOUT" if code == 124 else "WORKER_CRASH")
        result.checks.append(
            Check(
                name,
                "integration",
                "environment_error",
                code,
                timeout,
                f"运行器终止：{name}；已清理本轮进程组",
                [time.monotonic() - started],
                ["logs/worker.log"],
            )
        )
        code = code if code in (124, 130) else 3
        result.process_exit_code = code
        write_report(result, output)
    else:
        # Parent logs may contain the last worker messages; retain the worker's full figure.
        code = json.loads((output / "result.json").read_text())["exit_code"]
    return code, output / "report.html"
