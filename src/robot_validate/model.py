from dataclasses import asdict, dataclass, field
from pathlib import Path
import hashlib
import json
import math

import yaml

STATES = ("pass", "insufficient", "fail", "environment_error")
EXIT_CODES = {"pass": 0, "fail": 1, "insufficient": 2, "environment_error": 3}


@dataclass
class Check:
    id: str
    section: str
    status: str
    value: object = None
    threshold: object = None
    reason: str = ""
    window_s: list = field(default_factory=list)
    evidence: list = field(default_factory=list)
    required: bool = True

    def __post_init__(self):
        if self.status not in STATES:
            raise ValueError(f"Invalid status: {self.status}")


def aggregate(checks):
    required = [c.status for c in checks if c.required]
    return max(required, key=STATES.index) if required else "insufficient"


def sha256(path):
    digest = hashlib.sha256()
    with Path(path).open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def dump_json(path, value):
    Path(path).parent.mkdir(parents=True, exist_ok=True)
    Path(path).write_text(json.dumps(value, ensure_ascii=False, indent=2, allow_nan=False) + "\n")


@dataclass
class Result:
    scenario: str
    backend: str
    seed: int
    checks: list = field(default_factory=list)
    provenance: dict = field(default_factory=dict)
    stages: list = field(default_factory=list)
    artifacts: dict = field(default_factory=dict)
    metrics: dict = field(default_factory=dict)
    process_exit_code: object = None

    @property
    def status(self):
        return aggregate(self.checks)

    def as_dict(self):
        data = asdict(self)
        data.update(
            schema_version=1,
            status=self.status,
            exit_code=self.process_exit_code
            if self.process_exit_code is not None
            else EXIT_CODES[self.status],
        )
        data["sections"] = {
            s: aggregate([c for c in self.checks if c.section == s])
            for s in ("mapping", "navigation", "integration")
        }
        return data


DEFAULT_THRESHOLDS = {
    "max_scan_gap_s": 0.5,
    "min_known_cells": 100,
    "ate_rmse_m": 0.75,
    "min_pair_coverage": 0.95,
    "max_pair_dt_s": 0.03,
    "map_coverage": 0.70,
    "false_free_rate": 0.10,
    "wall_p95_m": 0.5,
    "position_m": 0.25,
    "yaw_rad": 0.25,
    "stop_speed_mps": 0.02,
}


def load_config(path=None):
    cfg = yaml.safe_load(Path(path).read_text()) if path else {}
    cfg = cfg or {}
    if not isinstance(cfg, dict):
        raise ValueError("Configuration must be a mapping")
    allowed = {"name", "seed", "thresholds", "bag", "ros", "reference", "algorithm"}
    if set(cfg) - allowed:
        raise ValueError(f"Unknown configuration fields: {sorted(set(cfg) - allowed)}")
    thresholds = dict(DEFAULT_THRESHOLDS)
    overrides = cfg.get("thresholds", {})
    if set(overrides) - set(thresholds):
        raise ValueError(f"Unknown thresholds: {sorted(set(overrides) - set(thresholds))}")
    thresholds.update(overrides)
    for key, value in thresholds.items():
        if not isinstance(value, (int, float)) or not math.isfinite(value) or value < 0:
            raise ValueError(f"Invalid threshold {key}: {value}")
        if key in ("min_pair_coverage", "map_coverage", "false_free_rate") and value > 1:
            raise ValueError(f"Threshold {key} must be a fraction")
    cfg["thresholds"] = thresholds
    cfg.setdefault("seed", 9999)
    if not isinstance(cfg["seed"], int) or not 1 <= cfg["seed"] <= 65535:
        raise ValueError("seed must be an integer in 1..65535 (BreezySLAM treats 0 as random)")
    return cfg
