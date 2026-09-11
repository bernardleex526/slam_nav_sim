#!/usr/bin/env python3
"""Bounded real-Gazebo navigation smoke; separate artifacts from benchmarks."""
import argparse
import json
from pathlib import Path
import sys
import yaml

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src/sim_benchmark/scripts"))
import stage_b_runner as b


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--all-controllers", action="store_true")
    args = parser.parse_args()
    cfg = yaml.safe_load((ROOT / "src/sim_benchmark/config/stage_b_matrix.yaml").read_text())
    selected = yaml.safe_load(Path(cfg["runner"]["stage_a_selected"]).expanduser().read_text())["selected"]
    cfg["runner"].update(goals_per_run=1, goal_timeout=100, cooldown=2,
                         startup_timeout=100, output_dir="~/.ros/sim_review_20260911")
    experiments = [e for e in cfg["experiments"] if e["world"] == "simple"]
    if not args.all_controllers:
        experiments = experiments[:1]
    results = []
    for index, exp in enumerate(experiments):
        exp = dict(exp, name="review_" + exp["name"])
        result = b.run_one(exp, cfg, index, len(experiments), selected["simple"]["map_yaml"])
        results.append(result)
    target = Path(cfg["runner"]["output_dir"]).expanduser() / "smoke_results.json"
    target.write_text(json.dumps(results, indent=2))
    print(target)
    return 0 if all(r and r.get("goal_statuses") == ["SUCCEEDED"] for r in results) else 1


if __name__ == "__main__":
    sys.exit(main())
