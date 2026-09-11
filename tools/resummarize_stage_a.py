#!/usr/bin/env python3
"""Rebuild Stage A tables from current per-run artifacts."""
import argparse
from pathlib import Path
import sys
import yaml

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src/sim_benchmark/scripts"))
from stage_a_runner import resummarize


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--config", default=str(ROOT / "src/sim_benchmark/config/stage_a_matrix.yaml"))
    parser.add_argument("--output-dir")
    args = parser.parse_args()
    with open(args.config, encoding="utf-8") as stream:
        cfg = yaml.safe_load(stream)
    rows, results = resummarize(args.output_dir or cfg["runner"]["output_dir"], cfg)
    print(f"Reports: {len(rows)}; runs: {len(results)}")
    for row in rows:
        print(row["world"], row["status"], row["map_score"], row["ghost_wall_ratio"])


if __name__ == "__main__":
    main()
