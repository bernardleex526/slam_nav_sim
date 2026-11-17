import argparse
from datetime import datetime
import importlib.util
import json
import os
from pathlib import Path
import shutil
import sys
import webbrowser

from .data import public_frames, write_bag
from .model import load_config
from .runner import project_root, supervised_run


def main(argv=None):
    parser = argparse.ArgumentParser(description="SLAM / 导航证据验证工具")
    commands = parser.add_subparsers(dest="command", required=True)
    commands.add_parser("doctor", help="检查 ROS / Gazebo 和本地依赖")
    prepare = commands.add_parser("prepare-public", help="把随附公开日志转为 ROS 2 SQLite bag")
    prepare.add_argument("--output", type=Path, required=True)
    for name in ("sim", "bag"):
        p = commands.add_parser(name)
        p.add_argument(
            "--backend", choices=("ros2", "local"), default="ros2" if name == "sim" else "local"
        )
        p.add_argument("--scenario", default="demo" if name == "sim" else "public")
        p.add_argument("--config", type=Path)
        p.add_argument("--bag", type=Path)
        p.add_argument("--output", type=Path)
        p.add_argument("--seed", type=int)
        p.add_argument(
            "--fault",
            choices=("none", "scan-gap", "odometry-bias", "map-export", "no-truth"),
            default="none",
        )
        p.add_argument("--timeout", type=float, default=1200 if name == "sim" else 300)
        p.add_argument("--open", action="store_true")
        p.add_argument(
            "--navigation-only", action="store_true", help="ROS 2: 在参考地图上独立验收导航"
        )
        p.add_argument(
            "--generated-map",
            type=Path,
            help="ROS 2 navigation-only: 复用 trinary PGM 地图，执行两图导航对照",
        )
    args = parser.parse_args(argv)
    if args.command == "doctor":
        state = {
            "python": sys.version.split()[0],
            "ROS_DISTRO": os.environ.get("ROS_DISTRO"),
            **{p: shutil.which(p) for p in ("ros2", "gz", "docker")},
            **{
                p: importlib.util.find_spec(p) is not None
                for p in ("breezyslam", "evo", "rosbags", "rclpy")
            },
        }
        print(json.dumps(state, ensure_ascii=False, indent=2))
        return 0 if state["ros2"] and state["gz"] and state["rclpy"] else 3
    if args.command == "prepare-public":
        frames, spec = public_frames(project_root() / "third_party/breezyslam/examples/exp2.dat")
        write_bag(args.output, frames, spec, ros2=True)
        print(args.output.resolve())
        return 0
    cfg = load_config(args.config)
    if args.seed is not None:
        if not 1 <= args.seed <= 65535:
            parser.error("--seed must be 1..65535")
        cfg["seed"] = args.seed
    if args.scenario not in (
        "demo",
        "public",
        "healthy",
        "temporary-block",
        "unreachable",
        "narrow",
    ):
        parser.error("Unknown scenario")
    if args.timeout <= 0:
        parser.error("--timeout must be positive")
    if args.navigation_only and (args.command != "sim" or args.backend != "ros2"):
        parser.error("--navigation-only requires sim --backend ros2")
    if args.navigation_only and args.fault != "none":
        parser.error("--navigation-only does not inject mapping faults; omit --fault")
    if args.generated_map and not args.navigation_only:
        parser.error("--generated-map requires --navigation-only")
    if args.command == "sim" and args.scenario == "public":
        parser.error("public is a bag scenario; use bag --scenario public")
    if args.command == "bag" and args.scenario != "public":
        parser.error("bag mode uses scenario public; supply --bag for custom sensor data")
    if args.backend == "local" and args.scenario in ("temporary-block", "unreachable", "narrow"):
        parser.error("Navigation scenarios require --backend ros2")
    if args.bag and args.fault not in ("none", "map-export"):
        parser.error("Sensor fault injection is supported on built-in fixtures only")
    if (
        args.backend == "local"
        and args.command == "bag"
        and args.fault in ("odometry-bias", "no-truth")
    ):
        parser.error("Local public bag supports scan-gap and map-export; use sim for this fault")
    if args.backend == "local" and cfg.get("algorithm", {}).get("launch_argv"):
        parser.error("algorithm.launch_argv requires --backend ros2")
    request = dict(
        scenario=args.scenario,
        backend=args.backend,
        config=cfg,
        fault=args.fault,
        bag=str(args.bag.resolve()) if args.bag else None,
        mode=args.command,
        navigation_only=args.navigation_only,
        generated_map=str(args.generated_map.resolve()) if args.generated_map else None,
    )
    output = args.output or Path("results") / f"{datetime.now():%Y%m%d-%H%M%S-%f}-{args.scenario}"
    try:
        code, report = supervised_run(request, output, args.timeout)
    except ValueError as error:
        parser.error(str(error))
    print(f"report: {report}\nexit_code: {code}")
    if args.open:
        webbrowser.open(report.as_uri())
    return code


if __name__ == "__main__":
    sys.exit(main())
