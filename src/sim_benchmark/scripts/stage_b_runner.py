#!/usr/bin/env python3
"""
stage_b_runner
==============
阶段 B（导航）批量评测编排器。

前提
----
必须先跑完阶段 A（stage_a_runner.py），产出 stage_a_selected.yaml ——
里面是每个环境选出的"最优地图"路径。

方法论
------
阶段 B 的变量【只有规划器和控制器】：
  - 地图：冻结（来自阶段 A 的最优地图）
  - 定位：amcl（不用 SLAM，消除冷启动盲规划问题）
  - 起终点：从 GT 可通行区固定采样（同一 seed → 每次完全一致）
  - 机器人/传感器：不变

只有这四项全部固定，换 smac_2d+dwb 与 smac_hybrid+mppi 跑出来的差异
才真的是规划器/控制器的差异。

为什么用 amcl 而不是 slam
-------------------------
slam 模式下全局地图随探索进程增长，起点附近是空的 → 全局规划器盲规划
→ 与局部代价地图认知不一致 → 控制器一跟就撞。这不是规划器的问题，
是方法论错误（README 第 24 条）。amcl + 冻结地图把定位变成"已知问题"，
规划器才能真正被公平比较。

用法
----
    python3 stage_b_runner.py --config config/stage_b_matrix.yaml
    python3 stage_b_runner.py --config ... --only simple
    python3 stage_b_runner.py --config ... --dry-run
"""

import argparse
import csv
import json
import hashlib
import math
import os
import random
import shutil
from pathlib import Path
import signal
import subprocess
import sys
import time

from stage_a_runner import atomic_write, valid_map, prepare_outdir, check_sim_failure
from stage_a_runner import kill_group

try:
    import numpy as np
    import yaml
except ImportError:
    print("需要 numpy 和 pyyaml", file=sys.stderr)
    sys.exit(2)


class _Result:
    def __init__(self, returncode=-1, stdout="", stderr=""):
        self.returncode = returncode
        self.stdout = stdout
        self.stderr = stderr


def sh(cmd, timeout=None):
    try:
        return subprocess.run(cmd, capture_output=True, text=True, timeout=timeout)
    except subprocess.TimeoutExpired:
        return _Result(-1, "", f"TIMEOUT({timeout}s)")
    except FileNotFoundError as e:
        return _Result(-2, "", f"NOT FOUND: {e}")
    except OSError as e:
        return _Result(-3, "", f"OSError: {e}")


def _share(pkg):
    r = sh(["ros2", "pkg", "prefix", pkg], timeout=10)
    if r.returncode != 0:
        return None
    return os.path.join(r.stdout.strip(), "share", pkg)


# ---------------------------------------------------------------------------
# GT 目标点采样（从 GT 栅格，保证可通行且离墙有余量）
# ---------------------------------------------------------------------------


def _read_pgm(path):
    with open(path, "rb") as f:
        raw = f.read()
    tokens, i = [], 2
    while len(tokens) < 3:
        while i < len(raw) and raw[i:i + 1].isspace():
            i += 1
        if raw[i:i + 1] == b"#":
            while i < len(raw) and raw[i:i + 1] != b"\n":
                i += 1
            continue
        j = i
        while j < len(raw) and not raw[j:j + 1].isspace():
            j += 1
        tokens.append(raw[i:j])
        i = j
    i += 1
    w, h = int(tokens[0]), int(tokens[1])
    return np.frombuffer(raw, np.uint8, w * h, i).reshape(h, w)


def sample_goals(gt_yaml, n, spawn, seed=0, min_dist=3.0, clearance=0.5):
    """与 benchmark_runner 同一套逻辑，抽出来复用（保持两处口径一致）。"""
    with open(gt_yaml) as f:
        meta = yaml.safe_load(f)
    img = _read_pgm(os.path.join(os.path.dirname(os.path.abspath(gt_yaml)), meta["image"]))
    res = float(meta["resolution"])
    ox, oy = float(meta["origin"][0]), float(meta["origin"][1])
    probability = img / 255.0 if meta.get("negate", 0) else (255.0-img) / 255.0
    free = probability < float(meta.get("free_thresh", .196))
    h, w = free.shape
    r = max(1, int(round(clearance / res)))
    cum = np.cumsum(np.cumsum(free.astype(np.int32), 0), 1)
    cum = np.pad(cum, ((1, 0), (1, 0)))
    ys, xs = np.mgrid[0:h, 0:w]
    y0 = np.clip(ys - r, 0, h - 1); y1 = np.clip(ys + r, 0, h - 1)
    x0 = np.clip(xs - r, 0, w - 1); x1 = np.clip(xs + r, 0, w - 1)
    area = (y1 - y0 + 1) * (x1 - x0 + 1)
    filled = cum[y1 + 1, x1 + 1] - cum[y0, x1 + 1] - cum[y1 + 1, x0] + cum[y0, x0]
    safe = free & (filled == area)
    safe[:r, :] = safe[-r:, :] = False
    safe[:, :r] = safe[:, -r:] = False
    from scipy.ndimage import label
    components, _ = label(safe)
    spawn_cell = (h-1-int(math.floor((spawn[1]-oy)/res)),
                  int(math.floor((spawn[0]-ox)/res)))
    if not (0 <= spawn_cell[0] < h and 0 <= spawn_cell[1] < w) or not safe[spawn_cell]:
        raise RuntimeError("Spawn is not in a safe navigable component")
    safe &= components == components[spawn_cell]
    cand = np.argwhere(safe)
    if len(cand) == 0:
        raise RuntimeError("GT 里没有满足 clearance 的可通行格")

    rng = random.Random(seed)
    goals, tries = [], 0
    while len(goals) < n and tries < 20000:
        tries += 1
        iy, ix = cand[rng.randrange(len(cand))]
        gx = ox + (ix + 0.5) * res
        gy = oy + (h - 1 - iy + 0.5) * res
        if (gx - spawn[0]) ** 2 + (gy - spawn[1]) ** 2 < min_dist ** 2:
            continue
        if any((gx - a) ** 2 + (gy - b) ** 2 < 9.0 for a, b in goals):
            continue
        goals.append((gx, gy))
    return goals


def read_spawn(world):
    s = _share("sim_worlds")
    with open(os.path.join(s, "maps", f"{world}_meta.json")) as f:
        meta = json.load(f)
    sp = meta["recommended_spawn"]
    return (sp["x"], sp["y"])


def freeze_maps(selected, target):
    """Copy map pixels and metadata once for all comparisons in this batch."""
    snapshot = {}
    for world, row in selected.items():
        source = Path(row["map_yaml"]).expanduser().resolve()
        if not valid_map(source):
            raise ValueError(f"Invalid selected map: {source}")
        meta = yaml.safe_load(source.read_text(encoding="utf-8"))
        image = source.parent / meta["image"]
        pixels = image.read_bytes()
        directory = Path(target) / world
        directory.mkdir(parents=True, exist_ok=True)
        frozen_image = directory / ("map" + image.suffix)
        frozen_image.write_bytes(pixels)
        meta["image"] = frozen_image.name
        frozen_yaml = directory / "map.yaml"
        atomic_write(frozen_yaml, yaml.safe_dump(meta))
        snapshot[world] = dict(row, map_yaml=str(frozen_yaml.resolve()),
                              source_map_yaml=str(source),
                              image_sha256=hashlib.sha256(pixels).hexdigest())
    atomic_write(Path(target) / "manifest.json", json.dumps(snapshot, indent=2))
    return snapshot


# ---------------------------------------------------------------------------
# 运行
# ---------------------------------------------------------------------------


def build_cmd(exp, cfg, run_name, map_yaml):
    cmd = [
        "ros2", "launch", "sim_bringup", "sim.launch.py",
        f"robot:={exp['robot']}",
        f"world:={exp['world']}",
        "mapping:=none",                         # 阶段B不建图
        "navigation:=nav2",
        f"global_planner:={exp['global_planner']}",
        f"local_planner:={exp['local_planner']}",
        f"odom_source:={exp.get('odom_source', 'gt')}",
        f"use_gimbal:={str(exp.get('use_gimbal', False)).lower()}",
        f"headless:={str(cfg.get('headless', True)).lower()}",
        "localization:=amcl",                    # ★ 关键：固定地图 + AMCL
        f"map:={map_yaml}",
        "metrics:=true",
        f"run_name:={run_name}",
        f"metrics_output_dir:={os.path.expanduser(cfg.get('output_dir', '~/.ros/sim_stage_b'))}",
    ]
    if cfg.get("headless", True):
        cmd.append("viz:=false")
    if exp.get("sensor_profile"):
        cmd.append(f"sensor_profile:={exp['sensor_profile']}")
    return cmd


def wait_for_action(name, timeout, settle=60.0):
    t0 = time.time()
    while time.time() - t0 < timeout:
        r = sh(["ros2", "action", "list"], timeout=15)
        el = time.time() - t0
        if name in (r.stdout or ""):
            return True, el
        time.sleep(3.0)
    return False, time.time() - t0


class NavigationSession:
    """One persistent action client; bounded discovery and explicit cancellation."""
    def __init__(self):
        import rclpy
        from rclpy.action import ActionClient
        from rclpy.node import Node
        from rclpy.parameter import Parameter
        from nav2_msgs.action import NavigateToPose
        from tf2_ros import Buffer, TransformListener
        self.ros = rclpy
        self.owns_context = not rclpy.ok()
        if self.owns_context:
            rclpy.init()
        self.node = Node("stage_b_client", parameter_overrides=[Parameter("use_sim_time", value=True)])
        self.client = ActionClient(self.node, NavigateToPose, "/navigate_to_pose")
        self.buffer = Buffer()
        self.listener = TransformListener(self.buffer, self.node)
        self.handle = None
        self.safe_to_continue = True

    def _wait(self, future, timeout):
        deadline = time.monotonic() + timeout
        while not future.done() and time.monotonic() < deadline:
            self.ros.spin_once(self.node, timeout_sec=min(.1, max(0., deadline-time.monotonic())))
        return future.done()

    def ready(self, timeout, proc, log_path):
        from rclpy.time import Time
        from lifecycle_msgs.srv import GetState
        started = time.monotonic()
        clients = [self.node.create_client(GetState, f"/{name}/get_state")
                   for name in ("amcl", "bt_navigator", "planner_server", "controller_server")]
        active = set()
        try:
            while time.monotonic() - started < timeout:
                check_sim_failure(proc, log_path)
                self.ros.spin_once(self.node, timeout_sec=.1)
                for index, client in enumerate(clients):
                    if index in active or not client.service_is_ready():
                        continue
                    future = client.call_async(GetState.Request())
                    if self._wait(future, .5) and future.result().current_state.id == 3:
                        active.add(index)
                if (len(active) == len(clients) and self.client.server_is_ready()
                        and self.buffer.can_transform("map", "base_footprint", Time())):
                    return True, time.monotonic() - started
            return False, time.monotonic() - started
        finally:
            for client in clients:
                self.node.destroy_client(client)

    def send_goal(self, x, y, yaw, timeout):
        from action_msgs.msg import GoalStatus
        from nav2_msgs.action import NavigateToPose
        started = time.monotonic()
        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = "map"
        goal.pose.header.stamp = self.node.get_clock().now().to_msg()
        goal.pose.pose.position.x = float(x)
        goal.pose.pose.position.y = float(y)
        goal.pose.pose.orientation.z = math.sin(yaw / 2)
        goal.pose.pose.orientation.w = math.cos(yaw / 2)
        future = self.client.send_goal_async(goal)
        if not self._wait(future, min(timeout, 10)):
            # Acceptance is unknown: caller must tear down this simulation.
            self.safe_to_continue = False
            return "TIMEOUT", time.monotonic() - started
        self.handle = future.result()
        if not self.handle.accepted:
            self.handle = None
            return "REJECTED", time.monotonic() - started
        result = self.handle.get_result_async()
        if not self._wait(result, max(0., timeout - (time.monotonic() - started))):
            canceled = self.handle.cancel_goal_async()
            self._wait(canceled, 5)
            if self._wait(result, 5):
                self.handle = None
            else:
                self.safe_to_continue = False
            return "TIMEOUT", time.monotonic() - started
        status = {GoalStatus.STATUS_SUCCEEDED: "SUCCEEDED", GoalStatus.STATUS_ABORTED: "ABORTED",
                  GoalStatus.STATUS_CANCELED: "CANCELED"}.get(result.result().status, "UNKNOWN")
        self.handle = None
        return status, time.monotonic() - started

    def close(self):
        if self.ros.ok() and self.handle is not None and self.handle.accepted:
            try:
                self._wait(self.handle.cancel_goal_async(), 3)
            except Exception as exc:
                print(f"Cancellation during shutdown failed; stopping simulation: {exc}")
        self.client.destroy()
        self.node.destroy_node()
        if self.owns_context and self.ros.ok():
            self.ros.shutdown()

    def flush_metrics(self, expected_goals):
        from std_srvs.srv import Trigger
        client = self.node.create_client(Trigger, '/eval/flush')
        try:
            deadline = time.monotonic()+10
            while time.monotonic() < deadline:
                if not client.wait_for_service(timeout_sec=.5):
                    continue
                future = client.call_async(Trigger.Request())
                if self._wait(future, 1) and future.result().success:
                    metrics = json.loads(future.result().message)
                    if metrics.get('goals_total', 0) >= expected_goals:
                        return True
                self.ros.spin_once(self.node, timeout_sec=.2)
            return False
        finally:
            self.node.destroy_client(client)


def run_one(exp, cfg, idx, total, map_yaml, repeat=0):
    runner_cfg = cfg["runner"]
    if runner_cfg.get("rmw_implementation"):
        os.environ["RMW_IMPLEMENTATION"] = runner_cfg["rmw_implementation"]
    run_name = f"B_{exp['name']}_r{repeat + 1}"
    out_dir = os.path.expanduser(runner_cfg["output_dir"])
    run_dir = os.path.join(out_dir, run_name)
    os.makedirs(run_dir, exist_ok=True)
    prepare_outdir(run_dir)
    prepare_outdir(os.path.expanduser(os.path.join("~/.ros/sim_eval", run_name)))

    print(f"\n{'='*72}\n[{run_name}]  ({idx+1}/{total})  "
          f"{exp['global_planner']}+{exp['local_planner']}  map={os.path.basename(map_yaml)}\n{'='*72}",
          flush=True)

    gt = os.path.join(_share("sim_worlds"), "maps", f"{exp['world']}.yaml")
    spawn = read_spawn(exp["world"])
    try:
        goals = sample_goals(gt, int(runner_cfg.get("goals_per_run", 5)), spawn,
                             seed=int(runner_cfg.get("goal_seed", 1000)) + repeat,
                             min_dist=float(cfg.get("min_goal_dist", 3.0)))
        if len(goals) != int(runner_cfg.get("goals_per_run", 5)):
            raise RuntimeError("Insufficient safe goals for the requested benchmark")
    except Exception as e:
        print(f"  [跳过] 目标采样失败：{e}")
        return None
    print(f"  目标：{[(round(g[0],1), round(g[1],1)) for g in goals]}", flush=True)

    cmd = build_cmd(exp, runner_cfg, run_name, map_yaml)
    print("  " + " ".join(cmd), flush=True)
    log_path = os.path.join(run_dir, "launch.log")
    log_fh = open(log_path, "w", encoding="utf-8", buffering=1)
    proc = subprocess.Popen(cmd, stdout=log_fh, stderr=subprocess.STDOUT,
                            text=True, start_new_session=True)

    result = {"run_name": run_name, "experiment": exp["name"], "world": exp["world"],
              "global_planner": exp["global_planner"], "local_planner": exp["local_planner"],
              "map_yaml": map_yaml, "status": "INCOMPLETE",
              "goals_requested": len(goals), "goals": goals,
              "goal_seed": int(runner_cfg.get("goal_seed", 1000)) + repeat}
    result['rmw_implementation'] = os.environ.get('RMW_IMPLEMENTATION', 'rmw_fastrtps_cpp')
    session = None
    try:
        session = NavigationSession()
        ok, waited = session.ready(float(runner_cfg.get("startup_timeout", 150)), proc, log_path)
        print(f"  导航栈{'就绪' if ok else '超时'}（{waited:.1f}s）", flush=True)
        if not ok:
            result["status"] = "STARTUP_TIMEOUT"
            return result

        # AMCL 需要先收敛：给机器人一点时间做首次定位
        time.sleep(float(runner_cfg.get("amcl_settle", 10.0)))

        statuses = []
        result["goal_statuses"] = statuses
        result["goal_seconds"] = []
        for gx, gy in goals:
            check_sim_failure(proc, log_path)
            st, dt = session.send_goal(gx, gy, 0.0, float(runner_cfg.get("goal_timeout", 150)))
            statuses.append(st)
            result["goal_seconds"].append(dt)
            print(f"    目标 ({gx:6.2f},{gy:6.2f}) -> {st}  {dt:6.1f}s", flush=True)
            if st == "TIMEOUT" and not session.safe_to_continue:
                break
        result["goal_statuses"] = statuses
        terminal = sum(status in ('SUCCEEDED', 'ABORTED', 'CANCELED', 'TIMEOUT') for status in statuses)
        result["status"] = "DONE" if session.flush_metrics(terminal) else "METRICS_INCOMPLETE"
    except KeyboardInterrupt:
        result["status"] = "INTERRUPTED"
        raise
    except Exception as e:
        result["status"] = f"ERROR:{type(e).__name__}"
        result["error"] = str(e)
        print(f"  [错误] {e}")
    finally:
        try:
            if session is not None:
                session.close()
        finally:
            kill_group(proc)
        log_fh.close()
        atomic_write(os.path.join(run_dir, "run_result.json"), json.dumps(result, indent=2))
        time.sleep(float(runner_cfg.get("cooldown", 8)))
    return result


# ---------------------------------------------------------------------------
# 主流程
# ---------------------------------------------------------------------------


def main():
    ap = argparse.ArgumentParser(description="阶段B：导航评测")
    ap.add_argument("--config", required=True)
    ap.add_argument("--only", default="")
    ap.add_argument("--dry-run", action="store_true")
    args = ap.parse_args()

    with open(args.config) as f:
        cfg = yaml.safe_load(f)
    runner_cfg = cfg["runner"]
    sel_path = os.path.expanduser(runner_cfg["stage_a_selected"])
    if not os.path.isfile(sel_path):
        print(f"找不到阶段A的选图结果：{sel_path}\n"
              f"请先运行：python3 stage_a_runner.py --config config/stage_a_matrix.yaml",
              file=sys.stderr)
        sys.exit(2)
    with open(sel_path) as f:
        selected = yaml.safe_load(f)["selected"]
    print(f"阶段A选出的地图：{ {w: v['experiment'] for w, v in selected.items()} }")

    exps = cfg["experiments"]
    if args.only:
        exps = [e for e in exps if args.only in e["world"]]
    if not exps:
        print("没有匹配的实验。")
        return

    # 展开：每个实验 × repeats；地图按 world 查表
    plan = []
    for e in exps:
        w = e["world"]
        if w not in selected:
            raise SystemExit(f"Missing qualified Stage A map: {w}")
        mp = selected[w]
        ghost = mp.get("ghost_wall_ratio")
        if not isinstance(ghost, (int, float)) or not 0 <= ghost < 0.3:
            raise SystemExit(f"Map failed ghost-wall gate: {w}")
        if not valid_map(mp["map_yaml"]):
            raise SystemExit(f"Missing or invalid map: {mp['map_yaml']}")
        for i in range(int(e.get("repeats", 1))):
            plan.append((e, selected[w]["map_yaml"], i))

    print(f"阶段 B：{len(plan)} 次运行")
    if args.dry_run:
        for e, m, repeat in plan:
            print(f"  - {e['name']:34s} {e['global_planner']}+{e['local_planner']:10s} "
                  f"map={os.path.basename(m)}")
        return

    out_root = os.path.expanduser(runner_cfg["output_dir"])
    os.makedirs(out_root, exist_ok=True)
    snapshot = freeze_maps({e["world"]: selected[e["world"]] for e, _, _ in plan},
                           Path(out_root) / "frozen_maps" / str(time.time_ns()))
    plan = [(e, snapshot[e["world"]]["map_yaml"], repeat) for e, _, repeat in plan]
    rows, results = [], []

    try:
        for i, (exp, map_yaml, repeat) in enumerate(plan):
            res = run_one(exp, cfg, i, len(plan), map_yaml, repeat)
            if not res:
                continue
            results.append(res)
            if res.get("status") != "DONE":
                continue
            run_dir = os.path.join(out_root, res["run_name"])
            rep = evaluate_nav(run_dir, res["run_name"])
            res["report"] = rep
            rows.append(flatten(exp, res, rep))
    except KeyboardInterrupt:
        print("\n收到中断，收尾中…")

    resummarize(out_root, cfg)
    print(f"\n原始结果 -> {os.path.join(out_root, 'stage_b_results.json')}")
    return 0 if len(results) == len(plan) and all(r.get("status") == "DONE" for r in results) else 1


def evaluate_nav(run_dir, run_name):
    mj = os.path.join(run_dir, "metrics.json")
    if not os.path.isfile(mj) and not os.path.isfile(os.path.join(run_dir, "run_result.json")):
        mj = os.path.join(os.path.expanduser("~/.ros/sim_eval"), run_name, "metrics.json")
    nav = {}
    if os.path.isfile(mj):
        try:
            with open(mj) as f:
                nav = json.load(f).get("navigation", {})
        except Exception:
            pass
    return {"navigation": nav}


def resummarize(out_root, cfg):
    import fcntl
    root = Path(out_root).expanduser()
    root.mkdir(parents=True, exist_ok=True)
    rows, results = [], []
    with (root / '.summary.lock').open('a') as lock:
        fcntl.flock(lock, fcntl.LOCK_EX)
        for exp in cfg['experiments']:
            for repeat in range(int(exp.get('repeats', 1))):
                run = root / f"B_{exp['name']}_r{repeat+1}"
                manifest = run / 'run_result.json'
                if not manifest.is_file():
                    continue
                result = json.loads(manifest.read_text())
                report = evaluate_nav(str(run), result['run_name'])
                result['report'] = report
                results.append(result)
                rows.append(flatten(exp, result, report))
        write_summary(str(root), rows, cfg.get('summary_metrics', []))
        atomic_write(root / 'stage_b_results.json', json.dumps(results, indent=2))
    return rows, results


def flatten(exp, res, rep):
    nav = (rep or {}).get("navigation", {}) or {}
    st = res.get("goal_statuses", [])
    row = {
        "experiment": exp["name"],
        "run": res["run_name"],
        "status": res.get("status", "DONE"),
        "world": exp["world"],
        "global_planner": exp["global_planner"],
        "local_planner": exp["local_planner"],
        "goals": res.get("goals_requested", len(st)),
        "goals_ok": sum(1 for s in st if s == "SUCCEEDED"),
    }
    row["success_rate"] = round(row["goals_ok"] / row["goals"], 3) if row["goals"] else None
    for k in ("time_to_goal_mean", "tracking_error_rmse", "collision_count",
              "min_obstacle_dist", "control_smoothness", "path_length_ratio"):
        row[k] = nav.get(k)
    return row


def write_summary(out_root, rows, metric_specs):
    if not rows:
        print("\n没有可汇总的有效结果。")
        return
    csv_path = os.path.join(out_root, "stage_b_summary.csv")
    with open(csv_path, "w", newline="", encoding="utf-8") as f:
        w = csv.DictWriter(f, fieldnames=list(rows[0].keys()))
        w.writeheader()
        w.writerows(rows)

    agg = {}
    for r in rows:
        agg.setdefault(r["experiment"], []).append(r)

    print(f"\n{'='*100}\n阶段 B 汇总\n{'='*100}")
    cols = ["success_rate", "time_to_goal_mean", "tracking_error_rmse", "collision_count"]
    labels = ["成功率", "耗时(s)", "跟踪误差", "碰撞"]
    hdr = f"{'实验':34s} " + " ".join(f"{l:>10s}" for l in labels) + "  n"
    print(hdr)
    print("-" * len(hdr))

    def fmt(v):
        if v is None:
            return "     -"
        return f"{v:10.3f}" if isinstance(v, float) else f"{v:10d}"

    for name, rs in agg.items():
        vals = []
        for c in cols:
            nums = [r[c] for r in rs if isinstance(r.get(c), (int, float))]
            vals.append(sum(nums) / len(nums) if nums else None)
        print(f"{name:34s} " + " ".join(fmt(v) for v in vals) + f"  {len(rs)}")

    print("\nCompare only within the same world and goal schedule. "
          "Interpret timing together with success rate; no cross-world winner is inferred.")
    print(f"\n明细 CSV -> {csv_path}")


if __name__ == "__main__":
    sys.exit(main())
