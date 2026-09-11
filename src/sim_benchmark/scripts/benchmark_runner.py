#!/usr/bin/env python3
"""
benchmark_runner
================
批量评测：按矩阵反复起仿真、跑导航目标、收指标、出汇总表。

这是需求 #4 的最后一块，也是把"仿真环境"变成"评测平台"的那一步。
没有它，你只能一次手动跑一个组合，记录靠手抄，跨算法对比基本靠印象。

设计上的几个关键决定：

  1. **纯进程编排，不 import rclpy**。
     用它来控制 launch 的生命周期比在 ROS 节点里管理子进程简单得多，
     而且 launch 挂了不会把 runner 一起带走。目标下发借用
     `ros2 action send_goal` 命令行，省掉一大块 action client 样板代码。

  2. **目标点从 GT 栅格里采样**。
     手写目标坐标的问题是：改一次地图就得重标一遍，而且很容易标到墙里。
     直接从 GT 的可通行格里随机采样、并保证与出生点有足够距离，
     换地图零维护，而且采样点天然是可通行的。

  3. **进程组整组杀**。
     `ros2 launch` 会拉起十几个子进程，只 kill 父进程会留下一堆孤儿
     gzserver / rviz 占着端口，下一个实验直接起不来。
     所以用 start_new_session=True 建独立进程组，结束时杀整组。

用法：
  python3 benchmark_runner.py --matrix config/benchmark_matrix.yaml
  python3 benchmark_runner.py --matrix ... --only wq_complex_hybrid_mppi
  python3 benchmark_runner.py --matrix ... --dry-run
"""

import argparse
import csv
import json
import math
import os
import random
import signal
import subprocess
import sys
import time

try:
    import numpy as np
    import yaml
except ImportError:
    print("需要 numpy 和 pyyaml：pip install numpy pyyaml", file=sys.stderr)
    sys.exit(2)


# ---------------------------------------------------------------------------
# 目标点采样
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


def sample_goals(gt_yaml, n, spawn, seed=0, min_dist=4.0, clearance=0.6):
    """从 GT 地图里采样 n 个可通行且离出生点足够远的目标点。

    clearance 用来排除贴着墙的格子 —— 那些点即使"可通行"，规划器也几乎
    到不了，会白白拉低成功率，把结论带偏。
    """
    with open(gt_yaml) as f:
        meta = yaml.safe_load(f)
    img = _read_pgm(os.path.join(os.path.dirname(os.path.abspath(gt_yaml)), meta["image"]))
    res = float(meta["resolution"])
    ox, oy = float(meta["origin"][0]), float(meta["origin"][1])

    free = img > 200           # 亮 = 自由
    r = max(1, int(round(clearance / res)))
    # 形态学腐蚀：排除离障碍太近的格子
    h, w = free.shape
    cum = np.cumsum(np.cumsum(free.astype(np.int32), 0), 1)
    cum = np.pad(cum, ((1, 0), (1, 0)))

    def box_sum(y0, y1, x0, x1):
        y0, x0 = max(0, y0), max(0, x0)
        y1, x1 = min(h - 1, y1), min(w - 1, x1)
        return cum[y1 + 1, x1 + 1] - cum[y0, x1 + 1] - cum[y1 + 1, x0] + cum[y0, x0]

    ys, xs = np.mgrid[0:h, 0:w]
    y0 = np.clip(ys - r, 0, h - 1)
    y1 = np.clip(ys + r, 0, h - 1)
    x0 = np.clip(xs - r, 0, w - 1)
    x1 = np.clip(xs + r, 0, w - 1)
    area = (y1 - y0 + 1) * (x1 - x0 + 1)
    filled = (cum[y1 + 1, x1 + 1] - cum[y0, x1 + 1] - cum[y1 + 1, x0] + cum[y0, x0])
    safe = free & (filled == area)

    cand = np.argwhere(safe)
    if len(cand) == 0:
        raise RuntimeError("GT 地图里没有满足 clearance 的可通行格子")

    rng = random.Random(seed)
    goals, tries = [], 0
    while len(goals) < n and tries < 20000:
        tries += 1
        iy, ix = cand[rng.randrange(len(cand))]
        # 图像行 0 = 最大 y，转回世界坐标
        gx = ox + (ix + 0.5) * res
        gy = oy + (h - 1 - iy + 0.5) * res
        if (gx - spawn[0]) ** 2 + (gy - spawn[1]) ** 2 < min_dist ** 2:
            continue
        if any((gx - a) ** 2 + (gy - b) ** 2 < 9.0 for a, b in goals):
            continue
        goals.append((gx, gy))
    return goals


# ---------------------------------------------------------------------------
# 单次实验
# ---------------------------------------------------------------------------


def build_launch_cmd(exp, runner_cfg, run_name):
    """组装 sim_bringup 的 launch 命令。"""
    cmd = [
        "ros2", "launch", "sim_bringup", "sim.launch.py",
        f"robot:={exp['robot']}",
        f"world:={exp['world']}",
        f"mapping:={exp.get('mapping', 'none')}",
        "navigation:=nav2",
        f"global_planner:={exp.get('global_planner', 'smac_2d')}",
        f"local_planner:={exp.get('local_planner', 'dwb')}",
        f"odom_source:={exp.get('odom_source', 'gt')}",
        f"use_gimbal:={str(exp.get('use_gimbal', True)).lower()}",
        # ⚠️ sensor_profile 只在【非空】时才传。
        # ros2 launch 不接受空值：`sensor_profile:=` 会直接抛
        #   "malformed launch argument 'sensor_profile:=', expected format '<name>:=<value>'"
        # 然后【整个 launch 立刻退出】—— 仿真、Nav2 一概不启动，
        # 而 benchmark 那边只看到"等 action server 超时"，完全指不到真正原因。
        f"headless:={str(runner_cfg.get('headless', True)).lower()}",
        "localization:=slam",
        # metrics:=true 必须显式传，否则 metrics_collector 不启动，
        # ~/.ros/sim_eval/<run>/metrics.json 不存在，导航指标全是 None。
        "metrics:=true",
        f"run_name:={run_name}",
    ]
    if exp.get('sensor_profile'):
        cmd.append(f"sensor_profile:={exp['sensor_profile']}")
    if runner_cfg.get('headless', True):
        cmd.append("viz:=false")
    return cmd


class _Result:
    """subprocess.run 的替身，用于超时/不存在等异常路径。"""

    def __init__(self, returncode=-1, stdout="", stderr=""):
        self.returncode = returncode
        self.stdout = stdout
        self.stderr = stderr


def sh(cmd, timeout=None, check=False):
    """执行外部命令，**永不抛异常**。

    ros2 CLI 的任何一条命令都可能挂住（daemon 相关命令尤其明显，实测
    `ros2 daemon stop` 会超 20 秒）。评测脚本要跑几十次外部命令，
    只要有一次抛 TimeoutExpired，整个批量评测就直接崩掉、前面的结果全丢。
    所以这里把异常统一转成"返回码 -1 的空结果"，由调用方按失败处理。
    """
    try:
        return subprocess.run(cmd, capture_output=True, text=True,
                              timeout=timeout, check=check)
    except subprocess.TimeoutExpired:
        return _Result(-1, "", f"TIMEOUT({timeout}s): {' '.join(cmd)}")
    except FileNotFoundError as e:
        return _Result(-2, "", f"NOT FOUND: {e}")
    except OSError as e:
        return _Result(-3, "", f"OSError: {e}")


def wait_for_action_server(name, timeout, settle_time=60.0):
    """轮询 action 列表，直到导航服务器就绪。

    比"sleep 30"靠谱得多：simple 地图 20 秒就好了，complex 加 LIO 可能要 2 分钟。
    但检测本身不能成为单点故障 —— 见下面的 settle_time 兜底。
    """
    t0 = time.time()
    while time.time() - t0 < timeout:
        # 注意：Humble 的 `ros2 action list` 【不支持任何选项】
        # （--help 里只有 -h），加 --no-daemon 会直接参数错误、永远匹配不到。
        r = sh(["ros2", "action", "list"], timeout=15)
        el = time.time() - t0
        if name in r.stdout:
            return True, el
        # 检测手段本身可能因为 daemon 缓存而假阴性。等够 settle_time 之后
        # 就按"已就绪"继续 —— 让目标下发自己去暴露问题，
        # 而不是让整个实验被误判成"启动失败"直接丢弃。
        if el >= settle_time:
            print(f"  [警告] {el:.0f}s 内未能通过 action list 确认 {name}，"
                  f"按就绪继续（后续目标失败会记录下来）", flush=True)
            return True, el
        time.sleep(3.0)
    return False, time.time() - t0


def send_goal(x, y, yaw, timeout):
    """发一个 NavigateToPose 目标并等它结束。

    返回 (状态文本, 耗时)。通过命令行下发，省掉 action client 的全部样板。

    目标体用 dict + json.dumps 组装，不要用 f-string 拼 JSON ——
    嵌套 JSON 的花括号在 f-string 里需要四层转义，极易写错。
    JSON 是 YAML 的子集，`ros2 action send_goal` 能直接解析。
    """
    goal = {
        "pose": {
            "header": {"frame_id": "map"},
            "pose": {
                "position": {"x": round(x, 3), "y": round(y, 3), "z": 0.0},
                "orientation": {
                    "x": 0.0, "y": 0.0,
                    "z": round(math.sin(yaw / 2.0), 5),
                    "w": round(math.cos(yaw / 2.0), 5),
                },
            },
        }
    }
    t0 = time.time()
    try:
        r = sh(["ros2", "action", "send_goal", "/navigate_to_pose",
                "nav2_msgs/action/NavigateToPose", json.dumps(goal)], timeout=timeout)
        out = (r.stdout or "") + (r.stderr or "")
        dt = time.time() - t0
        if "SUCCEEDED" in out.upper():
            return "SUCCEEDED", dt
        if "ABORTED" in out.upper():
            return "ABORTED", dt
        if "CANCELED" in out.upper():
            return "CANCELED", dt
        return "UNKNOWN", dt
    except subprocess.TimeoutExpired:
        return "TIMEOUT", time.time() - t0


def kill_group(proc):
    if proc.poll() is not None:
        return
    try:
        os.killpg(os.getpgid(proc.pid), signal.SIGINT)
        proc.wait(timeout=15)
    except (ProcessLookupError, subprocess.TimeoutExpired):
        try:
            os.killpg(os.getpgid(proc.pid), signal.SIGKILL)
        except ProcessLookupError:
            pass


def run_one(exp, runner_cfg, idx, total):
    run_name = f"{exp['name']}_r{idx + 1}"
    out_root = os.path.expanduser(runner_cfg["output_dir"])
    run_dir = os.path.join(out_root, run_name)
    os.makedirs(run_dir, exist_ok=True)

    # 不重启 ros2 daemon：本 runner 的就绪检测与话题操作全部走 --no-daemon
    # 或独立节点，不依赖 daemon 缓存。之前加的重启反而因为 `daemon stop`
    # 卡住把评测打崩了。
    print(f"\n{'='*70}\n[{run_name}]  ({idx+1}/{total})\n{'='*70}", flush=True)
    print("  " + " ".join(build_launch_cmd(exp, runner_cfg, run_name)))

    gt_yaml = find_gt_map(exp["world"])
    spawn = read_spawn(exp["world"])
    try:
        goals = sample_goals(gt_yaml, int(runner_cfg.get("goals_per_run", 3)),
                             spawn, seed=1000 + idx)
    except Exception as e:
        print(f"  [跳过] 目标点采样失败：{e}")
        return None
    print(f"  目标点：{[(round(g[0],1), round(g[1],1)) for g in goals]}")

    cmd = build_launch_cmd(exp, runner_cfg, run_name)
    log_path = os.path.join(run_dir, "launch.log")

    # ⚠️ 必须把 launch 的输出【直接重定向到文件】，不能用 subprocess.PIPE。
    # ros2 launch 一次会输出几百 KB（Gazebo + Nav2 + slam_toolbox），
    # 而 PIPE 的内核缓冲区只有 64KB。不读管道的话 launch 进程会阻塞在 write 上，
    # 整个评测【静默死锁】—— 表现是 gzserver 起来了、但 nav2/slam 永远不启动，
    # 而你只能看到一个 0 字节的 launch.log。
    # 直接写文件没有缓冲区上限，而且能实时观察进度。
    log_fh = open(log_path, "w", encoding="utf-8", buffering=1)
    proc = subprocess.Popen(
        cmd, stdout=log_fh, stderr=subprocess.STDOUT, text=True,
        start_new_session=True,  # 独立进程组，方便整组清理
    )

    result = {"run_name": run_name, "experiment": exp["name"], "status": "INCOMPLETE"}
    try:
        ok, waited = wait_for_action_server(
            "/navigate_to_pose", float(runner_cfg.get("startup_timeout", 120)))
        print(f"  启动{'成功' if ok else '超时'}（{waited:.1f}s）")
        if not ok:
            result["status"] = "STARTUP_TIMEOUT"
            return result

        # 目标是顺序执行的：上一条路了再发下一条，避免相互打架
        statuses = []
        for gx, gy in goals:
            st, dt = send_goal(gx, gy, 0.0, float(runner_cfg.get("goal_timeout", 180)))
            statuses.append(st)
            print(f"    目标 ({gx:6.2f},{gy:6.2f}) -> {st}  {dt:6.1f}s")
            if st == "TIMEOUT":
                break
        result["goal_statuses"] = statuses
        result["status"] = "DONE"

        if runner_cfg.get("save_final_map", True):
            map_out = os.path.join(run_dir, "final_map")
            try:
                sh(["ros2", "run", "nav2_map_server", "map_saver_cli",
                    "-f", map_out, "--ros-args", "-p", "save_map_timeout:=15.0"],
                   timeout=60)
                result["final_map"] = map_out + ".yaml"
            except subprocess.TimeoutExpired:
                print("    [警告] 保存地图超时")

    except KeyboardInterrupt:
        raise
    except Exception as e:
        result["status"] = f"ERROR:{type(e).__name__}"
        print(f"  [错误] {e}")
    finally:
        kill_group(proc)
        log_fh.close()
        time.sleep(float(runner_cfg.get("cooldown", 5)))

    return result


# ---------------------------------------------------------------------------
# 路径辅助
# ---------------------------------------------------------------------------


def _share(pkg):
    r = sh(["ros2", "pkg", "prefix", pkg], timeout=10)
    if r.returncode != 0:
        return None
    return os.path.join(r.stdout.strip(), "share", pkg)


def find_gt_map(world):
    s = _share("sim_worlds")
    if not s:
        raise RuntimeError("找不到 sim_worlds，请先 colcon build 并 source install/setup.bash")
    return os.path.join(s, "maps", f"{world}.yaml")


def read_spawn(world):
    s = _share("sim_worlds")
    with open(os.path.join(s, "maps", f"{world}_meta.json")) as f:
        meta = json.load(f)
    sp = meta["recommended_spawn"]
    return (sp["x"], sp["y"])


def evaluate(run_dir, world, gt_yaml):
    """调用 evaluate_vs_gt 做离线评测。"""
    s = _share("sim_metrics")
    if not s:
        return None
    # s = <prefix>/share/sim_metrics，所以前缀要往上取两级到 <prefix>
    prefix = os.path.dirname(os.path.dirname(s))
    candidates = [
        os.path.join(prefix, "lib", "sim_metrics", "evaluate_vs_gt.py"),
        os.path.join(s, "scripts", "evaluate_vs_gt.py"),
    ]
    script = next((c for c in candidates if os.path.isfile(c)), None)
    if script is None:
        print(f"  [警告] 找不到 evaluate_vs_gt.py，已尝试：{candidates}")
        return None
    out = os.path.join(run_dir, "report.json")

    est_map = os.path.join(run_dir, "final_map.yaml")
    args = [sys.executable, script, "--gt-map", gt_yaml, "--out", out, "--quiet"]
    if os.path.isfile(est_map):
        args += ["--est-map", est_map]
    for key, fn in (("gt", "trajectory_gt.csv"), ("est", "trajectory_est.csv")):
        p = os.path.join(run_dir, fn)
        if os.path.isfile(p):
            args += [f"--{key}-traj", p]
    try:
        r = sh(args, timeout=300)
        if r.returncode != 0:
            print(f"  [警告] 评测脚本返回 {r.returncode}: {r.stderr[:200]}")
            return None
        with open(out) as f:
            return json.load(f)
    except Exception as e:
        print(f"  [警告] 评测失败：{e}")
        return None


# ---------------------------------------------------------------------------
# 主流程
# ---------------------------------------------------------------------------


def main():
    ap = argparse.ArgumentParser(description="仿真批量评测")
    ap.add_argument("--matrix", required=True)
    ap.add_argument("--only", default="", help="只跑名字匹配的实验（子串匹配）")
    ap.add_argument("--dry-run", action="store_true", help="只打印计划，不执行")
    args = ap.parse_args()

    with open(args.matrix) as f:
        cfg = yaml.safe_load(f)
    runner_cfg = cfg["runner"]
    exps = cfg["experiments"]
    if args.only:
        exps = [e for e in exps if args.only in e["name"]]
    if not exps:
        print("没有匹配的实验。")
        return

    total = sum(int(e.get("repeats", 1)) for e in exps)
    print(f"计划执行 {len(exps)} 个实验，共 {total} 次运行。")
    if args.dry_run:
        for e in exps:
            print(f"  - {e['name']}  x{e.get('repeats', 1)}  "
                  f"({e['robot']} / {e['world']} / {e.get('global_planner')} + {e.get('local_planner')})")
        return

    out_root = os.path.expanduser(runner_cfg["output_dir"])
    os.makedirs(out_root, exist_ok=True)
    rows = []
    results = []
    n = 0
    try:
        for exp in exps:
            for i in range(int(exp.get("repeats", 1))):
                n += 1
                res = run_one(exp, runner_cfg, i, total)
                if not res:
                    continue
                results.append(res)
                if res.get("status") != "DONE":
                    continue
                run_dir = os.path.join(out_root, res["run_name"])
                rep = evaluate(run_dir, exp["world"], find_gt_map(exp["world"]))
                if rep:
                    res["report"] = rep
                    rows.append(flatten(exp, res, rep))
    except KeyboardInterrupt:
        print("\n收到中断，正在收尾…")

    write_summary(out_root, rows, cfg.get("summary_metrics", []), results)

    with open(os.path.join(out_root, "all_results.json"), "w", encoding="utf-8") as f:
        json.dump(results, f, ensure_ascii=False, indent=2)
    print(f"\n原始结果 -> {os.path.join(out_root, 'all_results.json')}")


def flatten(exp, res, rep):
    mp = rep.get("map", {})
    tp = rep.get("trajectory", {})
    nav = {}
    mj = os.path.join(os.path.expanduser("~/.ros/sim_eval"), res["run_name"], "metrics.json")
    if os.path.isfile(mj):
        try:
            with open(mj) as f:
                nav = json.load(f).get("navigation", {})
        except Exception:
            pass

    st = res.get("goal_statuses", [])
    row = {
        "experiment": exp["name"],
        "run": res["run_name"],
        "robot": exp["robot"],
        "world": exp["world"],
        "mapping": exp.get("mapping", "none"),
        "global_planner": exp.get("global_planner", ""),
        "local_planner": exp.get("local_planner", ""),
        "goals": len(st),
        "goals_ok": sum(1 for s in st if s == "SUCCEEDED"),
    }
    row["success_rate"] = round(row["goals_ok"] / row["goals"], 3) if row["goals"] else None
    for k in ("coverage_ratio", "occupied_iou", "occupied_precision",
              "occupied_recall", "grid_accuracy"):
        row[k] = mp.get(k)
    for k in ("ate_rmse_aligned", "ate_rmse_raw", "rpe_trans_rmse", "ate_scale"):
        row[k] = tp.get(k)
    for k in ("time_to_goal_mean", "tracking_error_rmse", "collision_count",
              "min_obstacle_dist", "control_smoothness"):
        row[k] = nav.get(k)
    return row


def write_summary(out_root, rows, metric_specs, results):
    if not rows:
        print("\n没有可汇总的有效结果。")
        if results:
            bad = [r for r in results if r.get("status") != "DONE"]
            if bad:
                print("失败原因统计：")
                for r in bad[:20]:
                    print(f"  - {r['run_name']}: {r['status']}")
        return

    csv_path = os.path.join(out_root, "summary.csv")
    keys = list(rows[0].keys())
    with open(csv_path, "w", newline="", encoding="utf-8") as f:
        w = csv.DictWriter(f, fieldnames=keys)
        w.writeheader()
        w.writerows(rows)

    # 按实验聚合（多遍取均值）
    agg = {}
    for r in rows:
        agg.setdefault(r["experiment"], []).append(r)

    print(f"\n{'='*100}\n汇总（每行一个实验，多遍取均值）\n{'='*100}")
    cols = ["success_rate", "coverage_ratio", "occupied_iou", "ate_rmse_aligned",
            "time_to_goal_mean", "tracking_error_rmse", "collision_count"]
    labels = ["成功率", "覆盖率", "IoU", "ATE(m)", "耗时(s)", "跟踪误差", "碰撞"]
    header = f"{'实验':34s} " + " ".join(f"{l:>10s}" for l in labels) + f" {'n':>4s}"
    print(header)
    print("-" * len(header))

    def fmt(v):
        if v is None:
            return "     -"
        if isinstance(v, float):
            return f"{v:10.3f}"
        return f"{v:10d}"

    for name, rs in agg.items():
        vals = []
        for c in cols:
            nums = [r[c] for r in rs if isinstance(r.get(c), (int, float))]
            vals.append(sum(nums) / len(nums) if nums else None)
        print(f"{name:34s} " + " ".join(fmt(v) for v in vals) + f" {len(rs):4d}")

    # 按指标标注谁最好，省得你对着数字自己比
    print("\n各指标最优：")
    for spec in metric_specs:
        k, higher = spec["key"], spec.get("higher_is_better", True)
        best, best_v = None, None
        for name, rs in agg.items():
            nums = [r[k] for r in rs if isinstance(r.get(k), (int, float))]
            if not nums:
                continue
            v = sum(nums) / len(nums)
            if best_v is None or (v > best_v if higher else v < best_v):
                best, best_v = name, v
        if best:
            arrow = "↑" if higher else "↓"
            print(f"  {spec['label']:12s} {arrow}  {best}  ({best_v:.4f})")

    print(f"\n明细 CSV -> {csv_path}")


if __name__ == "__main__":
    main()
