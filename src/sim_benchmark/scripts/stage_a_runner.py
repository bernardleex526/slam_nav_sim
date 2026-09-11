#!/usr/bin/env python3
"""
stage_a_runner
==============
阶段 A（建图）批量评测编排器。

方法论（这是整个平台最重要的一个设计决定）
------------------------------------------
不把"建图"和"导航"混在一次运行里测。原因见 README 第 24 条：

  SLAM 冷启动时全局地图基本是空的。此时若同时开 Nav2 并下发远处目标，
  全局规划器是在"盲规划"——它以为处处可通行，规划出穿墙路径；
  而局部代价地图用实时 /scan，看得见真障碍。两者认知不一致，
  控制器一跟就撞，成功率必然接近 0。

  这条错误路径会把两个变量耦合在一起：你以为是规划器不行，
  实际是"SLAM 还没探索完"。指标没有可比性。

正确做法是拆成两个独立阶段：

  阶段 A（本脚本）：navigation:=none + mapping:=<算法> + 割草机式探索
                   → 产出 N 张地图，各自对 GT 评建图指标
                   → 每张地图选出该环境下最好的一张，冻结为基准地图
  阶段 B（stage_b_runner）：localization:=amcl + map:=<冻结地图>
                   → 变量只剩规划器/控制器，指标才可比

每张环境选"最好地图"的判据
--------------------------
不用单一指标，用加权综合分（见 pick_best_map）：
  覆盖率 × 召回率 × IoU × 精确率 —— 四个都要好才算好地图。
只覆盖率高的地图可能是"扫得全但墙都是糊的"，只 IoU 高的可能是"只建了
一小块但那一块很准"。综合分强迫两者兼顾。

用法
----
    python3 stage_a_runner.py --config config/stage_a_matrix.yaml
    python3 stage_a_runner.py --config ... --only simple     # 只跑某环境
    python3 stage_a_runner.py --config ... --algo slam_toolbox
    python3 stage_a_runner.py --config ... --dry-run
"""

import argparse
import csv
import json
import math
import os
from pathlib import Path
import signal
import shutil
import subprocess
import sys
import time

try:
    import yaml
except ImportError:
    print("需要 pyyaml：pip install pyyaml", file=sys.stderr)
    sys.exit(2)


# ---------------------------------------------------------------------------
# 子进程辅助（与 benchmark_runner 同一套哲学：永不抛异常）
# ---------------------------------------------------------------------------


class _Result:
    def __init__(self, returncode=-1, stdout="", stderr=""):
        self.returncode = returncode
        self.stdout = stdout
        self.stderr = stderr


def sh(cmd, timeout=None):
    try:
        return subprocess.run(cmd, capture_output=True, text=True, timeout=timeout)
    except subprocess.TimeoutExpired:
        return _Result(-1, "", f"TIMEOUT({timeout}s): {' '.join(cmd)}")
    except FileNotFoundError as e:
        return _Result(-2, "", f"NOT FOUND: {e}")
    except OSError as e:
        return _Result(-3, "", f"OSError: {e}")


def prepare_outdir(outdir):
    """开跑前清掉上一轮的产物，避免"旧数据冒充新结果"。

    ⚠️ 为什么必须清（实测踩坑）：
      stage_a_runner 每次都把结果写进同一个 run_name 目录，但只在【跑完】时
      才覆写 map.pgm / report.json。若上一轮被杀在中途，目录里会留着上一轮的
      成品；本轮跑到一半时你去看目录：

          map.pgm   ← 时间戳是上一轮的（没在涨）
          exploration_traj.csv ← 时间戳也是上一轮的（没在涨）

      这会让人误判成"本轮结果已经有了"或"卡住了"，是最容易骗过眼睛的一种假象。
      所以开跑前先把旧产物挪走（不是删除 —— 万一需要回溯），只留 sim.log 等
      本轮新建的诊断文件。
    """
    if not os.path.isdir(outdir):
        os.makedirs(outdir, exist_ok=True)
        return []
    stale = ("map.pgm", "map.yaml", "report.json",
             "exploration_traj.csv", "exploration.log", "metrics_summary.json",
             "exploration.done", "run_result.json", "map.pgm.uncropped",
             "map_raw.yaml", "report_raw.json", "trajectory_gt.csv",
             "trajectory_est.csv", "metrics.json", "sim.log", "launch.log",
             "map_saver.log", "coverage_plan.json")
    moved = []
    stamp = str(time.time_ns())
    for fn in stale:
        p = os.path.join(outdir, fn)
        if os.path.isfile(p):
            bak = os.path.join(outdir, f"{fn}.prev-{stamp}")
            try:
                os.replace(p, bak)
                moved.append(fn)
            except OSError as exc:
                raise RuntimeError(f"Cannot archive {p}: {exc}") from exc
    return moved


def kill_group(proc):
    """杀掉整个进程组，并确认真的杀干净。

    ⚠️ 为什么不能只 killpg(SIGINT) 就走（实测踩坑）：
      ros2 launch 的子进程（gzserver / slam_toolbox / gimbal_stabilizer …）
      收到 SIGINT 后会各自处理退出，但【任何一个卡住不退】，它就会变成
      孤儿继续往话题里灌数据。下一轮实验起来后，DDS 图里就出现：

          /gimbal_stabilizer        ← ×3
          /points_to_scan           ← ×2
          /map  Publisher count: 2  ← 两个 slam_toolbox 抢发地图
          WARNING: nodes in the graph that share an exact name ...

      后果：三路云台 TF 抢发 → 雷达外参随机跳；两路 /scan 交错注入 →
      slam_toolbox 位姿被拉偏 → 【场外幽灵墙】+ 探索行为不可复现。

      这些"幽灵节点"不占 CPU、不报错，只安静地灌数据，从日志里完全看不出来。
      所以必须在 SIGINT 之后【再等一轮并复核】，仍有残留就 SIGKILL，
      最后再 SIGTERM/SIGKILL 兜底整个组。
    """
    if proc is None:
        return
    try:
        pgid = os.getpgid(proc.pid)
    except ProcessLookupError:
        pgid = proc.pid  # All callers use start_new_session=True.

    # 第一轮：SIGINT 给 launch 一个优雅退出的机会（它会转发给子节点）
    try:
        os.killpg(pgid, signal.SIGINT)
        proc.wait(timeout=15)
    except (ProcessLookupError, subprocess.TimeoutExpired):
        pass

    # 第二轮：复核进程组是否真的空了。用 killpg(pgid, 0) 探测组内是否还有活口。
    # （os.getpgid 只查 leader，leader 死了不代表组内没孤儿，所以用信号 0 探组。）
    for _ in range(3):
        try:
            os.killpg(pgid, 0)
        except ProcessLookupError:
            return  # 组已空
        except PermissionError:
            pass
        try:
            os.killpg(pgid, signal.SIGKILL)
        except ProcessLookupError:
            return
        time.sleep(1.0)

    # 第三轮：仍不死心就单独清点一次组成员（ps 按 pgid 过滤），逐个 -9
    try:
        out = subprocess.run(["ps", "-o", "pid=", "-g", str(pgid)],
                             capture_output=True, text=True, timeout=5).stdout
        for line in out.split():
            try:
                os.kill(int(line), signal.SIGKILL)
            except (ProcessLookupError, ValueError, PermissionError):
                pass
    except Exception:
        pass


def _share(pkg):
    r = sh(["ros2", "pkg", "prefix", pkg], timeout=10)
    if r.returncode != 0:
        return None
    return os.path.join(r.stdout.strip(), "share", pkg)


# ---------------------------------------------------------------------------
# 单次阶段A实验
# ---------------------------------------------------------------------------


def build_sim_cmd(exp, cfg, run_name):
    """组装【只建图、不开导航】的 launch 命令。

    关键差异（相对导航评测）：
      navigation:=none     不启动 Nav2，也就没有全局规划器盲规划的问题
      localization 不传    stage A 根本不需要 map->odom 之外的定位
      metrics:=true        仍要采轨迹，用于算 ATE（建图算法自己的定位精度）
    """
    cmd = [
        "ros2", "launch", "sim_bringup", "sim.launch.py",
        f"robot:={exp['robot']}",
        f"world:={exp['world']}",
        f"mapping:={exp['mapping']}",
        "navigation:=none",
        f"odom_source:={exp.get('odom_source', 'gt')}",
        f"use_gimbal:={str(exp.get('use_gimbal', True)).lower()}",
        f"headless:={str(cfg.get('headless', True)).lower()}",
        "metrics:=true",
        f"run_name:={run_name}",
        f"metrics_output_dir:={os.path.expanduser(cfg.get('output_dir', '~/.ros/sim_stage_a'))}",
    ]
    if cfg.get("headless", True):
        cmd.append("viz:=false")
    if exp.get("sensor_profile"):
        cmd.append(f"sensor_profile:={exp['sensor_profile']}")
    return cmd


def exploration_complete(path):
    try:
        return Path(path).read_text(encoding="utf-8").splitlines()[0] == "done"
    except (OSError, IndexError):
        return False


def check_sim_failure(sim, log_path):
    if sim is not None and sim.poll() is not None:
        raise RuntimeError(f"Simulation exited: {sim.returncode}")
    try:
        text = Path(log_path).read_text(encoding="utf-8", errors="replace")
    except OSError:
        return
    for line in text.splitlines():
        if ("FATAL ERROR" in line or "process has died" in line or "[FATAL]" in line
                or "Failed to bring up all requested nodes" in line):
            raise RuntimeError(line)


class MappingMonitor:
    """Observe actual messages without the ros2 CLI daemon/cache."""
    def __init__(self):
        import rclpy
        from rclpy.node import Node
        from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
        from nav_msgs.msg import OccupancyGrid
        from sensor_msgs.msg import LaserScan
        self.ros = rclpy
        self.owns_context = not rclpy.ok()
        if self.owns_context:
            rclpy.init()
        self.node = Node("stage_a_health")
        self.last_scan = self.last_map = None
        self.scan_stamp = None
        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.node.create_subscription(LaserScan, "/scan", self._scan, qos)
        qos_map = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.node.create_subscription(OccupancyGrid, "/map", self._map, qos_map)

    def _scan(self, msg):
        stamp = (msg.header.stamp.sec, msg.header.stamp.nanosec)
        if stamp != self.scan_stamp and len(msg.ranges):
            self.last_scan = time.monotonic()
            self.scan_stamp = stamp

    def _map(self, msg):
        if msg.info.width and msg.info.height and any(value >= 0 for value in msg.data):
            self.last_map = time.monotonic()

    def ready(self, timeout, sim, log_path):
        started = time.monotonic()
        while time.monotonic() - started < timeout:
            check_sim_failure(sim, log_path)
            self.ros.spin_once(self.node, timeout_sec=.1)
            if self.last_scan is not None and self.last_map is not None:
                return True, time.monotonic() - started
        return False, time.monotonic() - started

    def check(self):
        for _ in range(4):
            self.ros.spin_once(self.node, timeout_sec=0.)
        now = time.monotonic()
        if self.last_scan is None or now-self.last_scan > 30:
            raise RuntimeError("No fresh /scan for 30 wall seconds")
        if self.last_map is None or now-self.last_map > 90:
            raise RuntimeError("No valid /map for 90 wall seconds")

    def close(self):
        self.node.destroy_node()
        if self.owns_context and self.ros.ok():
            self.ros.shutdown()


def run_exploration(runner_cfg, exp, run_name, log_path, sim=None, sim_log=None, monitor=None):
    """跑探索 runner，等它完成或超时。

    完成判据用【哨兵文件】而不是订阅 done 话题，理由：
      `ros2 topic echo --once` 在轮询里非常不可靠 —— 它本身要起一个节点，
      每次 8~15s，而且 QoS 不匹配时会静默收不到（实测就卡在这里，把一次
      完整的探索误判成"10 秒超时"）。文件哨兵是进程间最稳的信号：
      文件出现 = 真的跑完了，没有中间层。
    """
    out_dir = os.path.expanduser(runner_cfg["output_dir"])
    run_dir = os.path.join(out_dir, run_name)
    traj_dir = run_dir
    sentinel = os.path.join(run_dir, "exploration.done")
    if os.path.exists(sentinel):
        os.remove(sentinel)

    cmd = [
        "ros2", "run", "sim_benchmark", "exploration_runner.py",
        "--ros-args",
        # ⚠️ 参数必须用 `-p name:=value` 形式（带冒号等号）。
        # 写成 `-p name=value` 会让 rcl 直接抛
        #   "Couldn't parse parameter override rule: '-p world=simple'"
        # 节点【一行都不跑】就退出，而外层只看到"探索超时"，
        # 完全指不到真正原因 —— 实测踩过一次。
        # ⚠️ 浮点参数必须传浮点字面量。
        # `-p max_duration:=2400` 会被 rcl 解析成 INTEGER，而节点声明的是
        # 1800.0（DOUBLE），于是直接抛
        #   InvalidParameterTypeException: Trying to set parameter
        #   'max_duration' to '2400' of type 'INTEGER', expecting type 'DOUBLE'
        # 节点一秒都不跑就退出。所以这里统一用 :.1f 格式化，保证带小数点。
        "-p", f"world:={exp['world']}",
        "-p", f"lane_spacing:={float(runner_cfg.get('lane_spacing', 1.2)):.3f}",
        "-p", f"speed:={float(runner_cfg.get('explore_speed', 0.5)):.3f}",
        "-p", f"clearance:={float(runner_cfg.get('clearance', 0.45)):.3f}",
        "-p", f"max_duration:={float(runner_cfg.get('max_explore_duration', 2400)):.1f}",
        "-p", f"traj_dir:={traj_dir}",
    ]
    log_fh = open(log_path, "w", encoding="utf-8", buffering=1)
    proc = subprocess.Popen(cmd, stdout=log_fh, stderr=subprocess.STDOUT,
                            text=True, start_new_session=True)

    timeout = float(runner_cfg.get("max_explore_duration", 2400)) + 180.0
    t0 = time.time()
    done = False
    last_report = 0.0
    traj_csv = os.path.join(traj_dir, "exploration_traj.csv")
    try:
        while time.time() - t0 < timeout:
            if sim_log:
                check_sim_failure(sim, sim_log)
            if monitor:
                monitor.check()
            if os.path.exists(sentinel):
                done = exploration_complete(sentinel)
                break
            if proc.poll() is not None:
                done = exploration_complete(sentinel)
                break
            el = time.time() - t0
            if el - last_report > 60:
                last_report = el
                print(f"    [exploring {el/60:.1f} min]{traj_brief(traj_csv)}", flush=True)
            time.sleep(2.0)
    finally:
        kill_group(proc)
        log_fh.close()
    return done, time.time() - t0


def check_map_publisher(timeout=8.0):
    """存图前体检 /map：返回 (是否有发布者, 详情字符串)。

    为什么要在存图前单独查一次：
      map_saver_cli 在 /map 无 publisher 时会一直卡到 save_map_timeout，
      而它的输出里只有 "map_saver lifecycle node launched" 就再无下文，
      完全区分不出"slam 没出图"还是"存图工具坏了"。
      先自检一遍，诊断成本从"再跑一轮 20 分钟"降到 3 秒。
    """
    r = sh(["ros2", "topic", "info", "-v", "/map"], timeout=timeout)
    out = r.stdout or ""
    detail = ""
    for line in out.splitlines():
        s = line.strip()
        if "Publisher count" in s or "Subscription count" in s:
            detail += s + "  "
    has = ("Publisher count:" in out) and ("Publisher count: 0" not in out)
    return has, detail.strip()


def traj_brief(csv_path):
    """读探索轨迹 CSV，返回一行摘要（点数 + 走过的路程 + 覆盖范围）。

    失败一律返回空串 —— 这只用于进度显示，绝不能因为它出错而影响实验。
    """
    try:
        if not os.path.isfile(csv_path):
            return "  （尚无轨迹）"
        n = 0
        dist = 0.0
        xs, ys = [], []
        px = py = None
        with open(csv_path, encoding="utf-8") as f:
            for i, line in enumerate(f):
                if i == 0:
                    continue
                parts = line.strip().split(",")
                if len(parts) < 3:
                    continue
                try:
                    x, y = float(parts[1]), float(parts[2])
                except ValueError:
                    continue
                n += 1
                xs.append(x)
                ys.append(y)
                if px is not None:
                    dist += ((x - px) ** 2 + (y - py) ** 2) ** 0.5
                px, py = x, y
        if n == 0:
            return "  （尚无轨迹）"
        return (f"  轨迹 {n} 点 路程 {dist:.1f}m  "
                f"x[{min(xs):.1f},{max(xs):.1f}] y[{min(ys):.1f},{max(ys):.1f}]")
    except Exception:
        return ""


def run_one(exp, cfg, idx, total):
    runner_cfg = cfg["runner"]
    run_name = f"A_{exp['name']}"
    out_dir = os.path.expanduser(runner_cfg["output_dir"])
    run_dir = os.path.join(out_dir, run_name)
    os.makedirs(run_dir, exist_ok=True)

    # 开跑前把上一轮的产物挪走，避免"旧数据冒充新结果"（见 prepare_outdir 注释）
    moved = prepare_outdir(run_dir)
    if moved:
        print(f"  已归档上一轮产物：{', '.join(moved)}", flush=True)

    print(f"\n{'='*72}\n[{run_name}]  ({idx+1}/{total})  环境={exp['world']}  算法={exp['mapping']}\n{'='*72}",
          flush=True)

    sim_cmd = build_sim_cmd(exp, runner_cfg, run_name)
    print("  " + " ".join(sim_cmd), flush=True)
    sim_log = os.path.join(run_dir, "sim.log")

    log_fh = open(sim_log, "w", encoding="utf-8", buffering=1)
    sim = subprocess.Popen(sim_cmd, stdout=log_fh, stderr=subprocess.STDOUT,
                           text=True, start_new_session=True)

    result = {"run_name": run_name, "experiment": exp["name"], "world": exp["world"],
              "mapping": exp["mapping"], "status": "INCOMPLETE"}

    monitor = None
    try:
        # ---- 等仿真与建图算法起来 ----
        monitor = MappingMonitor()
        ok, waited = monitor.ready(float(runner_cfg.get("startup_timeout", 120)), sim, sim_log)
        print(f"  仿真启动{'就绪' if ok else '超时'}（{waited:.1f}s）", flush=True)
        if not ok:
            result["status"] = "STARTUP_TIMEOUT"
            return result

        # 多等一会儿让 slam_toolbox / 点云稳定
        time.sleep(float(runner_cfg.get("warmup", 8.0)))

        # ---- 跑探索 ----
        exp_log = os.path.join(run_dir, "exploration.log")
        check_sim_failure(sim, sim_log)
        done, elapsed = run_exploration(runner_cfg, exp, run_name, exp_log, sim, sim_log, monitor)
        print(f"  探索{'完成' if done else '超时'}（{elapsed:.0f}s）", flush=True)
        result["exploration_done"] = done
        result["exploration_seconds"] = round(elapsed, 1)

        # 收尾前给 slam_toolbox 一点时间把最后一帧并进图
        time.sleep(3.0)

        # ---- 存图 ----
        # 存图前先体检 /map：publisher 为 0 时 map_saver_cli 会卡在
        # lifecycle activate，而且报错文案完全指不到真正原因（实测踩过，
        # 只看得见 "map_saver lifecycle node launched" 就再无下文）。
        # 先自检一遍，能立刻区分"slam 没出图"和"存图工具坏了"。
        monitor.check()
        has_pub = monitor.last_map is not None
        pub_detail = "Valid /map received by persistent health subscription"
        print(f"  存图前 /map 体检：{pub_detail or '（无法判定）'}", flush=True)
        if not has_pub:
            print("  [警告] /map 无发布者 —— slam 未产出地图，存图大概率超时",
                  flush=True)

        map_out = os.path.join(run_dir, "map")
        saver_log = ""

        # ---- 首选：slam_toolbox 自带的 SaveMap 服务 ----
        # 为什么它比 map_saver_cli 可靠（实测结论）：
        #   map_saver_cli 靠【订阅 /map 话题】拿地图，而 /map 是 transient_local，
        #   订阅时机一旦错开就永远收不到，然后一路卡到 save_map_timeout，
        #   输出里只有一行 "map_saver lifecycle node launched" 再无下文 ——
        #   看不出是 slam 没出图，还是存图工具坏了。
        #   SaveMap 服务直接序列化 slam_toolbox 内部的地图对象，没有这个
        #   时序问题，实测可用时 1 秒内返回 result=0 并落盘 ~1.9MB 的 pgm。
        r = sh(["ros2", "service", "call", "/slam_toolbox/save_map",
                "slam_toolbox/srv/SaveMap",
                "{name: {data: '%s'}}" % map_out], timeout=60)
        saver_log += "[SaveMap 服务]\n" + (r.stdout or "") + (r.stderr or "")
        saved = valid_map(map_out + ".yaml")
        if saved:
            print(f"  地图已保存（SaveMap 服务）-> {map_out}.yaml", flush=True)

        # ---- 兜底：map_saver_cli ----
        # 服务不可用（节点名被 remap、或 slam 侧改名）时退回老办法，
        # 至少不会因为"换了个新接口"就把已经跑完的实验整轮作废。
        if not saved:
            r = sh(["ros2", "run", "nav2_map_server", "map_saver_cli",
                    "-f", map_out, "--ros-args", "-p", "save_map_timeout:=20.0"],
                   timeout=90)
            saver_log += ("\n[map_saver_cli 兜底]\n"
                          + (r.stdout or "") + (r.stderr or ""))
            saved = valid_map(map_out + ".yaml")
            if saved:
                print(f"  地图已保存（map_saver_cli 兜底）-> {map_out}.yaml",
                      flush=True)

        # 两条路都失败时，必须把完整输出留档 —— 截断会把真正的原因藏起来，
        # 而重跑一轮的代价是 20 分钟。
        if not saved:
            sl_path = os.path.join(run_dir, "map_saver.log")
            try:
                with open(sl_path, "w", encoding="utf-8") as mfh:
                    mfh.write(saver_log)
            except Exception:
                sl_path = "（留档失败）"
            print(f"  [警告] 存图失败（两条路都没出图）。"
                  f"完整输出已留档：{sl_path}\n{saver_log[:2000]}", flush=True)
        else:
            result["map_yaml"] = map_out + ".yaml"
            shutil.copy2(map_out + ".pgm", map_out + ".pgm.uncropped")
            with open(map_out + ".yaml", encoding="utf-8") as stream:
                raw_meta = yaml.safe_load(stream)
            raw_meta["image"] = "map.pgm.uncropped"
            atomic_write(Path(run_dir) / "map_raw.yaml", yaml.safe_dump(raw_meta))
            result["raw_map_yaml"] = str(Path(run_dir) / "map_raw.yaml")
            result["postprocessing"] = "trajectory_bbox_heuristic; does not fix SLAM drift"
            # ---- 存图后按轨迹范围裁剪 ----
            # 为什么必须做：slam_toolbox 的 scan matching 偶发失配时，个别
            # 节点会被拉到场外几十米，再从错误位置把整束射线画出来，形成
            # 放射状幽灵墙（simple 实测：88% 占据格落在 20x20m 场外，
            # ghost_wall_ratio 0.88 —— 这种图直接交给 Nav2，全局代价地图
            # 会被虚假障碍堵死可通行区）。
            # 裁剪只清【轨迹从未到过的区域】（信息全部回退为 unknown），
            # 轨迹包围盒内一格不动 —— 是与事实一致的客观后处理，不是美化。
            # 实测同一张图：0.88 -> 0.042，综合分 0.10 -> 0.34。
            crop_script = str(Path(__file__).resolve().parents[3] / "tools/crop_map_to_traj.py")
            if not runner_cfg.get("crop_map", True):
                result["postprocessing"] = "none"
            elif os.path.isfile(crop_script):
                cr = sh(["python3", crop_script, run_dir], timeout=120)
                out = ((cr.stdout or "") + (cr.stderr or "")).strip()
                print("  " + out.replace("\n", "\n  "), flush=True)
                if cr.returncode:
                    raise RuntimeError("Map postprocessing failed: " + out)
            else:
                print("  [警告] 找不到 tools/crop_map_to_traj.py，跳过裁剪"
                      "（请在 sim_ws 根目录运行本脚本）", flush=True)
        result["status"] = ("MAP_SAVE_FAILED" if not saved else
                            "DONE" if done else "EXPLORATION_INCOMPLETE")

    except KeyboardInterrupt:
        result["status"] = "INTERRUPTED"
        raise
    except Exception as e:
        result["status"] = f"ERROR:{type(e).__name__}"
        result["error"] = str(e)
        print(f"  [错误] {e}", flush=True)
    finally:
        kill_group(sim)
        if monitor:
            monitor.close()
        log_fh.close()
        atomic_write(Path(run_dir) / "run_result.json", json.dumps(result, indent=2))
        time.sleep(float(runner_cfg.get("cooldown", 8)))

    return result


def wait_for_topic(topic, timeout, settle=40.0):
    """等某话题有发布者。

    为什么用 `ros2 topic info -v` 而不是 `topic list`：
      list 只说明话题被声明过，info -v 能看 publisher 数量。
    为什么不在超时后直接放弃：
      检测手段本身可能因 daemon 缓存假阴性（踩过），所以 settle 秒后
      按"就绪"继续，让真正的失败在后续步骤暴露。
    """
    t0 = time.time()
    while time.time() - t0 < timeout:
        r = sh(["ros2", "topic", "info", "-v", topic], timeout=12)
        out = r.stdout or ""
        if "Publisher count: 0" not in out and "Publisher count:" in out:
            return True, time.time() - t0
        # 退化方案：能 echo 到一帧也算就绪
        r2 = sh(["ros2", "topic", "echo", "--once", topic], timeout=6)
        if (r2.stdout or "").strip():
            return True, time.time() - t0
        time.sleep(2.5)
    return False, time.time() - t0


# ---------------------------------------------------------------------------
# 评测与选图
# ---------------------------------------------------------------------------


def evaluate(run_dir, world, gt_yaml, raw=False):
    s = _share("sim_metrics")
    if not s:
        return None
    prefix = os.path.dirname(os.path.dirname(s))
    candidates = [
        os.path.join(prefix, "lib", "sim_metrics", "evaluate_vs_gt.py"),
        os.path.join(s, "scripts", "evaluate_vs_gt.py"),
    ]
    script = next((c for c in candidates if os.path.isfile(c)), None)
    if script is None:
        print(f"  [警告] 找不到 evaluate_vs_gt.py")
        return None

    est_map = os.path.join(run_dir, "map_raw.yaml" if raw else "map.yaml")
    if not os.path.isfile(est_map):
        return None
    out = os.path.join(run_dir, "report_raw.json" if raw else "report.json")
    args = [sys.executable, script, "--gt-map", gt_yaml, "--est-map", est_map,
            "--out", out, "--quiet"]
    # 建图阶段也有轨迹（GT vs SLAM 估计），用于 ATE
    for key, fn in (("gt", "trajectory_gt.csv"), ("est", "trajectory_est.csv")):
        p = os.path.join(run_dir, fn)
        if not os.path.isfile(p) and not os.path.isfile(os.path.join(run_dir, "run_result.json")):
            p = os.path.expanduser(os.path.join("~/.ros/sim_eval", os.path.basename(run_dir), fn))
        if os.path.isfile(p):
            args += [f"--{key}-traj", p]
    r = sh(args, timeout=300)
    if r.returncode != 0:
        print(f"  [警告] 评测返回 {r.returncode}: {(r.stderr or '')[:150]}")
        return None
    try:
        with open(out) as f:
            return json.load(f)
    except Exception:
        return None


def map_score(mp):
    """综合建图质量分（0~1）。子指标几何平均，缺一即大幅拉低总分。

    为什么用几何平均而不是算术平均：
      算术平均下，"覆盖率 100% + 召回率 4%" 还能拿到 0.52 的"及格分"，
      会误导选图。几何平均里只要一个接近 0，总分就被拽到接近 0 ——
      这正是我们要的：四个指标必须【同时】好。

    ⚠️ 口径修正（本项目实测踩坑）：
      裸的 occupied_recall / occupied_precision 会系统性低估 SLAM 质量，
      原因不是算法差，而是【度量口径与物理表示不匹配】：
        ① 墙厚差异：GT 是实心体块（内墙 0.3m、柱子 0.5x0.5m 实心），
           SLAM 输出的是"表面细线"（激光看不到墙内部）。
           simple 实测 EST 墙实心度 4.4%，GT 墙实心度 45.2%，差 10 倍。
        ② 逐格比：GT 0.1m/格 vs EST 0.05m/格 对齐到 0.05m 后，
           一堵 0.3m 墙在 GT 占 6 格，EST 细线只占 2 格 → recall 天然 ~33%。
        ③ 实证：simple 裸 recall=0.240，但
             · EST 墙到最近 GT 墙的【中位距离只有 0.05m】（= 1 个栅格）
             · 允许 5cm 误差 → recall 0.528
             · 允许 10cm 误差 → recall 0.730
             · EST 墙加粗 1 格 → recall 0.528（+28.7pt）
           说明几何是准的，"低召回"是度量假象。

      所以选图时优先用容差口径 recall_tol2 / precision_tol2（允许 10cm），
      它反映"几何对不对"；裸值保留在报告里做版本间回归对比。
      若旧报告没有容差字段（scipy 未装 / 老版本产出），自动回退到裸值。

    ⚠️ 还必须乘一个【幽灵墙惩罚因子】(1 - ghost_wall_ratio)：
      上面三个指标都看不到"场外虚假占据格"，不惩罚的话会把一张
      "84% 墙画在场外"的废图评为最优（实测真发生过）。详见下方注释。
    """
    # 幽灵墙惩罚因子：占据格落在 GT 范围外的比例。
    #
    # 为什么必须进评分（本项目实测踩坑）：
    #   coverage / recall_tol2 / precision_tol2 这三个指标【全都看不到幽灵墙】——
    #   precision 的分母只统计 GT 已知格，场外的虚假占据格既不算命中也不算误报。
    #   于是出现过这种荒谬情况：一张地图 84% 的占据格画在场地外，
    #   评分却依然是"最优"（0.846）。
    #   而这张图交给 Nav2 会直接堵死全局代价地图 → 导航必然失败。
    #
    # 惩罚曲线（实测锚点）：
    #   ghost=0.00 → 因子 1.00（无惩罚）
    #   ghost=0.10 → 因子 0.90
    #   ghost=0.30 → 因子 0.70（开始明显不可用）
    #   ghost=0.60 → 因子 0.40
    #   ghost=1.00 → 因子 0.00
    # 即 factor = 1 - ghost_ratio，线性惩罚。宁可保守：地图干净比指标好看重要。
    ghost_pen = 1.0
    g = mp.get("ghost_wall_ratio")
    if isinstance(g, (int, float)):
        ghost_pen = max(0.0, 1.0 - min(1.0, float(g)))

    # 优先用容差口径：tolerance_metrics 里 recall_tol2 = 允许 10cm 定位误差
    if mp.get("recall_tol2") is not None and mp.get("precision_tol2") is not None:
        keys = ["coverage_ratio", "recall_tol2", "precision_tol2"]
        vals = [mp.get(k) for k in keys]
        # tol2 的 IoU 用两档容差指标的几何平均近似（真实 IoU 未在容差下重算）
        if all(v is not None for v in vals):
            prod = 1.0
            for v in vals:
                if not isinstance(v, (int, float)) or not math.isfinite(v):
                    return None
                prod *= max(0.0, min(1.0, float(v)))
            return (prod ** (1.0 / len(vals))) * ghost_pen

    # 回退：老口径
    keys = ["coverage_ratio", "occupied_recall", "occupied_iou", "occupied_precision"]
    vals = [mp.get(k) for k in keys]
    if any(v is None for v in vals):
        return None
    prod = 1.0
    for v in vals:
        if not isinstance(v, (int, float)) or not math.isfinite(v):
            return None
        prod *= max(0.0, min(1.0, float(v)))
    return (prod ** (1.0 / len(vals))) * ghost_pen


def pick_best_map(rows):
    """按环境分组，各选综合分最高的一张地图。返回 {world: row}。"""
    best = {}
    for r in rows:
        w = r["world"]
        sc = r.get("map_score")
        ghost = r.get("ghost_wall_ratio")
        if (sc is None or not math.isfinite(sc) or sc <= 0 or
                not isinstance(ghost, (int, float)) or not 0 <= ghost < 0.3 or
                r.get("status", "DONE") != "DONE"):
            continue
        if w not in best or sc > best[w]["map_score"]:
            best[w] = r
    return best


# ---------------------------------------------------------------------------
# 主流程
# ---------------------------------------------------------------------------


def main():
    ap = argparse.ArgumentParser(description="阶段A：建图评测与选图")
    ap.add_argument("--config", required=True)
    ap.add_argument("--only", default="", help="只跑 world 含该子串的实验")
    ap.add_argument("--algo", default="", help="只跑 mapping 等于该值的实验")
    ap.add_argument("--experiment", default="", help="Run one exact experiment name")
    ap.add_argument("--dry-run", action="store_true")
    args = ap.parse_args()

    with open(args.config) as f:
        cfg = yaml.safe_load(f)
    runner_cfg = cfg["runner"]
    exps = cfg["experiments"]
    if args.only:
        exps = [e for e in exps if args.only in e["world"]]
    if args.algo:
        exps = [e for e in exps if e["mapping"] == args.algo]
    if args.experiment:
        exps = [e for e in exps if e["name"] == args.experiment]
    if not exps:
        print("没有匹配的实验。")
        return

    print(f"阶段 A：{len(exps)} 个实验")
    if args.dry_run:
        for e in exps:
            print(f"  - {e['name']:24s} world={e['world']:8s} mapping={e['mapping']}")
        return

    out_root = os.path.expanduser(runner_cfg["output_dir"])
    os.makedirs(out_root, exist_ok=True)
    rows, results = [], []

    try:
        for i, exp in enumerate(exps):
            res = run_one(exp, cfg, i, len(exps))
            results.append(res)
            if not res.get("map_yaml"):
                continue
            run_dir = os.path.join(out_root, res["run_name"])
            gt = os.path.join(_share("sim_worlds"), "maps", f"{exp['world']}.yaml")
            rep = evaluate(run_dir, exp["world"], gt)
            if res.get("raw_map_yaml"):
                evaluate(run_dir, exp["world"], gt, raw=True)
            if not rep:
                continue
            res["report"] = rep
            row = flatten(exp, res, rep)
            rows.append(row)
            print(f"    覆盖率={row['coverage_ratio']}  召回={row['occupied_recall']}  "
                  f"IoU={row['occupied_iou']}  综合分={row['map_score']}", flush=True)
    except KeyboardInterrupt:
        print("\n收到中断，收尾中…")

    # ---- 汇总 ----
    rows, results = resummarize(out_root, cfg)
    out_json = os.path.join(out_root, "stage_a_results.json")

    if rows:
        csv_path = os.path.join(out_root, "stage_a_summary.csv")

        print(f"\n{'='*92}\n阶段 A 汇总\n{'='*92}")
        hdr = (f"{'实验':26s} {'环境':8s} {'算法':14s} "
               + " ".join(f"{l:>9s}" for l in
                          ["覆盖率", "召回率", "IoU", "精确率", "综合分", "ATE(m)"]))
        print(hdr)
        print("-" * len(hdr))
        for r in rows:
            print(f"{r['experiment']:26s} {r['world']:8s} {r['mapping']:14s} "
                  + " ".join(f"{(('%.3f' % v) if isinstance(v, float) else '-'):>9s}"
                             for v in [r.get("coverage_ratio"), r.get("occupied_recall"),
                                       r.get("occupied_iou"), r.get("occupied_precision"),
                                       r.get("map_score"), r.get("ate_rmse_aligned")]))

        best = pick_best_map(rows)
        print(f"\n{'='*92}\n各环境选出的最优地图（供阶段 B 使用）\n{'='*92}")
        for world, r in sorted(best.items()):
            print(f"  {world:8s} -> {r['experiment']:24s} (综合分 {r['map_score']:.3f})")
            # 防御：正常情况下 row 里一定有 run_name；但汇总阶段是整个实验的
            # 【最后一公里】，任何 KeyError 都会让辛苦跑出来的选图结果全部丢失。
            # 退回到约定命名总比崩掉强 —— 踩过一次（汇总崩 → 三份产物全作废）。
            rn = r.get("run_name") or f"A_{r['experiment']}"
            print(f"             map: {os.path.join(out_root, rn, 'map.yaml')}")

        sel_path = os.path.join(out_root, "stage_a_selected.yaml")
        print(f"\n选图结果 -> {sel_path}")
        print(f"明细 CSV -> {csv_path}")
    else:
        print("\n没有可汇总的有效结果。")
        for r in results:
            if r.get("status") != "DONE":
                print(f"  - {r['run_name']}: {r['status']}")

    print(f"\n原始结果 -> {out_json}")
    return 0 if all(r.get("status") == "DONE" for r in results
                    if r.get("experiment") in {e["name"] for e in exps}) else 1


def flatten(exp, res, rep):
    mp = rep.get("map", {})
    tp = rep.get("trajectory", {})
    row = {
        "experiment": exp["name"],
        "run_name": res["run_name"],
        "world": exp["world"],
        "mapping": exp["mapping"],
        "robot": exp["robot"],
    }
    for k in ("coverage_ratio", "occupied_iou", "occupied_precision",
              "occupied_recall", "grid_accuracy", "false_wall_cells", "missed_wall_cells"):
        row[k] = mp.get(k)
    # 容差口径指标：选图主依据（见 map_score 的说明）。
    # 裸 recall 0.240 时容差口径是 0.730 —— 后者才反映几何是否准确。
    for k in ("recall_tol1", "precision_tol1", "recall_tol2", "precision_tol2",
              "tol1_meters", "tol2_meters", "est_thick_ratio", "gt_thick_ratio",
              "est_to_gt_median_dist_m", "est_to_gt_far_ratio", "recall_after_thickening"):
        row[k] = mp.get(k)
    # 幽灵墙指标：占据格落在 GT 范围之外的比例。
    # 这是判断"地图能否直接给 Nav2 用"的硬门槛 —— 场外虚假障碍会堵死可通行区域。
    # 曾因为没输出这个字段，导致"64% 的墙画在场外"长期没被发现
    # （precision/recall 只统计 GT 已知格，场外格既不算命中也不算误报）。
    for k in ("est_occupied_total", "est_occupied_outside_gt", "ghost_wall_ratio"):
        row[k] = mp.get(k)
    for k in ("ate_rmse_aligned", "ate_rmse_raw", "rpe_trans_rmse"):
        row[k] = tp.get(k)
    score = map_score(mp)
    row["map_score"] = round(score, 4) if score is not None else None
    return row


def valid_map(path):
    try:
        path = Path(path)
        meta = yaml.safe_load(path.read_text(encoding="utf-8"))
        image = path.parent / meta["image"]
        return image.is_file() and image.stat().st_size > 0 and float(meta["resolution"]) > 0
    except (OSError, KeyError, TypeError, ValueError, yaml.YAMLError):
        return False


def atomic_write(path, text):
    path = Path(path)
    temp = path.with_name(path.name + f".tmp-{os.getpid()}")
    temp.write_text(text, encoding="utf-8")
    os.replace(temp, path)


def resummarize(out_root, cfg):
    """Rebuild from on-disk reports, never from only the current filtered batch."""
    root = Path(out_root).expanduser().resolve()
    root.mkdir(parents=True, exist_ok=True)
    import fcntl
    with (root / ".summary.lock").open("a") as lock:
        fcntl.flock(lock, fcntl.LOCK_EX)
        return _resummarize_locked(root, cfg)


def _resummarize_locked(root, cfg):
    experiments = {"A_" + e["name"]: e for e in cfg["experiments"]}
    rows, results = [], []
    for run in sorted(root.glob("A_*")):
        if not run.is_dir():
            continue
        exp = experiments.get(run.name)
        if exp is None:
            continue
        result = {"run_name": run.name, "experiment": exp["name"], "world": exp["world"],
                  "mapping": exp["mapping"], "status": "INCOMPLETE"}
        try:
            manifest = run / "run_result.json"
            if manifest.exists():
                result.update(json.loads(manifest.read_text(encoding="utf-8")))
            elif exploration_complete(run / "exploration.done"):
                result["status"] = "DONE"
            if result["status"] == "DONE" and not exploration_complete(run / "exploration.done"):
                result["status"] = "EXPLORATION_INCOMPLETE"
            report_path = run / "report.json"
            if report_path.exists() and valid_map(run / "map.yaml"):
                rep = json.loads(report_path.read_text(encoding="utf-8"))
                result["report"] = rep
                row = flatten(exp, result, rep)
                row["status"] = result["status"]
                rows.append(row)
        except (OSError, ValueError, TypeError, KeyError) as exc:
            result.update(status="INVALID_ARTIFACT", error=str(exc))
        results.append(result)
    atomic_write(root / "stage_a_results.json", json.dumps(
        {"results": results, "rows": rows}, ensure_ascii=False, indent=2))
    import io
    buf = io.StringIO()
    if rows:
        writer = csv.DictWriter(buf, fieldnames=list(rows[0]))
        writer.writeheader()
        writer.writerows(rows)
    atomic_write(root / "stage_a_summary.csv", buf.getvalue())
    selected = {}
    for world, row in pick_best_map(rows).items():
        selected[world] = dict(row, map_yaml=str(root / row["run_name"] / "map.yaml"))
    atomic_write(root / "stage_a_selected.yaml", yaml.safe_dump(
        {"selected": selected}, allow_unicode=True, sort_keys=False))
    return rows, results


if __name__ == "__main__":
    sys.exit(main())
