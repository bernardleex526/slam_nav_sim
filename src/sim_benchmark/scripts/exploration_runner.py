#!/usr/bin/env python3
"""
exploration_runner
==================
阶段 A（建图）的驱动节点：割草机式（boustrophedon）全覆盖探索。

为什么阶段 A 不能用 Nav2 驱动
------------------------------
1. 循环依赖：Nav2 需要一张地图才能规划，而阶段 A 的目的恰恰是产出那张地图。
2. 割草机式覆盖本质上是「几条平行直线 + 转弯」，不需要避障规划。用
   Nav2 反而引入了一堆无关变量（全局规划器、代价地图、恢复行为），
   最后测出来的差异不知道是建图算法的功劳还是规划器的功劳。
3. 脚本直接发 /cmd_vel 最简单可靠，且天然可复现 —— 同一张地图每次跑的
   轨迹完全一致，建图算法的对比才干净。

于是本节点的定位是：**给定地图边界，生成一条确定的覆盖轨迹，闭环跟踪它**。

为什么要闭环（读 /odom）而不是开环按时间推
--------------------------------------------
仿真里轮子会打滑、加速度有惯性、dt 抖动。开环按"走 5 秒就该到下一行"
的方式发指令，误差会累积，几行之后轨迹就歪了，出现大片漏扫。
所以这里用一个极简的追踪器：看当前位姿 → 算目标航向 → 转正 → 直走 → 到点。

跟踪器故意做得"笨"（纯 P 控制 + 分段），因为它的目标不是跑得漂亮，
而是【可复现地从 A 到 B】，不要喧宾夺主。

用法
----
    # 独立跑（仿真已经起来的情况下）
    ros2 run sim_benchmark exploration_runner.py --ros-args \
        -p world:=simple -p lane_spacing:=1.2 -p speed:=0.35

    # 由 benchmark 编排（见 stage_a_runner.py）
"""

import json
import math
import os
import time

import numpy as np
import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy
from std_msgs.msg import Bool, String
from exploration_routes import make_safe_path


# ---------------------------------------------------------------------------
# 覆盖轨迹生成
# ---------------------------------------------------------------------------


def read_pgm(path):
    """读 P5 二进制 PGM，返回 (img[H,W] uint8)。"""
    with open(path, "rb") as f:
        raw = f.read()
    if not raw.startswith(b"P5"):
        raise ValueError(f"{path} 不是 P5 二进制 PGM")
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


def load_gt_free_bbox(gt_yaml, clearance=0.45, margin=0.30):
    """从 GT 地图求出「可安全通行的矩形包围盒」，作为探索范围。

    为什么不去逐格规划覆盖路径：
      那需要地图的可通行连通域分析 + 走廊分解，代码量和失败模式都显著上升，
      而阶段 A 只需要"扫得够全"，不需要"扫得最优"。用一个内缩后的矩形
      覆盖住可通行区域即可 —— 简单、可复现、不含随机性。

    clearance : 距离障碍至少留多少米（含机器人半宽）
    margin    : 矩形再往里缩多少米，保证机器人不会贴着墙走
    """
    import yaml as _yaml

    with open(gt_yaml) as f:
        meta = _yaml.safe_load(f)
    img = read_pgm(os.path.join(os.path.dirname(os.path.abspath(gt_yaml)),
                                meta["image"]))
    res = float(meta["resolution"])
    ox, oy = float(meta["origin"][0]), float(meta["origin"][1])

    free = img > 200                       # 亮 = 自由
    h, w = free.shape

    # 用距离变换求「到最近障碍的欧氏距离」，比形态学腐蚀更准，一次搞定。
    # 没有 scipy 时退化成矩形扫描的近似（用累积和算方形邻域全自由）。
    try:
        from scipy import ndimage as ndi  # type: ignore
        dist = ndi.distance_transform_edt(free) * res
        safe = free & (dist >= clearance + margin)
    except ImportError:
        r = max(1, int(round((clearance + margin) / res)))
        cum = np.cumsum(np.cumsum(free.astype(np.int32), 0), 1)
        cum = np.pad(cum, ((1, 0), (1, 0)))
        ys, xs = np.mgrid[0:h, 0:w]
        y0 = np.clip(ys - r, 0, h - 1); y1 = np.clip(ys + r, 0, h - 1)
        x0 = np.clip(xs - r, 0, w - 1); x1 = np.clip(xs + r, 0, w - 1)
        area = (y1 - y0 + 1) * (x1 - x0 + 1)
        filled = cum[y1 + 1, x1 + 1] - cum[y0, x1 + 1] - cum[y1 + 1, x0] + cum[y0, x0]
        safe = free & (filled == area)

    if not np.any(safe):
        raise RuntimeError(
            f"GT 地图 {gt_yaml} 里没有满足 clearance={clearance} 的可通行区域，"
            "调小 clearance 或检查地图"
        )

    iy, ix = np.where(safe)
    # 图像行 0 = 最大 y，转世界坐标
    xs_w = ox + (ix + 0.5) * res
    ys_w = oy + (h - 1 - iy + 0.5) * res
    bbox = (float(xs_w.min()), float(ys_w.min()), float(xs_w.max()), float(ys_w.max()))
    return bbox, res


def make_lawnmower_path(bbox, lane_spacing, start, yaw0=0.0):
    """生成割草机式覆盖路点列表 [(x, y, arrive_yaw), ...]。

    方向选择：沿 x 轴做长边扫描（每条 lane 沿 x 走），lane 之间在 y 方向推进。
    相邻 lane 的行走方向相反（蛇形），这样转弯只发生在两端，路径最短。

    起点处理：把离 start 最近的 lane 端点放在最前面，避免机器人先横向
    穿越大半个地图才到第一条 lane（那一段会额外贡献里程但不增加覆盖质量）。
    """
    x0, y0, x1, y1 = bbox
    if x1 <= x0 or y1 <= y0:
        raise RuntimeError(f"探索包围盒无效：{bbox}")

    # 在 y 方向排 lane，第一条和最后一条离边界各留半个 lane，避免死角
    n_lanes = max(1, int(math.floor((y1 - y0) / lane_spacing)) + 1)
    ys = [y0 + (i + 0.5) * (y1 - y0) / n_lanes for i in range(n_lanes)]

    path = []
    for i, y in enumerate(ys):
        # 蛇形：偶数行 +x 方向，奇数行 -x 方向
        if i % 2 == 0:
            path.append((x0, y, 0.0))
            path.append((x1, y, math.pi))
        else:
            path.append((x1, y, math.pi))
            path.append((x0, y, 0.0))

    # 起点重排：找到离机器人当前位置最近的 lane，从它开始走。
    #
    # ⚠️ 必须【环形】遍历全部 lane。
    # 旧实现写的是 `for i in range(lane_idx, n_lanes)`，从 lane_idx 一路走到
    # 最后一条就结束了 —— 前面那 lane_idx 条 lane 被永久丢弃。
    # 而机器人出生点在地图正中，lane_idx ≈ n_lanes/2，于是【每次都只探索半张图】。
    # 实测后果：simple 环境 20x20m，轨迹 y 只覆盖 -0.53~8.57（应为 -9~9），
    # 下半张图完全没去，占据召回率被死死压在 0.24。
    # 这个 bug 不会报错、不会告警，只会让地图默默缺一半，极难察觉。
    sx, sy = start
    best = min(range(len(path)), key=lambda k: (path[k][0] - sx) ** 2 + (path[k][1] - sy) ** 2)
    lane_idx = best // 2

    order = [((lane_idx + k) % n_lanes) for k in range(n_lanes)]
    trimmed = []
    for i in order:
        if i % 2 == 0:
            trimmed.append((x0, ys[i], 0.0))
            trimmed.append((x1, ys[i], math.pi))
        else:
            trimmed.append((x1, ys[i], math.pi))
            trimmed.append((x0, ys[i], 0.0))
    # 若 start 更靠近第一条 lane 的末端，颠倒这一 lane 的进入顺序
    if best % 2 == 1:
        trimmed[0], trimmed[1] = trimmed[1], trimmed[0]
    return trimmed


# ---------------------------------------------------------------------------
# 追踪控制器
# ---------------------------------------------------------------------------


class ExplorationRunner(Node):
    """闭环跟踪割草机轨迹，完成一整轮覆盖后发 done 信号。"""

    # 追踪器状态
    IDLE, TURN, DRIVE, ALIGN, RECOVER = 0, 1, 2, 3, 4

    def __init__(self):
        super().__init__("exploration_runner")

        # ---- 探索参数 ----
        self.declare_parameter("world", "simple")
        self.declare_parameter("lane_spacing", 1.2)
        """相邻扫描线的间距（m）。应 ≤ 2 × 雷达有效半径 × 视场重叠系数。
        经验值：3D 雷达水平 360°，有效工作半径取 8m 的话，间距 1.2m 时
        每条 lane 之间有充分重叠，墙不会漏。太大 → 中间出现未观测带。"""

        self.declare_parameter("speed", 0.35)
        """直线段速度（m/s）。偏慢是有意为之：快走会让点云帧间运动畸变
        变大（尤其转弯时），而我们要评的是建图算法的极限，不是速度。"""

        self.declare_parameter("turn_speed", 1.0)
        """转弯角速度上限（rad/s），受机器人 max_vel.theta 约束。"""

        self.declare_parameter("clearance", 0.45)
        """路径离障碍的最小距离（m）。"""

        self.declare_parameter("goal_tol", 0.10)
        """到点判据（m）。太小会因为控制抖动卡住，太大会切角漏扫。"""

        self.declare_parameter("yaw_tol", 0.10)
        """航向对准判据（rad）。"""

        self.declare_parameter("align_dist", 1.0)
        """进入该距离（m）后若航向偏差大，先停车转向再前进。

        不加这一条时，接近路点但航向偏差大 → v 与 w 同时作用画出圆弧，
        机器人绕着路点转圈、dist 不收敛 → 触发卡死跳过。
        实测 simple 一次探索里 11 次卡死、浪费 220s（29%），并跳过 11 个路点。
        """

        self.declare_parameter("cmd_topic", "/cmd_vel")
        self.declare_parameter("odom_topic", "/odom")
        self.declare_parameter("done_topic", "/exploration_done")
        self.declare_parameter("status_topic", "/exploration_status")
        self.declare_parameter("traj_dir", "~/.ros/sim_eval")

        # ---- 里程计来源（用于闭环反馈）----
        # 用 GT odom 还是 SLAM odom？阶段 A 必须用 GT ——
        # 否则"轨迹跟踪误差"里混进了建图算法的定位误差，两个变量again耦合。
        self.declare_parameter("use_gt_odom", True)

        # ---- 超时保护 ----
        self.declare_parameter("max_duration", 1800.0)
        self.declare_parameter("stuck_timeout", 20.0)
        """位置多久不变算卡死（s）。"""

        self.declare_parameter("recover_max", 2)
        """卡死时先做几次恢复动作（后退+转向）再跳过路点。

        设为 0 则退回旧行为（直接跳过）。旧行为实测一次探索触发 11 次跳过，
        意味着 11 段扫描线完全没走，是地图出现未观测空洞的直接原因。"""

        self.declare_parameter("recover_duration", 3.0)
        """单次恢复动作持续时间（s）。"""

        self.declare_parameter("recover_speed", 0.25)
        """恢复时后退速度（m/s）。"""

        self.declare_parameter("recover_turn", 1.0)
        """恢复时转向角速度（rad/s）。"""

        self._world = self.get_parameter("world").value
        self._spacing = float(self.get_parameter("lane_spacing").value)
        self._speed = float(self.get_parameter("speed").value)
        self._turn_speed = float(self.get_parameter("turn_speed").value)
        self._clearance = float(self.get_parameter("clearance").value)
        self._goal_tol = float(self.get_parameter("goal_tol").value)
        self._yaw_tol = float(self.get_parameter("yaw_tol").value)
        self._align_dist = float(self.get_parameter("align_dist").value)
        self._max_dur = float(self.get_parameter("max_duration").value)
        self._stuck_to = float(self.get_parameter("stuck_timeout").value)
        self._recover_max = int(self.get_parameter("recover_max").value)
        self._recover_dur = float(self.get_parameter("recover_duration").value)
        self._recover_speed = float(self.get_parameter("recover_speed").value)
        self._recover_turn = float(self.get_parameter("recover_turn").value)
        self._recover_count = 0
        self._t_recover = 0.0

        # ---- 状态 ----
        self._pose = None            # (x, y, yaw)
        self._path = []
        self._wp_idx = 0
        self._state = self.IDLE
        self._t_start = None
        self._t_last_progress = None
        self._last_pos = None
        self._finished = False
        self._skipped = 0
        self._tracked = []           # 实际走过的点，用于存轨迹

        # ---- 通信 ----
        qos = QoSProfile(depth=10, history=QoSHistoryPolicy.KEEP_LAST,
                         reliability=QoSReliabilityPolicy.RELIABLE)
        odom_topic = ("/ground_truth/odom" if self.get_parameter("use_gt_odom").value
                      else self.get_parameter("odom_topic").value)
        self.create_subscription(Odometry, odom_topic,
                                 self._on_odom, qos)
        self._pub_cmd = self.create_publisher(Twist, self.get_parameter("cmd_topic").value, 10)
        self._pub_done = self.create_publisher(Bool, self.get_parameter("done_topic").value, 2)
        self._pub_status = self.create_publisher(String, self.get_parameter("status_topic").value, 2)

        self.create_timer(0.05, self._tick)          # 20 Hz 控制回路
        self.create_timer(10.0, self._report)        # 10 s 打一次进度

        self.get_logger().info(
            f"探索 runner 启动：world={self._world} spacing={self._spacing}m "
            f"speed={self._speed}m/s clearance={self._clearance}m"
        )

    # ---------------------------------------------------------------
    def _on_odom(self, msg: Odometry):
        p = msg.pose.pose
        q = p.orientation
        # yaw from quaternion（仅需平面姿态）
        siny = 2.0 * (q.w * q.z + q.x * q.y)
        cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        yaw = math.atan2(siny, cosy)
        self._pose = (p.position.x, p.position.y, yaw)

    # ---------------------------------------------------------------
    def _load_path(self):
        from ament_index_python.packages import get_package_share_directory
        share = get_package_share_directory("sim_worlds")
        gt_yaml = os.path.join(share, "maps", f"{self._world}.yaml")
        meta_path = os.path.join(share, "maps", f"{self._world}_meta.json")
        with open(meta_path) as f:
            meta = json.load(f)
        sp = meta["recommended_spawn"]
        start = (float(sp["x"]), float(sp["y"]))

        bbox, res = load_gt_free_bbox(gt_yaml, clearance=self._clearance)
        self._path, route_info = make_safe_path(gt_yaml, start, self._spacing,
                                               self._clearance, read_pgm)
        out_dir = os.path.expanduser(self.get_parameter("traj_dir").value)
        os.makedirs(out_dir, exist_ok=True)
        with open(os.path.join(out_dir, "coverage_plan.json"), "w") as stream:
            json.dump(dict(route_info, waypoints=self._path), stream, indent=2)
        self._wp_idx = 0
        self._state = self.TURN
        self._t_start = time.time()
        self._t_last_progress = time.time()
        total_len = sum(
            math.hypot(self._path[k + 1][0] - self._path[k][0],
                       self._path[k + 1][1] - self._path[k][1])
            for k in range(len(self._path) - 1)
        )
        self.get_logger().info(
            f"探索路径已生成：{len(self._path)} 个路点 / {len(self._path)//2} 条 lane，"
            f"总长 {total_len:.1f} m，包裹 box={tuple(round(v,2) for v in bbox)}，"
            f"起点=({start[0]:.2f},{start[1]:.2f})，res={res}"
        )
        self._publish_status("path_ready")

    def _publish_status(self, s):
        m = String()
        m.data = s
        self._pub_status.publish(m)

    # ---------------------------------------------------------------
    def _stop(self):
        self._pub_cmd.publish(Twist())

    def _report(self):
        if self._finished or not self._path:
            return
        pct = 100.0 * self._wp_idx / max(1, len(self._path))
        self.get_logger().info(
            f"探索进度 {self._wp_idx}/{len(self._path)} ({pct:.0f}%)  "
            f"state={['IDLE','TURN','DRIVE','ALIGN','RECOVER'][self._state]}"
        )

    # ---------------------------------------------------------------
    def _tick(self):
        if self._finished:
            return
        if self._pose is None:
            self._publish_status("waiting_odom")
            return
        if not self._path:
            self._load_path()
            return

        now = time.time()
        if now - self._t_start > self._max_dur:
            self.get_logger().warn("探索超过最大时长，提前结束")
            self._finish("timeout")
            return

        x, y, yaw = self._pose
        # 记录实际轨迹（去重：移动超过 5cm 才记一笔）
        if not self._tracked or math.hypot(x - self._tracked[-1][1],
                                           y - self._tracked[-1][2]) > 0.05:
            self._tracked.append((now, x, y, yaw))
            # 增量落盘：每积够 50 个点就追加一次。
            #
            # ⚠️ 为什么不能只在 _finish() 里一次性写（实测踩坑）：
            #   本轮实验跑到 6 分钟时，用 ls 看输出目录发现 map.pgm 和
            #   exploration_traj.csv 的时间戳都还是【上一轮的】、大小也不变，
            #   看上去像"卡住了"。实际机器人正在正常走 —— 只是这两个文件
            #   都是【跑完才写】。这种"进度不可观测"会让人反复误判、
            #   甚至误杀正在正常运行的实验。所以改成边走边追加。
            if len(self._tracked) % 50 == 0:
                self._append_traj()

        # 卡死检测：位置长时间不变
        #
        # 实测失效模式（simple 一次探索 4~11 次）：
        #   机器人贴着墙（或墙的拐角）时，纯追踪律仍在满速下发 linear.x，
        #   但前进方向与墙面法向冲突 → 位置在 10^-5 精度上冻结，
        #   而 cmd_vel 一直显示 0.5 m/s。机器人【没有】被物理卡住
        #   （sim_kinematic_freeze 已把 mu/kp/kd 全置 0，没有接触力），
        #   只是运动学插件在这个朝向下无法产生有效位移。
        #
        #   原实现在此处直接 skip 路点：浪费 20s，且那一段扫描线【完全没走】，
        #   留下未观测的空洞。改为先做一次恢复动作（后退 + 转向），
        #   给机器人一个脱离墙面的机会，再继续追同一个路点。
        #   只有恢复也失败时才跳过。
        #
        # ⚠️ _recover_count 只在【路点前进】时清零（见 DRIVE 的到点分支），
        #    不能在"位置动了"时清零：恢复动作本身就是后退（位置会动），
        #    若在这里清零，计数永远涨不上去 → 无限恢复 → 死循环。
        if self._last_pos is None or math.hypot(x - self._last_pos[0],
                                                y - self._last_pos[1]) > 0.05:
            self._last_pos = (x, y)
            self._t_last_progress = now
        elif now - self._t_last_progress > self._stuck_to:
            if self._recover_count < self._recover_max:
                self._recover_count += 1
                self.get_logger().warn(
                    f"卡死（{self._stuck_to:.0f}s 未移动），"
                    f"执行恢复动作 {self._recover_count}/{self._recover_max}"
                    f"（后退 + 转向），不跳过路点"
                )
                self._state = self.RECOVER
                self._t_recover = now
                self._t_last_progress = now
                return
            self.get_logger().warn(
                f"恢复 {self._recover_max} 次仍无进展，跳过当前路点 "
                f"{self._wp_idx}/{len(self._path)}"
            )
            self._wp_idx += 1
            self._skipped += 1
            self._t_last_progress = now
            self._recover_count = 0
            if self._wp_idx >= len(self._path):
                self._finish("done_with_stuck_skips")
                return
            self._state = self.TURN

        if self._wp_idx >= len(self._path):
            self._finish("done_with_stuck_skips" if self._skipped else "done")
            return

        tx, ty, _t_yaw = self._path[self._wp_idx]
        dx, dy = tx - x, ty - y
        dist = math.hypot(dx, dy)
        desired_yaw = math.atan2(dy, dx)
        yaw_err = math.atan2(math.sin(desired_yaw - yaw),
                             math.cos(desired_yaw - yaw))

        cmd = Twist()

        if dist < self._goal_tol and self._state != self.RECOVER:
            self._stop()
            self._wp_idx += 1
            self._state = self.TURN
            self._recover_count = 0
            self._t_last_progress = now
            return

        # ---- 恢复动作：后退 + 转向，脱离卡住的朝向/位置 ----
        # 必须在 TURN/DRIVE/ALIGN 之前处理，否则会被正常追踪逻辑覆盖。
        # 后退速度用负 linear.x；转向固定方向（转身 180° 的近似），
        # 目的是换一个朝向重新尝试，而不是精确控制末端位姿。
        if self._state == self.RECOVER:
            if now - self._t_recover < self._recover_dur:
                cmd.linear.x = -self._recover_speed
                cmd.angular.z = self._recover_turn
                self._pub_cmd.publish(cmd)
                return
            # 恢复结束 → 重新对准当前路点
            self._state = self.TURN
            self._t_last_progress = now
            self._last_pos = (x, y)
            return

        if self._state == self.TURN:
            if abs(yaw_err) < self._yaw_tol:
                self._state = self.DRIVE
            else:
                # 纯比例控制，限幅到 turn_speed
                w = max(-self._turn_speed, min(self._turn_speed, 2.5 * yaw_err))
                cmd.angular.z = float(w)
                self._pub_cmd.publish(cmd)
                return

        if self._state == self.DRIVE:
            if dist < self._goal_tol:
                self._wp_idx += 1
                self._state = self.TURN
                self._last_pos = (x, y)
                self._t_last_progress = now
                # 成功到达一个路点才算"恢复了"，清空恢复计数
                self._recover_count = 0
                return

            # ---- 近距对准优先（修"绕路点画圈"导致的卡死）----
            #
            # 原实现无条件同时下发 v 和 w：
            #   v = min(speed, 1.5*dist)   w = clamp(±0.6, 2.0*yaw_err)
            # 当机器人接近路点但航向偏得多时（dist≈0.3, yaw_err≈0.8rad），
            # v=0.45 m/s 与 w=0.6 rad/s 同时作用 → 曲率半径 0.75m 的圆弧。
            # 圆弧的圆心如果落在路点附近，机器人就会【绕着路点转圈】，
            # 位置一直在动（>5cm，所以不会重置卡死计时器），但 dist 不收敛。
            # 实测 simple 一次探索里触发 11 次"20s 未移动"、浪费 220s
            # （占总时长 29%），且每次都跳过 1 个路点 → 有 11 段扫描线没走。
            #
            # 修法：进入到达半径后先停车转向（差速机器人的标准做法），
            # 对齐了再走。避免"边走边大角度修正"画出圆弧。
            if dist < self._align_dist and abs(yaw_err) > self._yaw_tol:
                self._state = self.ALIGN
                cmd.angular.z = float(max(-self._turn_speed,
                                          min(self._turn_speed, 2.5 * yaw_err)))
                self._pub_cmd.publish(cmd)
                return

            # 边走边修航向：小角度偏差用角速度补偿
            cmd.linear.x = float(min(self._speed, 1.5 * dist))
            cmd.angular.z = float(max(-self._turn_speed * 0.6,
                                      min(self._turn_speed * 0.6, 2.0 * yaw_err)))
            self._pub_cmd.publish(cmd)

        elif self._state == self.ALIGN:
            # 只转不走，直到对准；对准后回到 DRIVE 继续逼近
            if abs(yaw_err) < self._yaw_tol:
                self._state = self.DRIVE
                self._t_last_progress = now
                return
            cmd.angular.z = float(max(-self._turn_speed,
                                      min(self._turn_speed, 2.5 * yaw_err)))
            self._pub_cmd.publish(cmd)

    # ---------------------------------------------------------------
    def _traj_path(self):
        out_dir = os.path.expanduser(self.get_parameter("traj_dir").value)
        return os.path.join(out_dir, "exploration_traj.csv")

    def _append_traj(self):
        """把已积累的轨迹点追加到 CSV（幂等：每次都整体重写，避免重复行）。"""
        try:
            csv_path = self._traj_path()
            os.makedirs(os.path.dirname(csv_path), exist_ok=True)
            with open(csv_path, "w", newline="\n", encoding="utf-8") as f:
                f.write("t,x,y,yaw\n")
                for t, xx, yy, yy2 in self._tracked:
                    f.write(f"{t:.3f},{xx:.4f},{yy:.4f},{yy2:.4f}\n")
        except OSError as e:
            self.get_logger().warn(f"增量写轨迹失败：{e}")

    # ---------------------------------------------------------------
    def _finish(self, reason):
        if self._finished:
            return
        self._stop()
        self._finished = True
        self.get_logger().info(f"探索结束（{reason}）")
        # 存实际轨迹 CSV（收尾时整体再写一遍，确保一定是最新且完整的）
        out_dir = os.path.expanduser(self.get_parameter("traj_dir").value)
        os.makedirs(out_dir, exist_ok=True)
        try:
            csv_path = os.path.join(out_dir, "exploration_traj.csv")
            with open(csv_path, "w", newline="\n", encoding="utf-8") as f:
                f.write("t,x,y,yaw\n")
                for t, xx, yy, yy2 in self._tracked:
                    f.write(f"{t:.3f},{xx:.4f},{yy:.4f},{yy2:.4f}\n")
            self.get_logger().info(f"探索轨迹已保存 -> {csv_path}")
        except OSError as e:
            self.get_logger().warn(f"写轨迹失败：{e}")

        done = Bool()
        done.data = True
        self._pub_done.publish(done)
        self._publish_status(f"finished:{reason}")

        # 哨兵文件：给编排器一个不依赖 ROS 的可靠完成信号。
        # 为什么不用 done 话题：编排器轮询 `ros2 topic echo --once` 每次要起
        # 一个节点、耗时 8~15s，且 QoS 不匹配时静默收不到 —— 实测把一次
        # 完整探索误判成"10 秒超时"。文件是最稳的进程间信号。
        try:
            sentinel = os.path.join(out_dir, "exploration.done")
            with open(sentinel, "w") as f:
                f.write(f"{reason}\n{len(self._tracked)} points\n")
        except OSError as e:
            self.get_logger().warn(f"写完成哨兵失败：{e}")


def main():
    rclpy.init()
    node = ExplorationRunner()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
