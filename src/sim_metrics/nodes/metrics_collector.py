#!/usr/bin/env python3
"""
metrics_collector
=================
在线指标采集 + 轨迹记录。

对应需求 #4：「可视化界面，可以展示建图点云、不同稀疏程度的点云、pcd 和
导航指标、建图轨迹和导航轨迹，方便进行对比」。

这个节点干三件事：

  1. **轨迹双记录**。同时记真值轨迹（/ground_truth/odom）和估计轨迹（/odom），
     发布成 /eval/gt_path 和 /eval/est_path，rviz2 里红青双色同屏。
     真值轨迹存在的唯一目的是"当裁判"，算法绝不能消费它。

  2. **在线导航指标**。到达状态、耗时、路径长度、路径效率、与障碍最小距离、
     碰撞计数、路径跟踪误差、控制量平滑度。全部实时算，实时发 JSON。

  3. **落盘**。轨迹和指标写 CSV，供离线用 evo / 自研脚本做精细对齐分析。

为什么指标要在线算而不是等离线：
  离线只能告诉你"最后结果差了 0.8m"，在线能告诉你"在第 37 秒进长走廊时
  开始偏"。定位失效时刻才是真正有价值的信息。

坐标系处理：
  两条轨迹都尝试变换到 target_frame（默认 map）。建图没起来时 map 不存在，
  自动回退到 odom。这样在"纯导航无建图"的场景下 rviz 也能正常显示。
"""

import csv
import json
import math
import os
import time as _time

import numpy as np
import rclpy
from action_msgs.msg import GoalStatus, GoalStatusArray
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Odometry, OccupancyGrid, Path
from rclpy.node import Node
from rclpy.qos import QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy
from rclpy.time import Time
from std_msgs.msg import String, Bool
from std_srvs.srv import Trigger
from tf2_ros import Buffer, TransformException, TransformListener


def quat_to_yaw(x, y, z, w):
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


def yaw_to_quat(yaw):
    return (0.0, 0.0, math.sin(yaw * 0.5), math.cos(yaw * 0.5))


class MetricsCollector(Node):
    def __init__(self):
        super().__init__("metrics_collector")

        self.declare_parameter("target_frame", "map")
        self.declare_parameter("fallback_frame", "odom")
        self.declare_parameter("robot_frame", "base_footprint")
        self.declare_parameter("run_name", "")
        self.declare_parameter("output_dir", "~/.ros/sim_eval")
        self.declare_parameter("goal_status_topic", "/navigate_to_pose/_action/status")
        self.declare_parameter("local_costmap_topic", "/local_costmap/costmap")
        self.declare_parameter("metrics_period", 0.5)
        self.declare_parameter("csv_period", 2.0)
        self.declare_parameter("lethal_cost", 90)
        self.declare_parameter("collision_persist_frames", 5)
        self.declare_parameter("save_trajectory_csv", True)

        self._target = self.get_parameter("target_frame").value
        self._fallback = self.get_parameter("fallback_frame").value
        self._lethal = int(self.get_parameter("lethal_cost").value)
        self._coll_persist = int(self.get_parameter("collision_persist_frames").value)
        self._save_csv = bool(self.get_parameter("save_trajectory_csv").value)

        run_name = self.get_parameter("run_name").value or _time.strftime("%Y%m%d_%H%M%S")
        self._out_dir = os.path.expanduser(os.path.join(
            self.get_parameter("output_dir").value, run_name))
        os.makedirs(self._out_dir, exist_ok=True)

        # ---- 轨迹缓冲 ----
        self._gt = []      # (t, x, y, yaw)
        self._est = []
        self._cmd = []     # (t, vx, wz)
        self._plan = []    # 当前全局路径 [(x,y)...]

        # ---- 状态 ----
        self._goal_state = "IDLE"
        self._goal_start_t = None
        self._goal_end_t = None
        self._goals = []          # (start, end, status)
        self._goal_gt_index = 0
        self._goal_reference_length = None
        self._path_ratios = []
        self._col_count = 0
        self._col_streak = 0
        self._contact_seen = False
        self._contact_active = False
        self._min_obs = float("inf")
        self._track_err = []
        self._use_frame = None
        self._last_est_stamp = -1.0

        self._buffer = Buffer()
        self._listener = TransformListener(self._buffer, self)
        self.create_timer(.1, self._sample_est_tf)

        qos = QoSProfile(depth=10, history=QoSHistoryPolicy.KEEP_LAST,
                         reliability=QoSReliabilityPolicy.RELIABLE)

        self.create_subscription(Odometry, "/ground_truth/odom", self._on_gt, qos)
        self.create_subscription(Odometry, "/odom", self._on_est, qos)
        self.create_subscription(Path, "/plan", self._on_plan, qos)
        self.create_subscription(Twist, "/cmd_vel", self._on_cmd, qos)
        self.create_subscription(Bool, "/eval/contact", self._on_contact, qos)
        self.create_subscription(
            GoalStatusArray, self.get_parameter("goal_status_topic").value,
            self._on_status, qos)
        self.create_subscription(
            OccupancyGrid, self.get_parameter("local_costmap_topic").value,
            self._on_costmap, QoSProfile(depth=1, history=QoSHistoryPolicy.KEEP_LAST,
                                         reliability=QoSReliabilityPolicy.RELIABLE))

        self._pub_gt = self.create_publisher(Path, "/eval/gt_path", 1)
        self._pub_est = self.create_publisher(Path, "/eval/est_path", 1)
        self._pub_metrics = self.create_publisher(String, "/eval/metrics", 1)

        p = float(self.get_parameter("metrics_period").value)
        self.create_timer(max(0.1, p), self._publish_metrics)
        c = float(self.get_parameter("csv_period").value)
        self.create_timer(max(0.5, c), self._flush)
        self.create_service(Trigger, '/eval/flush', self._flush_service)

        self.get_logger().info(
            f"指标采集已启动。输出目录：{self._out_dir}\n"
            "  真值轨迹 -> /eval/gt_path    估计轨迹 -> /eval/est_path\n"
            "  实时指标 -> /eval/metrics"
        )

    # ------------------------------------------------------------------
    def _pose_in_target(self, msg: Odometry):
        """把里程计位姿变换到 target_frame。拿不到 TF 就退回 fallback_frame。

        回退是有必要的：纯导航测试时没有建图节点，map 根本不存在。
        硬等 map 会让 /eval/*_path 一直是空的，rviz 里什么都看不到，
        而你会以为是记录逻辑坏了。
        """
        t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        px = msg.pose.pose.position.x
        py = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        yaw = quat_to_yaw(q.x, q.y, q.z, q.w)
        src = msg.header.frame_id or self._fallback

        for frame in (self._target, self._fallback):
            if src == frame:
                return t, px, py, yaw, frame
            try:
                tf = self._buffer.lookup_transform(frame, src, Time())
            except TransformException:
                continue
            tr = tf.transform.translation
            rq = tf.transform.rotation
            tyaw = quat_to_yaw(rq.x, rq.y, rq.z, rq.w)
            c, s = math.cos(tyaw), math.sin(tyaw)
            return t, tr.x + c * px - s * py, tr.y + s * px + c * py, yaw + tyaw, frame
        return t, px, py, yaw, src

    def _on_gt(self, msg: Odometry):
        # Gazebo p3d supplies world coordinates. Never apply the estimator's
        # map->odom correction to the reference trajectory as well.
        p = msg.pose.pose
        q = p.orientation
        rec = (msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9,
               p.position.x, p.position.y, quat_to_yaw(q.x, q.y, q.z, q.w), self._target)
        self._gt.append(rec[:4])
        self._use_frame = self._use_frame or rec[4]
        if len(self._gt) % 10 == 0:
            self._pub_gt.publish(self._path_msg(self._gt, rec[4]))

    def _on_est(self, msg: Odometry):
        if self._target == "map":
            return  # Sample the full SLAM/AMCL transform at its own timestamp.
        t, x, y, yaw, frame = self._pose_in_target(msg)
        self._est.append((t, x, y, yaw))
        self._use_frame = self._use_frame or frame
        if len(self._est) % 10 == 0:
            self._pub_est.publish(self._path_msg(self._est, frame))
        if self._plan:
            self._track_err.append(self._dist_to_plan(x, y))

    def _sample_est_tf(self):
        if self._target != "map":
            return
        try:
            tf = self._buffer.lookup_transform(
                self._target, self.get_parameter("robot_frame").value, Time())
        except TransformException:
            return
        t = tf.header.stamp.sec + tf.header.stamp.nanosec * 1e-9
        if t <= self._last_est_stamp:
            return
        self._last_est_stamp = t
        p, q = tf.transform.translation, tf.transform.rotation
        self._est.append((t, p.x, p.y, quat_to_yaw(q.x, q.y, q.z, q.w)))
        self._use_frame = self._target
        if len(self._est) % 10 == 0:
            self._pub_est.publish(self._path_msg(self._est, self._target))
        if self._plan and self._goal_state == "EXECUTING":
            self._track_err.append(self._dist_to_plan(p.x, p.y))

    @staticmethod
    def _path_msg(points, frame):
        msg = Path()
        msg.header.frame_id = frame
        msg.header.stamp = Time(seconds=points[-1][0]).to_msg() if points else Time().to_msg()
        for (t, x, y, yaw) in points[::max(1, len(points)//2000)]:
            ps = PoseStamped()
            ps.header.frame_id = frame
            ps.header.stamp = Time(seconds=t).to_msg()
            ps.pose.position.x = float(x)
            ps.pose.position.y = float(y)
            ps.pose.position.z = 0.0
            qx, qy, qz, qw = yaw_to_quat(yaw)
            ps.pose.orientation.x = qx
            ps.pose.orientation.y = qy
            ps.pose.orientation.z = qz
            ps.pose.orientation.w = qw
            msg.poses.append(ps)
        return msg

    def _dist_to_plan(self, x, y):
        arr = np.asarray(self._plan, dtype=np.float64)
        if arr.size == 0:
            return float("nan")
        return float(np.min(np.hypot(arr[:, 0] - x, arr[:, 1] - y)))

    def _on_plan(self, msg: Path):
        self._plan = [(p.pose.position.x, p.pose.position.y) for p in msg.poses]
        if self._goal_state == "EXECUTING" and self._goal_reference_length is None and len(self._plan) > 1:
            points = np.asarray(self._plan)
            self._goal_reference_length = float(np.hypot(*np.diff(points, axis=0).T).sum())

    def _on_cmd(self, msg: Twist):
        t = self.get_clock().now().nanoseconds * 1e-9
        self._cmd.append((t, msg.linear.x, msg.angular.z))

    def _on_contact(self, msg):
        self._contact_seen = True
        if not msg.data:
            self._col_streak = 0
            self._contact_active = False
            return
        self._col_streak += 1
        if self._col_streak >= self._coll_persist and not self._contact_active:
            self._col_count += 1
            self._contact_active = True

    def _on_status(self, msg: GoalStatusArray):
        if not msg.status_list:
            return
        s = msg.status_list[-1].status
        now = self.get_clock().now().nanoseconds * 1e-9
        if s in (GoalStatus.STATUS_EXECUTING, GoalStatus.STATUS_ACCEPTED):
            if self._goal_state != "EXECUTING":
                self._goal_state = "EXECUTING"
                self._goal_start_t = now
                self._goal_gt_index = max(0, len(self._gt)-1)
                self._goal_reference_length = None
                self.publish_metrics()
        elif s == GoalStatus.STATUS_SUCCEEDED and self._goal_state == "EXECUTING":
            self._goal_state = "SUCCEEDED"
            self._goal_end_t = now
            self._goals.append((self._goal_start_t, now, "SUCCEEDED"))
            if self._goal_reference_length and len(self._gt)-self._goal_gt_index > 1:
                points = np.asarray(self._gt[self._goal_gt_index:])[:, 1:3]
                actual = float(np.hypot(*np.diff(points, axis=0).T).sum())
                self._path_ratios.append(actual/self._goal_reference_length)
            self.get_logger().info(
                f"目标达成，耗时 {now - (self._goal_start_t or now):.2f}s")
        elif s in (GoalStatus.STATUS_ABORTED, GoalStatus.STATUS_CANCELED) \
                and self._goal_state == "EXECUTING":
            self._goal_state = "FAILED"
            self._goal_end_t = now
            self._goals.append((self._goal_start_t, now, "FAILED"))

    def _on_costmap(self, msg: OccupancyGrid):
        # 最小障碍距离：只扫局部代价地图，120x120 网格跑 numpy 无压力
        arr = np.asarray(msg.data, dtype=np.int16).reshape(msg.info.height, msg.info.width)
        lethal = arr >= self._lethal
        if not np.any(lethal):
            return
        if self._est:
            try:
                tf = self._buffer.lookup_transform(msg.header.frame_id,
                    self.get_parameter("robot_frame").value, Time.from_msg(msg.header.stamp))
            except TransformException:
                return
            x, y = tf.transform.translation.x, tf.transform.translation.y
            ix = int((x - msg.info.origin.position.x) / msg.info.resolution)
            iy = int((y - msg.info.origin.position.y) / msg.info.resolution)
            if 0 <= ix < msg.info.width and 0 <= iy < msg.info.height:
                ys, xs = np.nonzero(lethal)
                d = np.hypot((xs - ix) * msg.info.resolution,
                             (ys - iy) * msg.info.resolution)
                self._min_obs = min(self._min_obs, float(d.min()))

    # ------------------------------------------------------------------
    def _nav_metrics(self):
        m = {
            "goal_state": self._goal_state,
            "goals_total": len(self._goals),
            "goals_succeeded": sum(1 for g in self._goals if g[2] == "SUCCEEDED"),
            "collision_count": self._col_count if getattr(self, "_contact_seen", False) else None,
            "collision_note": "Gazebo contact episodes; ground/self excluded; debounced" if
                getattr(self, "_contact_seen", False) else "No contact messages received",
            "min_obstacle_dist": None if math.isinf(self._min_obs) else round(self._min_obs, 3),
        }
        if self._goals:
            durs = [g[1] - (g[0] if g[0] is not None else g[1]) for g in self._goals]
            m["time_to_goal_last"] = round(durs[-1], 3)
            m["time_to_goal_mean"] = round(float(np.mean(durs)), 3)
        m["success_rate"] = (
            round(m["goals_succeeded"] / m["goals_total"], 3) if m["goals_total"] else None
        )

        # 路径长度：真值和估计都算，估计偏长说明里程计在虚增里程
        for key, buf in (("gt", self._gt), ("est", self._est)):
            if len(buf) > 1:
                a = np.asarray(buf, dtype=np.float64)
                m[f"path_length_{key}"] = round(
                    float(np.sum(np.hypot(np.diff(a[:, 1]), np.diff(a[:, 2])))), 3)

        if self._track_err:
            e = np.asarray([v for v in self._track_err if not math.isnan(v)])
            if e.size:
                m["tracking_error_rmse"] = round(float(np.sqrt(np.mean(e ** 2))), 4)
                m["tracking_error_max"] = round(float(e.max()), 4)
        ratios = getattr(self, "_path_ratios", [])
        m["path_length_ratio"] = round(float(np.mean(ratios)), 4) if ratios else None
        m["path_length_ratio_note"] = "Successful goals: GT travel / first received global plan length"

        # 平滑度：|d(cmd)/dt| 积分。腿式平台频繁顿挫会明显抬高这个值
        if len(self._cmd) > 2:
            a = np.asarray(self._cmd, dtype=np.float64)
            # Integral of |du/dt| is total variation, including irregular stamps.
            variation = np.abs(np.diff(a[:, 1])) + np.abs(np.diff(a[:, 2]))
            m["control_smoothness"] = round(float(np.sum(variation)), 4)
        return m

    def _mapping_metrics(self):
        m = {
            "gt_pose_count": len(self._gt),
            "est_pose_count": len(self._est),
        }
        if len(self._gt) > 5 and len(self._est) > 5:
            g = np.asarray(self._gt, dtype=np.float64)
            e = np.asarray(self._est, dtype=np.float64)
            # 时间最近邻配对，算在线 ATE（不做对齐，反映的是"实时偏差"）
            idx = np.searchsorted(g[:, 0], e[:, 0])
            idx = np.clip(idx, 0, len(g) - 1)
            left = np.maximum(idx - 1, 0)
            idx = np.where(np.abs(g[left, 0] - e[:, 0]) < np.abs(g[idx, 0] - e[:, 0]), left, idx)
            d = np.hypot(g[idx, 1] - e[:, 1], g[idx, 2] - e[:, 2])
            d = d[np.isfinite(d) & (np.abs(g[idx, 0] - e[:, 0]) < .5)]
            if d.size:
                m["ate_rmse_online"] = round(float(np.sqrt(np.mean(d ** 2))), 4)
                m["ate_max_online"] = round(float(d.max()), 4)
                m["ate_note"] = "未做轨迹对齐，反映实时偏差；离线请用 evo 做 Umeyama 对齐"
        return m

    def publish_metrics(self):
        if not rclpy.ok():
            return
        payload = {
            "frame": self._use_frame,
            "navigation": self._nav_metrics(),
            "mapping": self._mapping_metrics(),
        }
        msg = String()
        msg.data = json.dumps(payload, ensure_ascii=False)
        self._pub_metrics.publish(msg)

    def _publish_metrics(self):
        self.publish_metrics()

    # ------------------------------------------------------------------
    def _flush(self):
        if not self._save_csv:
            return
        try:
            if self._gt:
                self._write_csv(os.path.join(self._out_dir, "trajectory_gt.csv"),
                                ["t", "x", "y", "yaw"], self._gt)
            if self._est:
                self._write_csv(os.path.join(self._out_dir, "trajectory_est.csv"),
                                ["t", "x", "y", "yaw"], self._est)
            if self._cmd:
                self._write_csv(os.path.join(self._out_dir, "cmd_vel.csv"),
                                ["t", "vx", "wz"], self._cmd)
            metrics_path = os.path.join(self._out_dir, "metrics.json")
            with open(metrics_path + ".tmp", "w",
                      encoding="utf-8") as f:
                json.dump({
                    "navigation": self._nav_metrics(),
                    "mapping": self._mapping_metrics(),
                    "goals": [
                        {"start": g[0], "end": g[1], "status": g[2]} for g in self._goals
                    ],
                }, f, ensure_ascii=False, indent=2)
            os.replace(metrics_path + ".tmp", metrics_path)
        except OSError as e:
            self.get_logger().warn(f"写 CSV 失败：{e}")
            return False
        return True

    def _flush_service(self, request, response):
        response.success = bool(self._flush())
        response.message = json.dumps(self._nav_metrics())
        return response

    @staticmethod
    def _write_csv(path, header, rows):
        with open(path + ".tmp", "w", newline="", encoding="utf-8") as f:
            w = csv.writer(f)
            w.writerow(header)
            w.writerows(rows)
        os.replace(path + ".tmp", path)

    def destroy_node(self):
        self._flush()
        super().destroy_node()


def main():
    rclpy.init()
    node = MetricsCollector()
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
