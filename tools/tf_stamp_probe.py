#!/usr/bin/env python3
"""
tf_stamp_probe.py —— 测量 /scan 时戳与 TF(odom->lidar_stabilized) 时戳的错位
================================================================================
背景（2026-09-11）：
  gimbal_stabilizer 修复后 frame 已存在（tf2_echo 可查），
  但 slam_toolbox 的 MessageFilter 仍以每 2.5s 一帧的速度丢帧
  （= 队列 25 只进不出 → 全部帧按 scan 时戳查 TF 失败）。

  tf2_echo 查询用的是 Time(0)（最新值），所以它说"存在"；
  MessageFilter 用的是 scan.stamp（精确时刻），所以它说"查不到"。
  两者不矛盾 —— 问题出在时间戳覆盖范围。

  本探针直接测量三个量：
    ① 每个 scan 到达时，buffer 里 odom->lidar_stabilized 的最新时戳
    ② scan.stamp 与该最新时戳的差值（正 = scan 比 TF 新 = 需要外插）
    ③ can_transform(odom, lidar_stabilized, scan.stamp) 的实时结果
       以及"等到能查到"需要多久

用法：
    python3 tools/tf_stamp_probe.py [duration_sec]   # 默认 30
"""

import sys

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from sensor_msgs.msg import LaserScan
from tf2_msgs.msg import TFMessage
import tf2_ros


class StampProbe(Node):
    def __init__(self):
        super().__init__("tf_stamp_probe")
        self.set_parameters(
            [Parameter("use_sim_time", Parameter.Type.BOOL, True)])

        self.buffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.buffer, self)

        self.scans = []          # (recv_wall, stamp, ok_now, wait_needed)
        self.pending = {}        # stamp -> arrival_wall_time
        self.tf_latest = {}      # (parent, child) -> latest stamp seen

        self.create_subscription(LaserScan, "/scan", self._on_scan, 10)
        self.create_subscription(TFMessage, "/tf", self._on_tf, 100)

        self.n_ok = 0
        self.n_fail = 0
        self.n_recovered = 0
        self.deltas = []         # scan.stamp - tf_latest（可查时刻）

    def _on_tf(self, msg: TFMessage):
        for tr in msg.transforms:
            key = (tr.header.frame_id, tr.child_frame_id)
            self.tf_latest[key] = tr.header.stamp

    def _on_scan(self, msg: LaserScan):
        stamp = msg.header.stamp
        now_wall = self.get_clock().now()

        # ⚠️ rclpy.time.Time 的构造签名是
        #   Time(*, seconds=0, nanoseconds=0, clock_type=...)
        # 没有 stamp 关键字。差值一律用 to_msg() 后的整数纳秒算，
        # 不用 Time 的算术（Humble 的 Time 不支持 +/- 运算符）。
        t_scan = rclpy.time.Time(
            seconds=stamp.sec, nanoseconds=stamp.nanosec)

        try:
            self.buffer.lookup_transform(
                "odom", "lidar_stabilized", t_scan)
            self.n_ok += 1
            self.pending.pop((stamp.sec, stamp.nanosec), None)
            return
        except Exception:
            self.n_fail += 1
            self.pending[(stamp.sec, stamp.nanosec)] = now_wall

        # 每 20 帧输出一次现场
        total = self.n_ok + self.n_fail
        if total % 20 == 0:
            latest = self.tf_latest.get(("odom", "base_footprint"))
            latest_bs = self.tf_latest.get(("base_link", "lidar_stabilized"))
            sim_now = self.get_clock().now()

            def _ns(m):
                return m.sec * 1_000_000_000 + m.nanosec

            scan_ns = _ns(stamp)

            def _dt(t_msg):
                if t_msg is None:
                    return "N/A"
                return f"{(scan_ns - _ns(t_msg)) / 1e9:+.3f}s（正=TF比scan旧）"

            print(f"[scan #{total}] stamp={stamp.sec}.{stamp.nanosec:09d} "
                  f"sim_now={sim_now.nanoseconds / 1e9:.3f}")
            print(f"    最新 odom->base_footprint 相对 scan: "
                  f"{_dt(latest)}")
            print(f"    最新 base_link->lidar_stabilized 相对 scan: "
                  f"{_dt(latest_bs)}")
            print(f"    累计：按戳可查 {self.n_ok} / 失败 {self.n_fail} "
                  f"/ 事后恢复 {self.n_recovered}")

    def check_recovered(self):
        """检查之前失败的 stamp 现在能否查到（量化'等多久'）。"""
        for key in list(self.pending.keys()):
            sec, nsec = key
            t = rclpy.time.Time(seconds=sec, nanoseconds=nsec)
            try:
                self.buffer.lookup_transform("odom", "lidar_stabilized", t)
                waited = (self.get_clock().now().nanoseconds
                          - self.pending.pop(key).nanoseconds) / 1e9
                self.n_recovered += 1
                self.deltas.append(waited)
                if self.n_recovered <= 5:
                    print(f"    ↩ stamp={sec}.{nsec:09d} 在 {waited:.3f}s 后"
                          f"变得可查（恢复）")
            except Exception:
                pass


def main():
    duration = float(sys.argv[1]) if len(sys.argv) > 1 else 30.0
    rclpy.init()
    node = StampProbe()

    import time as _time
    t0 = _time.monotonic()
    while rclpy.ok() and _time.monotonic() - t0 < duration:
        rclpy.spin_once(node, timeout_sec=0.05)
        node.check_recovered()

    print("\n" + "=" * 60)
    print(f"总结（观察 {duration:.0f}s）：")
    print(f"  /scan 总数          : {node.n_ok + node.n_fail}")
    print(f"  按时戳立即可查      : {node.n_ok}")
    print(f"  按时戳查询失败      : {node.n_fail}")
    print(f"  其中事后恢复        : {node.n_recovered}")
    if node.deltas:
        ds = sorted(node.deltas)
        print(f"  恢复等待中位数      : {ds[len(ds)//2]:.3f}s "
              f"（min {ds[0]:.3f} / max {ds[-1]:.3f}）")
    if node.tf_latest:
        print("  TF 链最新时戳（相对仿真时钟）：")
        now_ns = node.get_clock().now().nanoseconds
        for (p, c), st in sorted(node.tf_latest.items()):
            age = (now_ns - (st.sec * 1_000_000_000 + st.nanosec)) / 1e9
            print(f"    {p:>18s} -> {c:<20s} 落后 now {age:+.3f}s")
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
