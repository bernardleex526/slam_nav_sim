#!/usr/bin/env python3
"""
points_to_scan
==============
从 3D 点云切出虚拟 2D 激光扫描（/scan）。

为什么需要它：
  本项目第一轮不单独挂 2D 单线雷达。但 slam_toolbox / Cartographer 2D 这些
  二维建图算法必须要 LaserScan。与其加一套传感器（多一份配置、多一份时间
  同步问题），不如从已有的 3D 点云切一刀 —— 零额外硬件成本，而且切出来的
  scan 天然和 3D 点云共享同一时间戳和 frame，不存在外参和同步误差。

  代价要说清楚：切片高度带内的信息被丢弃，2D 建图看不到桌腿以下的障碍。
  这对室内导航是正常的（本来也只关心可通行平面附近的障碍）。

输出 /scan 的 frame 与输入点云一致，角度约定符合 REP-103：
  角度 0 = 机器人正前方 (+x)，逆时针为正。
"""

import math

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy
from sensor_msgs.msg import LaserScan, PointCloud2, PointField

_DATATYPE_TO_NP = {
    PointField.INT8: np.int8, PointField.UINT8: np.uint8,
    PointField.INT16: np.int16, PointField.UINT16: np.uint16,
    PointField.INT32: np.int32, PointField.UINT32: np.uint32,
    PointField.FLOAT32: np.float32, PointField.FLOAT64: np.float64,
}


def cloud_to_xyz(msg: PointCloud2):
    names, formats, offsets = [], [], []
    for f in msg.fields:
        if f.datatype not in _DATATYPE_TO_NP:
            continue
        names.append(f.name)
        formats.append(_DATATYPE_TO_NP[f.datatype])
        offsets.append(f.offset)
    if not all(k in names for k in ("x", "y", "z")):
        return None
    dtype = np.dtype({"names": names, "formats": formats,
                      "offsets": offsets, "itemsize": msg.point_step})
    dtype = dtype.newbyteorder(">" if msg.is_bigendian else "<")
    raw = np.ndarray((msg.height, msg.width), dtype=dtype, buffer=msg.data,
                     strides=(msg.row_step, msg.point_step))
    xyz = np.stack([raw["x"], raw["y"], raw["z"]], axis=-1).reshape(-1, 3).astype(np.float64)
    return xyz[np.isfinite(xyz).all(axis=1)]


class PointsToScan(Node):
    def __init__(self):
        super().__init__("points_to_scan")

        self.declare_parameter("in_topic", "/points")
        self.declare_parameter("out_topic", "/scan")
        # 盲区：丢弃距离传感器过近的点。
        # Gazebo 的 ray sensor 只能忽略【父 link】，无法忽略整车，
        # 因此雷达会打到自己车体顶面（本模型约 0.46m 处）。
        # 若不滤掉，代价地图会在机器人【自身足迹内部】(0.48m)
        # 标出致命障碍 —— DWB 直接报
        #   "BaseObstacle/Trajectory Hits Obstacle"
        # 起步即碰撞，机器人一步都走不了。真机驱动也有同样的盲区处理。
        self.declare_parameter("blind_zone", 0.6)
        self.declare_parameter("num_bins", 720)          # 0.5 度分辨率
        self.declare_parameter("height_min", -0.35)      # 相对雷达的高度带（m）
        self.declare_parameter("height_max", 1.20)
        self.declare_parameter("range_min", 0.30)
        self.declare_parameter("range_max", 40.0)
        self.declare_parameter("min_points_per_bin", 1)

        self._bins = int(self.get_parameter("num_bins").value)
        self._hmin = float(self.get_parameter("height_min").value)
        self._hmax = float(self.get_parameter("height_max").value)
        self._rmin = float(self.get_parameter("range_min").value)
        self._rmax = float(self.get_parameter("range_max").value)
        self._min_pts = int(self.get_parameter("min_points_per_bin").value)
        self._blind = float(self.get_parameter("blind_zone").value)

        self._angles = np.linspace(-math.pi, math.pi, self._bins, endpoint=False)

        qos = QoSProfile(depth=5, history=QoSHistoryPolicy.KEEP_LAST,
                         reliability=QoSReliabilityPolicy.BEST_EFFORT)
        self.create_subscription(
            PointCloud2, self.get_parameter("in_topic").value, self._on_cloud, qos
        )
        self._pub = self.create_publisher(
            LaserScan, self.get_parameter("out_topic").value, 5
        )

        self._miss = 0
        self.get_logger().info(
            f"3D -> 2D 虚拟扫描：{self.get_parameter('in_topic').value}  ->  "
            f"{self.get_parameter('out_topic').value}  "
            f"({self._bins} bins, 高度带 [{self._hmin}, {self._hmax}] m, "
            f"盲区 {self._blind} m)"
        )

    def _on_cloud(self, msg: PointCloud2):
        xyz = cloud_to_xyz(msg)
        if xyz is None or xyz.shape[0] == 0:
            return

        band = (xyz[:, 2] >= self._hmin) & (xyz[:, 2] <= self._hmax)
        p = xyz[band]
        if p.shape[0] == 0:
            self._miss += 1
            return

        r = np.hypot(p[:, 0], p[:, 1])
        # 有效距离下界必须同时满足 range_min 和盲区 blind_zone，取较大者。
        # blind_zone 的作用是滤掉雷达打到【自身车体】的回波 —— 见参数声明处注释。
        # 这一步是代价地图能用的前提：少了它，机器人足迹内部就有致命障碍。
        r_lo = max(self._rmin, self._blind)
        valid = (r >= r_lo) & (r <= self._rmax)
        if not np.any(valid):
            self._miss += 1
            return
        p, r = p[valid], r[valid]

        ang = np.arctan2(p[:, 1], p[:, 0])
        # 角度 -> bin，范围 [-pi, pi)
        idx = np.floor((ang + math.pi) / (2.0 * math.pi) * self._bins).astype(np.int64)
        idx = np.clip(idx, 0, self._bins - 1)

        # 每个 bin 取最近距离
        ranges = np.full(self._bins, np.inf, dtype=np.float64)
        counts = np.zeros(self._bins, dtype=np.int64)
        order = np.argsort(-r)          # 从远到近写，最近的覆盖最后留下
        np.minimum.at(ranges, idx[order], r[order])
        np.add.at(counts, idx, 1)

        enough = counts >= self._min_pts
        out = np.where(np.isfinite(ranges) & enough, ranges, np.inf)
        out = np.where(np.isfinite(out), out, 0.0)   # 0.0 表示无回波，符合 ROS 约定

        scan = LaserScan()
        scan.header = msg.header
        scan.angle_min = -math.pi
        scan.angle_increment = 2.0 * math.pi / self._bins
        scan.angle_max = scan.angle_min + (self._bins - 1) * scan.angle_increment
        scan.time_increment = 0.0
        scan.scan_time = 0.0
        scan.range_min = max(self._rmin, self._blind)
        scan.range_max = self._rmax
        scan.ranges = out.tolist()
        self._pub.publish(scan)


def main():
    rclpy.init()
    node = PointsToScan()
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
