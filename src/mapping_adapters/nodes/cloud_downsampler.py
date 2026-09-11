#!/usr/bin/env python3
"""
cloud_downsampler
=================
把原始点云拆成多个稀疏度层级同时发布，供可视化做纵向对比。

对应需求 #4：「展示...不同稀疏程度的点云」。

为什么值得单独做个节点：
  点云稀疏度直接决定建图精度和实时性，是调参时最需要盯的变量之一。
  但 rviz2 里手动改 VoxelGrid 的 leaf size 只能一次看一层，没法对比。
  这里预先把几档稀疏度算好、同时发布，就能在 rviz2 里叠着看：
  "我砍到 0.5m 体素，地图还剩下多少结构？"

输出：
  /points_filtered   高度带过滤 + 无效点剔除（去地面、去天花板）
  /points_light      voxel 0.10 m  近似原始密度
  /points_medium     voxel 0.25 m  常规建图档位
  /points_heavy      voxel 0.50 m  稀疏档位，用于快速预览大场景
"""

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy
from sensor_msgs.msg import PointCloud2, PointField

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


def make_cloud(header, xyz):
    msg = PointCloud2()
    msg.header = header
    msg.height = 1
    msg.width = int(xyz.shape[0])
    msg.fields = [
        PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
        PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
        PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
    ]
    msg.is_bigendian = False
    msg.point_step = 12
    msg.row_step = 12 * msg.width
    msg.is_dense = True
    msg.data = xyz.astype(np.float32).tobytes()
    return msg


def voxel_downsample(xyz, leaf):
    if xyz.shape[0] == 0:
        return xyz
    keys = np.floor(xyz / leaf).astype(np.int64)
    # 用唯一化拿每体素的第一个点，避免累加平均带来的额外开销
    _, idx = np.unique(keys, axis=0, return_index=True)
    return xyz[idx]


class CloudDownsampler(Node):
    def __init__(self):
        super().__init__("cloud_downsampler")

        self.declare_parameter("in_topic", "/points")
        self.declare_parameter("height_min", -0.35)
        self.declare_parameter("height_max", 1.20)
        self.declare_parameter("leaf_light", 0.10)
        self.declare_parameter("leaf_medium", 0.25)
        self.declare_parameter("leaf_heavy", 0.50)
        self.declare_parameter("publish_levels", True)

        self._hmin = float(self.get_parameter("height_min").value)
        self._hmax = float(self.get_parameter("height_max").value)
        self._leaves = {
            "/points_light": float(self.get_parameter("leaf_light").value),
            "/points_medium": float(self.get_parameter("leaf_medium").value),
            "/points_heavy": float(self.get_parameter("leaf_heavy").value),
        }
        self._levels = bool(self.get_parameter("publish_levels").value)

        qos = QoSProfile(depth=5, history=QoSHistoryPolicy.KEEP_LAST,
                         reliability=QoSReliabilityPolicy.BEST_EFFORT)
        self.create_subscription(
            PointCloud2, self.get_parameter("in_topic").value, self._on_cloud, qos
        )

        self._pub_filtered = self.create_publisher(PointCloud2, "/points_filtered", 5)
        self._pubs = {
            name: self.create_publisher(PointCloud2, name, 5)
            for name in self._leaves
        }
        self._log_n = 0
        self.get_logger().info(
            "点云多稀疏度拆分已启动：/points -> /points_filtered + "
            + " + ".join(f"{k}({v}m)" for k, v in self._leaves.items())
        )

    def _on_cloud(self, msg):
        xyz = cloud_to_xyz(msg)
        if xyz is None or xyz.shape[0] == 0:
            return

        band = (xyz[:, 2] >= self._hmin) & (xyz[:, 2] <= self._hmax)
        xyz = xyz[band]
        if xyz.shape[0] == 0:
            return

        self._pub_filtered.publish(make_cloud(msg.header, xyz))

        if not self._levels:
            return
        counts = []
        for topic, leaf in self._leaves.items():
            ds = voxel_downsample(xyz, leaf)
            self._pubs[topic].publish(make_cloud(msg.header, ds))
            counts.append(f"{leaf}m:{ds.shape[0]}")

        self._log_n += 1
        if self._log_n % 100 == 0:
            self.get_logger().info(
                f"原始 {xyz.shape[0]} 点  ->  " + "  ".join(counts)
            )


def main():
    rclpy.init()
    node = CloudDownsampler()
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
