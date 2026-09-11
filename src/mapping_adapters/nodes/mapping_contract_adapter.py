#!/usr/bin/env python3
"""
mapping_contract_adapter
========================
把「任意建图算法的输出」规范化成接口契约。

这是整个适配层的地基。它解决的问题是：
  FAST-LIO2 输出的是 livox_ros_driver2/CustomMsg 或者 sensor_msgs/PointCloud2，
  挂在 camera_init / body frame 下；LIO-SAM 挂在 map / velodyne 下；
  RTAB-Map 输出 /map 但可能不输出点云。你不可能去改它们每一个。

  所以在这里做一次归一化：
    输入：任意 frame、任意点云类型、任意话题名
    输出：固定的 /map_cloud（map frame）+ /map（OccupancyGrid）

做完这一步，下游的 rviz2 配置、指标节点、benchmark runner 全都只需要认
契约话题，不需要知道跑的是哪个算法。

2D 投影用 ray-cast 更新空闲空间，不是简单把点打成占据格 —— 后者会把所有
未观测区域留在 unknown，覆盖率指标就没法算了。
"""

import numpy as np
import rclpy
from nav_msgs.msg import OccupancyGrid
from rclpy.node import Node
from rclpy.qos import QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy
from rclpy.time import Time
from sensor_msgs.msg import PointCloud2, PointField
from tf2_ros import Buffer, TransformException, TransformListener

try:
    from tf2_sensor_msgs import do_transform_cloud

    _HAS_TF2_MSG = True
except ImportError:  # 退化路径：只归一化话题名，不做坐标变换
    _HAS_TF2_MSG = False

_DATATYPE_TO_NP = {
    PointField.INT8: np.int8,
    PointField.UINT8: np.uint8,
    PointField.INT16: np.int16,
    PointField.UINT16: np.uint16,
    PointField.INT32: np.int32,
    PointField.UINT32: np.uint32,
    PointField.FLOAT32: np.float32,
    PointField.FLOAT64: np.float64,
}


def cloud_to_xyz(msg: PointCloud2):
    """把 PointCloud2 解成 (N,3) float64 数组。

    用手工构造 dtype 的方式解析，好处是能正确处理 point_step 里的 padding
    （很多算法输出的点云字段之间有填充，直接 reshape 会读到垃圾值）。
    """
    names, formats, offsets = [], [], []
    for f in msg.fields:
        if f.datatype not in _DATATYPE_TO_NP:
            continue
        names.append(f.name)
        formats.append(_DATATYPE_TO_NP[f.datatype])
        offsets.append(f.offset)

    if not all(k in names for k in ("x", "y", "z")):
        return None

    dtype = np.dtype({
        "names": names,
        "formats": formats,
        "offsets": offsets,
        "itemsize": msg.point_step,
    })
    dtype = dtype.newbyteorder(">" if msg.is_bigendian else "<")
    raw = np.ndarray((msg.height, msg.width), dtype=dtype, buffer=msg.data,
                     strides=(msg.row_step, msg.point_step))
    xyz = np.stack([raw["x"], raw["y"], raw["z"]], axis=-1).reshape(-1, 3).astype(np.float64)

    finite = np.isfinite(xyz).all(axis=1)
    return xyz[finite]


class MappingContractAdapter(Node):
    def __init__(self):
        super().__init__("mapping_contract_adapter")

        # ---- 输入：算法侧（按实际算法改这两个参数即可）----
        self.declare_parameter("in_cloud_topic", "/cloud_registered")
        self.declare_parameter("in_odom_topic", "")
        self.declare_parameter("cloud_qos_reliable", False)

        # ---- 输出：契约侧 ----
        self.declare_parameter("out_cloud_topic", "/map_cloud")
        self.declare_parameter("out_downsampled_topic", "/map_cloud_downsampled")
        self.declare_parameter("out_map_topic", "/map")
        self.declare_parameter("map_frame", "map")
        self.declare_parameter("robot_frame", "base_link")

        # ---- 2D 投影 ----
        self.declare_parameter("publish_2d_map", True)
        self.declare_parameter("grid_size_m", 60.0)
        self.declare_parameter("grid_resolution", 0.05)
        self.declare_parameter("height_min", -0.5)     # 参与投影的高度带（map frame 下的 z）
        self.declare_parameter("height_max", 2.0)
        self.declare_parameter("ray_samples", 24)      # 每条光线的空闲空间采样数
        self.declare_parameter("occ_hit_thresh", 3)    # 命中几次才算占据
        self.declare_parameter("map_publish_period", 2.0)

        # ---- 降采样（可视化稀疏度对比用）----
        self.declare_parameter("voxel_leaf", 0.25)

        self._map_frame = self.get_parameter("map_frame").value
        self._robust = not bool(self.get_parameter("cloud_qos_reliable").value)
        self._publish_2d = bool(self.get_parameter("publish_2d_map").value)
        self._res = float(self.get_parameter("grid_resolution").value)
        self._size = float(self.get_parameter("grid_size_m").value)
        self._hmin = float(self.get_parameter("height_min").value)
        self._hmax = float(self.get_parameter("height_max").value)
        self._ray_samples = int(self.get_parameter("ray_samples").value)
        self._occ_thresh = int(self.get_parameter("occ_hit_thresh").value)
        self._leaf = float(self.get_parameter("voxel_leaf").value)

        self._n = int(round(self._size / self._res))
        self._occ = np.zeros((self._n, self._n), dtype=np.uint16)
        self._free = np.zeros((self._n, self._n), dtype=np.uint16)

        self._buffer = Buffer()
        self._listener = TransformListener(self._buffer, self)

        qos = QoSProfile(
            depth=5,
            history=QoSHistoryPolicy.KEEP_LAST,
            reliability=QoSReliabilityPolicy.BEST_EFFORT
            if self._robust
            else QoSReliabilityPolicy.RELIABLE,
        )

        self._pub_cloud = self.create_publisher(
            PointCloud2, self.get_parameter("out_cloud_topic").value, 5
        )
        self._pub_down = self.create_publisher(
            PointCloud2, self.get_parameter("out_downsampled_topic").value, 5
        )
        # 只有真的要用 2D 投影时才创建 /map 发布者。
        # 无条件创建会踩到这个坑：本节点发的是 VOLATILE，而 Nav2 的
        # global_costmap static_layer 用 TRANSIENT_LOCAL 订阅（它要拿
        # map_server / slam_toolbox 的锁存地图）。两者 QoS 不兼容，
        # Nav2 会打印
        #   "New publisher discovered on topic '/map', offering incompatible QoS"
        # 并【一帧地图都收不到】—— 代价地图是空的，全局规划自然全失败。
        self._pub_map = None
        if self._publish_2d:
            self._pub_map = self.create_publisher(
                OccupancyGrid, self.get_parameter("out_map_topic").value,
                QoSProfile(depth=1, history=QoSHistoryPolicy.KEEP_LAST,
                           reliability=QoSReliabilityPolicy.RELIABLE),
            )

        in_topic = self.get_parameter("in_cloud_topic").value
        self.create_subscription(PointCloud2, in_topic, self._on_cloud, qos)

        period = float(self.get_parameter("map_publish_period").value)
        self.create_timer(max(0.2, period), self._publish_map)

        self._warned_tf = False
        self._count = 0
        self.get_logger().info(
            f"契约适配器就绪：{in_topic}  ->  "
            f"{self.get_parameter('out_cloud_topic').value} / "
            f"{self.get_parameter('out_map_topic').value}"
            f"（{'含 2D 投影' if self._publish_2d else '仅点云归一化'}）"
        )
        if not _HAS_TF2_MSG:
            self.get_logger().error(
                "未找到 tf2_sensor_msgs，无法做点云坐标变换，只能做话题名归一化。"
                "请安装：sudo apt install ros-$ROS_DISTRO-tf2-sensor-msgs"
            )

    # ------------------------------------------------------------------
    def _transform_to_map(self, msg: PointCloud2):
        if msg.header.frame_id == self._map_frame:
            return msg
        if not _HAS_TF2_MSG:
            return msg
        try:
            tf = self._buffer.lookup_transform(
                self._map_frame, msg.header.frame_id, Time()
            )
        except TransformException:
            if not self._warned_tf:
                self.get_logger().warn(
                    f"暂时查不到 {self._map_frame} -> {msg.header.frame_id}，"
                    "跳过本帧。若持续出现，说明建图算法还没发布 map->odom（正常，"
                    "算法刚启动时就是这样）。"
                )
                self._warned_tf = True
            return None
        return do_transform_cloud(msg, tf)

    def _sensor_origin_in_map(self, frame_id):
        try:
            tf = self._buffer.lookup_transform(self._map_frame, frame_id, Time())
        except TransformException:
            return None
        t = tf.transform.translation
        return np.array([t.x, t.y], dtype=np.float64)

    def _on_cloud(self, msg: PointCloud2):
        out = self._transform_to_map(msg)
        if out is None:
            return
        self._pub_cloud.publish(out)

        xyz = cloud_to_xyz(out)
        if xyz is None or xyz.shape[0] == 0:
            return

        # 降采样点云（可视化稀疏度对比用）
        self._pub_down.publish(self._make_downsampled(out, xyz))

        if not self._publish_2d:
            return

        band = (xyz[:, 2] >= self._hmin) & (xyz[:, 2] <= self._hmax)
        pts = xyz[band][:, :2]
        if pts.shape[0] == 0:
            return

        origin = self._sensor_origin_in_map(out.header.frame_id)
        if origin is None:
            origin = np.zeros(2, dtype=np.float64)

        self._accumulate(pts, origin)
        self._count += 1

    def _make_downsampled(self, ref_msg: PointCloud2, xyz):
        vox = np.floor(xyz / self._leaf).astype(np.int64)
        _, idx = np.unique(vox, axis=0, return_index=True)
        sel = xyz[idx]

        out = PointCloud2()
        out.header = ref_msg.header
        out.height = 1
        out.width = sel.shape[0]
        out.fields = [
            PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
        ]
        out.is_bigendian = False
        out.point_step = 12
        out.row_step = 12 * sel.shape[0]
        out.is_dense = True
        out.data = sel.astype(np.float32).tobytes()
        return out

    def _to_cell(self, xy):
        ix = np.floor(xy[:, 0] / self._res).astype(np.int64) + self._n // 2
        iy = np.floor(xy[:, 1] / self._res).astype(np.int64) + self._n // 2
        valid = (ix >= 0) & (ix < self._n) & (iy >= 0) & (iy < self._n)
        return ix[valid], iy[valid]

    def _accumulate(self, pts, origin):
        # 占据：命中计数
        ix, iy = self._to_cell(pts)
        if ix.size:
            np.add.at(self._occ, (iy, ix), 1)

        # 空闲：沿射线采样打 miss。
        # 这一步不能省 —— 只打占据点的话，所有观测过的空闲区域都会停留在
        # unknown，跟 GT 一比覆盖率就是一片惨淡，评测直接失真。
        o = origin.reshape(1, 2)
        d = pts - o
        for k in range(1, self._ray_samples + 1):
            t = k / float(self._ray_samples + 1)
            samples = o + t * d
            si, sj = self._to_cell(samples)
            if si.size:
                np.add.at(self._free, (sj, si), 1)

    def _publish_map(self):
        if self._pub_map is None or self._occ.sum() == 0:
            return

        data = np.full((self._n, self._n), -1, dtype=np.int8)
        occupied = (self._occ >= self._occ_thresh) & (self._occ > self._free)
        observed_free = (self._free > 0) & ~occupied
        data[observed_free] = 0
        data[occupied] = 100

        msg = OccupancyGrid()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self._map_frame
        msg.info.resolution = self._res
        msg.info.width = self._n
        msg.info.height = self._n
        half = self._size / 2.0
        msg.info.origin.position.x = -half
        msg.info.origin.position.y = -half
        msg.info.origin.position.z = 0.0
        msg.info.origin.orientation.w = 1.0
        # OccupancyGrid 的数据行序：第 0 行 = 最小 y（与 PGM 相反，别搞混）
        msg.data = data.flatten().tolist()
        self._pub_map.publish(msg)


def main():
    rclpy.init()
    node = MappingContractAdapter()
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
