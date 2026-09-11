#!/usr/bin/env python3
"""
gimbal_stabilizer
=================
虚拟稳定云台：为腿式 / 轮足平台提供一个"俯仰滚转被补偿掉"的雷达 frame。

为什么需要它（这是腿式平台建图的头号真实痛点）：
  四足/轮足抬腿行走时躯干会发生 roll/pitch 抖动，雷达随手抖动，点云出现
  层间错位（scan-to-scan 不一致）。这会让 LIO 的内部运动模型和实际观测错配，
  典型表现是地图出现"重影"或墙壁厚度异常。

  真实机器人靠机械云台或 IMU 辅助去畸变解决。这里做一个等价的虚拟 frame：
    lidar_stabilized 的朝向只保留 base_link 的 yaw，roll/pitch 被去掉。

两个用途：
  1. 作为"已减震机器人"的对照组 —— 点云干净，用来先把算法本身测明白。
  2. 关掉它（use_gimbal:=false）就是"未减震机器人"，两者对比可以直接
     量化出抖动对建图精度的影响。这是变量隔离，不是可有可无的花活。

实现：
  T(base->stabilized) = inv(T(world->base)) * T(world->stabilized)
  其中 stabilized 的世界位姿 = 只保留 yaw 的朝向 + 名义安装偏移。
  lag_alpha < 1.0 时对结果做一阶低通 + 四元数 slerp，模拟真实云台的响应滞后。
"""

import math

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped
from tf2_ros import (
    Buffer,
    ConnectivityException,
    ExtrapolationException,
    LookupException,
    TransformBroadcaster,
    TransformListener,
)


def _rpy_to_quat(roll, pitch, yaw):
    cr, sr = math.cos(roll * 0.5), math.sin(roll * 0.5)
    cp, sp = math.cos(pitch * 0.5), math.sin(pitch * 0.5)
    cy, sy = math.cos(yaw * 0.5), math.sin(yaw * 0.5)
    return [
        sr * cp * cy - cr * sp * sy,
        cr * sp * cy + sr * cp * sy,
        cr * cp * sy - sr * sp * cy,
        cr * cp * cy + sr * sp * sy,
    ]


def _quat_to_rpy(q):
    x, y, z, w = q
    sinr_cosp = 2.0 * (w * x + y * z)
    cosr_cosp = 1.0 - 2.0 * (x * x + y * y)
    roll = math.atan2(sinr_cosp, cosr_cosp)

    sinp = 2.0 * (w * y - z * x)
    sinp = max(-1.0, min(1.0, sinp))
    pitch = math.asin(sinp)

    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    yaw = math.atan2(siny_cosp, cosy_cosp)
    return roll, pitch, yaw


def _quat_mul(a, b):
    ax, ay, az, aw = a
    bx, by, bz, bw = b
    return [
        aw * bx + ax * bw + ay * bz - az * by,
        aw * by - ax * bz + ay * bw + az * bx,
        aw * bz + ax * by - ay * bx + az * bw,
        aw * bw - ax * bx - ay * by - az * bz,
    ]


def _quat_conj(q):
    return [-q[0], -q[1], -q[2], q[3]]


def _quat_rotate(q, v):
    qv = [v[0], v[1], v[2], 0.0]
    res = _quat_mul(_quat_mul(q, qv), _quat_conj(q))
    return [res[0], res[1], res[2]]


def _quat_norm(q):
    n = math.sqrt(sum(c * c for c in q))
    if n < 1e-12:
        return [0.0, 0.0, 0.0, 1.0]
    return [c / n for c in q]


def _slerp(q0, q1, t):
    q0 = _quat_norm(q0)
    q1 = _quat_norm(q1)
    dot = sum(a * b for a, b in zip(q0, q1))
    if dot < 0.0:
        q1 = [-c for c in q1]
        dot = -dot
    if dot > 0.9995:
        return _quat_norm([a + t * (b - a) for a, b in zip(q0, q1)])
    theta0 = math.acos(max(-1.0, min(1.0, dot)))
    theta = theta0 * t
    sin_theta0 = math.sin(theta0)
    s0 = math.sin(theta0 - theta) / sin_theta0
    s1 = math.sin(theta) / sin_theta0
    return _quat_norm([s0 * a + s1 * b for a, b in zip(q0, q1)])


class GimbalStabilizer(Node):
    def __init__(self):
        super().__init__("gimbal_stabilizer")

        self.declare_parameter("enabled", True)
        self.declare_parameter("world_frame", "odom")
        self.declare_parameter("base_frame", "base_link")
        self.declare_parameter("stabilized_frame", "lidar_stabilized")
        self.declare_parameter("mount_xyz", [0.18, 0.0, 0.16])
        self.declare_parameter("mount_rpy", [0.0, 0.0, 0.0])
        self.declare_parameter("publish_rate", 100.0)
        self.declare_parameter("lag_alpha", 1.0)
        self.declare_parameter("scan_topic", "/scan")

        self._enabled = bool(self.get_parameter("enabled").value)
        self._world = self.get_parameter("world_frame").value
        self._base = self.get_parameter("base_frame").value
        self._child = self.get_parameter("stabilized_frame").value
        self._mount_xyz = list(self.get_parameter("mount_xyz").value)
        self._mount_rpy = list(self.get_parameter("mount_rpy").value)
        self._alpha = float(self.get_parameter("lag_alpha").value)
        self._alpha = max(0.0, min(1.0, self._alpha))
        self._scan_topic = self.get_parameter("scan_topic").value

        self._q_mount = _rpy_to_quat(*self._mount_rpy)

        self._buffer = Buffer()
        self._listener = TransformListener(self._buffer, self)
        self._broadcaster = TransformBroadcaster(self)

        self._state_t = None
        self._state_q = None
        self._miss_count = 0
        self._warned = False

        # ---------------- 消费者驱动的时间戳（2026-09-11 关键修复）------
        # 本节点发布的 TF 是给 slam_toolbox / AMCL 消费 /scan 用的。
        # MessageFilter 按【scan 自身的时戳】查询 TF 树，因此
        # 我们必须发布【恰好带 scan 时戳】的 TF —— 而不是自己拍脑袋
        # 选时刻（无论是 now() 还是 get_latest_common_time 都会错位）。
        #
        # 实测数据（tools/tf_stamp_probe.py）：
        #   scan.stamp ≈ 传感器真实采样时刻（比 /clock 还超前 ~70ms）
        #   odom TF 时戳 ≈ scan.stamp - 2ms（几乎同步）
        #   get_latest_common_time ≈ scan.stamp - 100ms（落后一拍）
        # 用 common_time 打时戳 → 每帧 scan 到达时 lidar_stabilized
        # 链都差 100ms 盖不住 scan 时戳 → MessageFilter 全部卡队
        # → 每 2.5s 丢一帧（队列 25 只进不出）→ /map 永远为空。
        #
        # 修法：订阅 /scan，把每帧 scan 的时戳记为"待发布时戳"，
        # 查不到 odom->base_link（差 2~50ms 属正常）就等 100Hz 定时器
        # 重试，查到立即按该时戳发布。
        self._needed = None        # 最近一帧 scan 的时戳（builtin msg）
        self._needed_published = False
        self._scan_count = 0
        self._stale_since = None

        from sensor_msgs.msg import LaserScan  # 仅用 header，局部导入
        self._scan_sub = self.create_subscription(
            LaserScan, self._scan_topic, self._on_scan, 10)

        rate = max(1.0, float(self.get_parameter("publish_rate").value))
        self._timer = self.create_timer(1.0 / rate, self._tick)

        if not self._enabled:
            self.get_logger().info("稳定云台已禁用，不发布补偿 TF。")
        else:
            self.get_logger().info(
                f"稳定云台已启用：{self._base} -> {self._child}"
                f"（挂载 {self._mount_xyz}，lag_alpha={self._alpha}，"
                f"时戳跟随 {self._scan_topic}）"
            )

    def _on_scan(self, msg):
        """消费者驱动：记住 scan 的时戳，并立即尝试发布该时戳的 TF。"""
        if not self._enabled:
            return
        self._scan_count += 1
        self._needed = msg.header.stamp
        self._needed_published = False
        self._publish_for(msg.header.stamp)

    # ------------------------------------------------------------------
    def _nominal_transform(self):
        """TF 查不到时的兜底：直接发布名义安装位姿，保证 TF 树不断。"""
        return list(self._mount_xyz), list(self._q_mount)

    def _compute(self, stamp=None):
        """算出 T(base_link -> lidar_stabilized)。

        ⚠️ 为什么要接受 stamp 参数（实测踩坑，这是幽灵墙的头号来源之一）：
          原来这里写死用 `Time()`（= 最新可用时刻）去查 world->base。
          但本节点发布的 TF 是给【雷达点云】用的，而点云带的是【传感器采样时刻】
          的时间戳。slam_toolbox 会按【点云时间戳】去 TF 树里插值查
          base->lidar_stabilized。于是出现错配：

             · 我们按时刻 T_now 算出的变换，被贴到了时戳 T_scan 的帧上；
             · 机器人以 0.5 m/s 边走边转时，T_now 与 T_scan 之间 base 已经动过，
               导致每帧扫描都被贴上一个「略微错误」的外参；
             · 每帧错一点、方向随机 → 地图上出现【成千上万个小碎块散布在场外】，
               而不是一两个大块（后者才是单次误匹配拖走子图的特征）。

          修法：发布时用「当前采样时刻」算变换是对的，但要保证
          slam_toolbox 按点云时戳查到的，就是那个时刻的正确值。
          这里改为参数化 stamp，让调用方传入明确时刻。

        ⚠️ 2026-09-11 行为变更（重要，关系到 "扩贴错误值"）：
          以前查不到时返回【名义安装位姿】作为兜底。
          那在 scan 驱动的架构下是【有害】的 —— 调用方无法区分
          "查不到，下次重试" 和 "查到了，值就是这个"。
          结果 scan 时戳 T 在 odom 还没盖住时被贴上一个
          【按别的时刻算出来的名义值】并发出去，下游拿到的就是
          时戳 T、但内容错误的 TF（比不发布更坏：不发布还能等到
          重试后拿到正确值，错误值一旦发布就再也无法纠正）。
          现在改为返回 (None, None)：调用方 (_publish_for)
          据此判断"还没查到"，由 100Hz 定时器继续重试。
        """
        try:
            tf = self._buffer.lookup_transform(
                self._world, self._base, stamp if stamp is not None
                else rclpy.time.Time()
            )
        except Exception as e:
            # ⚠️ 必须 catch-all，不能只 catch 三个 tf2 异常类型。
            #    这个函数跑在 100Hz 定时器里，任何漏网的异常都会让
            #    rclpy 打死整个节点；而本节点一旦死掉，lidar_stabilized
            #    这个 frame 就不存在了，下游 slam_toolbox 会把【每一帧】
            #    扫描都丢掉。代价远大于"少发布一帧 TF"。
            self._miss_count += 1
            if self._miss_count > 200 and not self._warned:
                self.get_logger().warn(
                    f"连续 {self._miss_count} 次查不到 {self._world}->{self._base}"
                    f"（{type(e).__name__}: {e}）。检查里程计是否在发布 TF。"
                )
                self._warned = True
            return None, None

        self._miss_count = 0
        self._warned = False

        t = tf.transform.translation
        r = tf.transform.rotation
        q_wb = _quat_norm([r.x, r.y, r.z, r.w])
        p_wb = [t.x, t.y, t.z]

        # 只保留 yaw：这一步就是"云台补偿"的本质 —— 去掉躯干 roll/pitch
        _, _, yaw = _quat_to_rpy(q_wb)
        q_yaw = _rpy_to_quat(0.0, 0.0, yaw)

        # stabilized 的世界位姿
        q_ws = _quat_mul(q_yaw, self._q_mount)
        off_w = _quat_rotate(q_yaw, self._mount_xyz)
        p_ws = [p_wb[i] + off_w[i] for i in range(3)]

        # T(base->stabilized) = inv(T(world->base)) * T(world->stabilized)
        q_bw = _quat_conj(q_wb)
        q_bs = _quat_norm(_quat_mul(q_bw, q_ws))
        p_bs = _quat_rotate(q_bw, [p_ws[i] - p_wb[i] for i in range(3)])

        return p_bs, q_bs

    def _apply_lag(self, p, q):
        if self._alpha >= 1.0 or self._state_t is None:
            self._state_t = list(p)
            self._state_q = list(q)
            return p, q
        a = self._alpha
        t = [a * p[i] + (1.0 - a) * self._state_t[i] for i in range(3)]
        qq = _slerp(self._state_q, q, a)
        self._state_t = list(t)
        self._state_q = list(qq)
        return t, qq

    def _tick(self):
        if not self._enabled:
            return
        try:
            self._tick_impl()
            # 成功清零：只要恢复正常就不该再刷错误
            if getattr(self, "_crash_count", 0):
                self.get_logger().info(
                    f"_tick 已恢复正常（此前累计异常 {self._crash_count} 次）")
                self._crash_count = 0
        except Exception as e:
            # 最后一道防线：任何异常都不允许打死节点。
            # 见 _publish_for 里关于「节点一死 → lidar_stabilized 消失 →
            # slam_toolbox 丢光所有帧」的说明。
            #
            # ⚠️⚠️ 日志策略是【持续刷】而不是只报前 3 次 —— 这是用血换来的：
            #   上一版写 `if self._crash_count <= 3`，结果这个异常在 100Hz 定时器里
            #   每秒抛 100 次，日志只留 3 行就彻底静音了。我因此误判"修复已生效"，
            #   白跑了一轮 5 分钟 smoke 体检。
            #   静默降级比直接崩溃危险得多：崩溃至少会留下 exit code。
            #   所以这里改成「前 3 次每次报 + 之后每 100 次（约 1s）报一次」，
            #   既不会刷爆日志，也【永远不会静音】。
            self._crash_count = getattr(self, "_crash_count", 0) + 1
            if self._crash_count <= 3 or self._crash_count % 100 == 0:
                self.get_logger().error(
                    f"_tick 内部异常（累计第 {self._crash_count} 次）："
                    f"{type(e).__name__}: {e}"
                    f"  ← 本节点负责发布 {self._base} -> {self._child}，"
                    f"持续异常将导致该 frame 缺失、下游建图全废，必须修。")

    def _tick_impl(self):
        """100Hz 定时器：重试还没发出去的 scan 时戳；首帧 scan 之前发保底 TF。

        时戳策略的三个版本（演进史，别再走回头路）：
          v1  now() 打时戳 + Time(0) 查位姿 —— 时戳与位姿错配，
              运动时每帧外参错一点 → 幽灵墙（累积漂移型）。
          v2  get_latest_common_time 打时戳 —— 位姿与时戳一致了，
              但比 scan 时戳落后 ~100ms → MessageFilter 每帧卡队
              → 全部丢弃 → /map 永远为空（本轮踩的最大坑）。
          v3  【当前】scan 驱动：消费者要什么时戳就发什么时戳。
              收到 /scan 记下它的时戳，按该时戳查 odom->base_link
              （查不到很正常：odom 比传感器晚 2~50ms），100Hz 重试，
              查到后按该时戳发布。位姿与时戳严格一致，且与消费者
              的查询时刻精确对齐。
        """
        if self._needed is not None:
            if not self._needed_published:
                self._publish_for(self._needed)
            else:
                self._stale_since = None
            # 看门狗：scan 在流、但我们一直发不出去 → 必须【可见】。
            # 前文所有故障的共性就是"静默失败"，所以这里宁可刷日志。
            if not self._needed_published:
                if self._stale_since is None:
                    self._stale_since = self.get_clock().now()
                elif (self.get_clock().now()
                      - self._stale_since).nanoseconds > 2e9:
                    self.get_logger().warn(
                        f"已 2s 无法按 {self._scan_topic} 时戳发布 "
                        f"{self._base} -> {self._child}：通常是 odom TF 停止更新，"
                        f"或 scan 时戳超出 TF 覆盖范围。下游建图会丢帧。")
                    self._stale_since = self.get_clock().now()
            return

        # 还没收到任何 scan（启动初期 / 传感器未就绪）：
        # 用名义位姿 + now() 把 frame 撑起来，否则 frame 不存在，
        # tf2_echo / mapping_contract_adapter 全都查不到。
        p, q = self._nominal_transform()
        p, q = self._apply_lag(p, q)
        msg = TransformStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self._base
        msg.child_frame_id = self._child
        msg.transform.translation.x = float(p[0])
        msg.transform.translation.y = float(p[1])
        msg.transform.translation.z = float(p[2])
        msg.transform.rotation.x = float(q[0])
        msg.transform.rotation.y = float(q[1])
        msg.transform.rotation.z = float(q[2])
        msg.transform.rotation.w = float(q[3])
        self._broadcaster.sendTransform(msg)

        self._sent_count = getattr(self, "_sent_count", 0) + 1
        if self._sent_count == 1:
            self.get_logger().info(
                f"✅ 保底 TF 已开始发布：{self._base} -> {self._child}"
                f"（等首帧 {self._scan_topic} 到达后切换为 scan 时戳驱动）")

    def _publish_for(self, stamp_msg):
        """按【scan 的时戳】计算并发布 base_link -> lidar_stabilized。

        查不到 odom->base_link 时静默返回（等 _tick 重试）——这不是错误：
        scan 时戳通常比最新 odom TF 超前 2~50ms，属正常竞态。
        """
        # ⚠️⚠️ rclpy.time.Time 的构造签名是
        #   Time(*, seconds=0, nanoseconds=0, clock_type=...)
        # 没有 stamp / sec / nanosec 关键字（.sec/.nanosec 是 msg 的字段）。
        # 这里连环踩过 is_zero / .sec 两个 AttributeError，都让节点
        # "活着但从不发 frame"，详见 _tick_impl 的版本演进注释。
        t = rclpy.time.Time(
            seconds=stamp_msg.sec, nanoseconds=stamp_msg.nanosec)

        p, q = self._compute(t)
        if p is None:
            # _compute 里 lookup 失败（odom 还没盖住这个时戳），下次重试
            return
        p, q = self._apply_lag(p, q)

        msg = TransformStamped()
        # 关键：时戳【原样】使用 scan 的时戳，保证 MessageFilter
        # 按 scan 时戳查询时精确命中
        msg.header.stamp = stamp_msg
        msg.header.frame_id = self._base
        msg.child_frame_id = self._child
        msg.transform.translation.x = float(p[0])
        msg.transform.translation.y = float(p[1])
        msg.transform.translation.z = float(p[2])
        msg.transform.rotation.x = float(q[0])
        msg.transform.rotation.y = float(q[1])
        msg.transform.rotation.z = float(q[2])
        msg.transform.rotation.w = float(q[3])
        self._broadcaster.sendTransform(msg)
        self._needed_published = True

        # 首帧发出后的【事后记账】：显式证明"这个 frame 真的发出去了"。
        # 动机：之前故障的全部表现都是"日志看着正常但 frame 不存在"，
        # 所以必须有一条正面证据。只报一次，不刷屏。
        self._sent_count = getattr(self, "_sent_count", 0) + 1
        if self._sent_count == 1:
            self.get_logger().info(
                f"✅ 已按 scan 时戳发布 TF：{self._base} -> {self._child}"
                f"（首帧 t={stamp_msg.sec}.{stamp_msg.nanosec:09d}，"
                f"位移=({p[0]:.4f},{p[1]:.4f},{p[2]:.4f})）")


def main():
    rclpy.init()
    node = GimbalStabilizer()
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
