#!/usr/bin/env python3
"""
test_gimbal_unit.py —— gimbal_stabilizer 的【离线单元测试】
================================================================
动机（2026-09-11 连环翻车）：
  gimbal_stabilizer 里连续两处 rclpy.time.Time 的 API 误用
  （`.is_zero` 和 `.sec`/`.nanosec`），两处都抛 AttributeError。
  因为异常被 _tick 外壳接住，节点不崩、日志只有 3 行就静音了，
  结果「节点活着但从不发布 frame」→ slam_toolbox 丢光所有帧
  → 地图永远是空的，而话题频率看起来完全正常。

  代价：为了定位这一个 bug，跑了 2 轮 20 分钟探索 + 3 轮 smoke 体检。

  所以必须有一个【秒级】的离线测试，把这类错误挡在仿真之前。
  这个脚本不启动任何 ROS 仿真，只做两件事：
    ① 静态 AST 检查：扫描脚本里对 rclpy.time.Time 对象的属性访问，
       凡是不在 Time 白名单里的属性名一律报错。
    ② 动态行为测试：用伪造的 TF buffer 驱动 _tick_impl，
       确认在各种输入下都能走到 sendTransform（而不是中途抛异常）。

用法：
    python3 tools/test_gimbal_unit.py
退出码 0 = 全过；非 0 = 有问题，先修再跑仿真。
"""

import ast
import importlib.util
import os
import sys
import types

WS = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
TARGET = os.path.join(WS, "src", "robot_descriptions",
                      "scripts", "gimbal_stabilizer.py")

# rclpy.time.Time 真实存在的公开属性（2026-09-11 实测确认）
TIME_ALLOWED = {
    "clock_type", "from_msg", "nanoseconds", "seconds_nanoseconds", "to_msg",
}

FAIL = []


def check(label, ok, detail=""):
    mark = "✅" if ok else "❌"
    print(f"  {mark} {label}" + (f"  —— {detail}" if detail else ""))
    if not ok:
        FAIL.append(label)


# ---------------------------------------------------------------- ① 静态检查
def static_audit():
    print("\n① 静态 AST 检查：rclpy.time.Time 的属性访问")
    with open(TARGET, "r", encoding="utf-8") as f:
        src = f.read()

    # ⚠️ 检查逻辑必须分清两种"Time"，否则会误报：
    #
    #   rclpy.time.Time             —— 属性只有 clock_type / from_msg /
    #                                  nanoseconds / seconds_nanoseconds / to_msg
    #   builtin_interfaces.msg.Time —— 属性是 sec / nanosec（header.stamp 就属这种）
    #
    # 所以不能"凡是叫 stamp 的变量访问 .sec 就报错"。正确做法：
    #   追踪【哪些变量持有 rclpy.time.Time】（构造或 get_latest_common_time 的返回值），
    #   只对这些变量检查属性白名单。
    #   is_zero 对两者都非法（那是 C++ tf2 / ROS1 的用法），全局禁止。
    TIME_ALLOWED_ATTRS = {
        "clock_type", "from_msg", "nanoseconds", "seconds_nanoseconds",
        "to_msg",
    }

    tree = ast.parse(src, filename=TARGET)

    def _name_of(node):
        """取表达式的基础名字用于启发式匹配。"""
        if isinstance(node, ast.Name):
            return node.id
        if isinstance(node, ast.Attribute):
            return node.attr
        return ""

    time_vars = set()
    for node in ast.walk(tree):
        if not isinstance(node, ast.Assign):
            continue
        val = node.value
        is_time = False
        if isinstance(val, ast.Call):
            fn = val.func
            base = ""
            if isinstance(fn, ast.Attribute):
                base = fn.attr
            elif isinstance(fn, ast.Name):
                base = fn.id
            if base in ("Time", "get_latest_common_time",
                        "seconds_nanoseconds") or base.startswith("from_msg"):
                is_time = True
        if not is_time:
            continue
        for tgt in node.targets:
            if isinstance(tgt, ast.Name):
                time_vars.add(tgt.id)

    hits = []
    for node in ast.walk(tree):
        if not isinstance(node, ast.Attribute):
            continue
        attr = node.attr
        # is_zero 在任何 Time 上都不存在 → 全局报错
        if attr == "is_zero":
            hits.append((node.lineno, _name_of(node.value), attr))
            continue
        if attr in ("sec", "nanosec", "secs", "nsecs"):
            if isinstance(node.value, ast.Name) and \
               node.value.id in time_vars:
                hits.append((node.lineno, node.value.id, attr))

    check("无 rclpy.time.Time 属性误用", not hits,
          "；".join(f"L{l}: {b}.{a}" for l, b, a in hits))

    # 额外：确认源码里确实用了 to_msg / seconds_nanoseconds 这类正确 API
    check("使用了合法 API（to_msg）", ".to_msg()" in src)


# ---------------------------------------------------------------- ② 动态测试
def load_module_with_fake_ros():
    """把 gimbal_stabilizer.py 加载进来，但用假的 rclpy/tf2_ros 顶替。

    目的：不启动 ROS master、不依赖 DDS，纯 CPU 跑逻辑。
    """
    fake_rclpy = types.ModuleType("rclpy")

    class _Time:
        """复刻真实 rclpy.time.Time 的属性面（只有 5 个公开成员）。"""
        __slots__ = ("_ns", "clock_type")

        def __init__(self, seconds=0, nanoseconds=0, clock_type=0):
            self._ns = int(seconds * 1e9 + nanoseconds)
            self.clock_type = clock_type

        @property
        def nanoseconds(self):
            return self._ns

        def seconds_nanoseconds(self):
            return (self._ns // 1_000_000_000, self._ns % 1_000_000_000)

        def to_msg(self):
            sec, nsec = self.seconds_nanoseconds()
            msg = types.SimpleNamespace(sec=sec, nanosec=nsec)
            return msg

        def is_zero(self):  # noqa: F811
            raise AssertionError("真 Time 没有 is_zero，不该被调用到")

    def _init(*a, **k):
        pass

    def _ok(*a, **k):
        return True

    fake_rclpy.init = _init
    fake_rclpy.ok = _ok
    fake_rclpy.time = types.SimpleNamespace(Time=_Time)
    fake_rclpy.node = types.ModuleType("rclpy.node")

    class FakeLogger:
        def __init__(self, sink):
            self.sink = sink

        def info(self, m):
            self.sink.append(("INFO", m))

        def warn(self, m):
            self.sink.append(("WARN", m))

        def error(self, m):
            self.sink.append(("ERROR", m))

    class FakeClock:
        def now(self):
            return _Time(seconds=100.0)

    class Node:
        def __init__(self, name):
            self._node_name = name
            self.messages = []
            self._logger = FakeLogger(self.messages)
            self._params = {}

        def get_logger(self):
            return self._logger

        def get_clock(self):
            return FakeClock()

        def declare_parameter(self, name, default=None):
            # 复刻 rclpy 语义：声明时给初值，get 时原样取回
            self._params[name] = default
            return types.SimpleNamespace(value=default)

        def get_parameter(self, name):
            if name not in self._params:
                raise KeyError(f"参数未声明：{name}")
            return types.SimpleNamespace(value=self._params[name])

        def create_timer(self, period, callback):
            # 保存回调，测试里可以手动驱动（不真的跑定时器）
            self._timer_cb = callback
            return None

        def create_subscription(self, msg_type, topic, cb, depth):
            self._scan_cb = cb
            self._scan_topic = topic
            return None

    fake_rclpy.node.Node = Node

    # tf2_ros
    fake_tf = types.ModuleType("tf2_ros")

    class Buffer:
        def __init__(self, *a, **k):
            self.mode = "ok"
            self.latest_ns = 20 * 1_000_000_000  # stale 模式下 TF 覆盖到 20s
            self.calls = []

        def get_latest_common_time(self, target, source):
            self.calls.append(("glct", target, source))
            if self.mode == "raise":
                raise RuntimeError(
                    'tf2.LookupException: "odom" passed to '
                    "get_latest_common_time does not exist")
            return _Time(seconds=12.5)

        def lookup_transform(self, target, source, stamp=None):
            self.calls.append(("lookup", target, source, stamp))
            if self.mode == "raise":
                raise RuntimeError("tf2.LookupException: no transform")
            if self.mode == "stale":
                # 模拟「odom 还没盖住 scan 时戳」：
                #   比 record_latest_stamp 更新的一律查不到，更老的才给
                #   （对应 scan 时戳超前 odom 2~50ms 的正常竞态）
                if stamp is not None and stamp.nanoseconds > self.latest_ns:
                    raise RuntimeError("tf2.ExtrapolationException: into future")
            return types.SimpleNamespace(
                transform=types.SimpleNamespace(
                    translation=types.SimpleNamespace(x=1.0, y=2.0, z=0.4),
                    rotation=types.SimpleNamespace(
                        x=0.0, y=0.0, z=0.1, w=0.9949874371),
                )
            )

    class TransformBroadcaster:
        def __init__(self, *a, **k):
            self.sent = []

        def sendTransform(self, msg):
            self.sent.append(msg)

    class TransformListener:
        def __init__(self, *a, **k):
            pass

    class _Exc(Exception):
        pass

    fake_tf.Buffer = Buffer
    fake_tf.TransformBroadcaster = TransformBroadcaster
    fake_tf.TransformListener = TransformListener
    fake_tf.LookupException = _Exc
    fake_tf.ConnectivityException = _Exc
    fake_tf.ExtrapolationException = _Exc

    # geometry_msgs.msg.TransformStamped —— 只需要能被实例化并挂字段
    fake_gm = types.ModuleType("geometry_msgs")
    fake_gm_msg = types.ModuleType("geometry_msgs.msg")

    class _V3:
        def __init__(self):
            self.x = 0.0
            self.y = 0.0
            self.z = 0.0

    class _Q:
        def __init__(self):
            self.x = 0.0
            self.y = 0.0
            self.z = 0.0
            self.w = 1.0

    class _Header:
        def __init__(self):
            self.stamp = None
            self.frame_id = ""

    class TransformStamped:
        def __init__(self):
            self.header = _Header()
            self.child_frame_id = ""
            self.transform = types.SimpleNamespace(
                translation=_V3(), rotation=_Q())

    fake_gm_msg.TransformStamped = TransformStamped
    fake_gm.msg = fake_gm_msg

    # sensor_msgs.msg.LaserScan —— 只需要 header.stamp
    fake_sm = types.ModuleType("sensor_msgs")
    fake_sm_msg = types.ModuleType("sensor_msgs.msg")

    class LaserScan:
        def __init__(self):
            self.header = types.SimpleNamespace(stamp=None)

    fake_sm_msg.LaserScan = LaserScan
    fake_sm.msg = fake_sm_msg

    sys.modules["rclpy"] = fake_rclpy
    sys.modules["rclpy.node"] = fake_rclpy.node
    sys.modules["tf2_ros"] = fake_tf
    sys.modules["geometry_msgs"] = fake_gm
    sys.modules["geometry_msgs.msg"] = fake_gm_msg
    sys.modules["sensor_msgs"] = fake_sm
    sys.modules["sensor_msgs.msg"] = fake_sm_msg

    # rclpy.time 需要在包级别被 import 到（脚本里用 rclpy.time.Time()）
    spec = importlib.util.spec_from_file_location("gimbal_under_test", TARGET)
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod, fake_rclpy, fake_tf


def _make_scan(sec, nanosec=0):
    """造一帧假 scan（只需要 header.stamp）。"""
    s = types.SimpleNamespace(
        sec=sec, nanosec=nanosec,
        __class__=types.SimpleNamespace(__name__="Time"))

    class _Scan:
        def __init__(self):
            self.header = types.SimpleNamespace(stamp=s)

    return _Scan()


def dynamic_test():
    print("\n② 动态行为测试：伪造 TF buffer 驱动 _tick")
    try:
        mod, fake_rclpy, fake_tf = load_module_with_fake_ros()
    except Exception as e:  # noqa: BLE001
        check("模块可加载", False, f"{type(e).__name__}: {e}")
        return

    check("模块可加载（无 import 期异常）", True)

    # --- 场景 A：收到 scan(T=12.5)，TF 覆盖 T → 立即按 T 发布 ---
    try:
        node = mod.GimbalStabilizer()
        node._buffer.mode = "ok"
        scan = _make_scan(12, 500000000)
        node._scan_cb(scan)          # 模拟 /scan 到达
        n_sent = len(node._broadcaster.sent)
        check("场景A（scan 到达且 TF 就绪）发出 TF", n_sent == 1,
              f"sendTransform 调用 {n_sent} 次")
        if n_sent:
            st = node._broadcaster.sent[0].header.stamp
            check("场景A 时戳精确等于 scan 时戳（12.500000000）",
                  st.sec == 12 and st.nanosec == 500000000,
                  f"stamp={st.sec}.{st.nanosec:09d}")
        errs = [m for lv, m in node.messages if lv == "ERROR"]
        check("场景A 无 ERROR 日志", not errs, "；".join(errs[:2]))
        sent_logs = [m for lv, m in node.messages if lv == "INFO"
                     and "已按 scan 时戳发布 TF" in m]
        check("场景A 有『已按 scan 时戳发布』正面证据", len(sent_logs) == 1,
              sent_logs[0][:60] if sent_logs else "缺失")
    except Exception as e:  # noqa: BLE001
        check("场景A（scan 到达且 TF 就绪）发出 TF", False,
              f"{type(e).__name__}: {e}")

    # --- 场景 B：启动期尚无 scan → 保底 TF（撑住 frame 不被查成不存在）---
    try:
        node = mod.GimbalStabilizer()
        node._buffer.mode = "ok"
        node._timer_cb()             # 模拟 100Hz 定时器
        n_sent = len(node._broadcaster.sent)
        check("场景B（尚无 scan）发出保底 TF", n_sent == 1,
              f"sendTransform 调用 {n_sent} 次")
        if n_sent:
            st = node._broadcaster.sent[0].header.stamp
            check("场景B 保底用时戳 now（100s）",
                  st.sec == 100, f"stamp={st.sec}.{st.nanosec:09d}")
    except Exception as e:  # noqa: BLE001
        check("场景B（尚无 scan）发出保底 TF", False,
              f"{type(e).__name__}: {e}")

    # --- 场景 C：scan 时戳超出 TF 覆盖 → 不发布，等定时器重试后补齐 ---
    try:
        node = mod.GimbalStabilizer()
        node._buffer.mode = "stale"     # 只覆盖 ≤20s
        scan = _make_scan(25, 0)
        node._scan_cb(scan)
        check("场景C scan 时戳未被覆盖时不发布（避免贴错值）",
              len(node._broadcaster.sent) == 0,
              f"实际发了 {len(node._broadcaster.sent)} 帧")
        # 时间推进：TF 覆盖扩展到 26s 后重试应成功
        node._buffer.latest_ns = 26 * 1_000_000_000
        node._timer_cb()
        check("场景C 重试后成功发布",
              len(node._broadcaster.sent) == 1,
              f"共发出 {len(node._broadcaster.sent)} 帧")
        if node._broadcaster.sent:
            st = node._broadcaster.sent[0].header.stamp
            check("场景C 发布的仍是最初那个 scan 时戳（25.000000000）",
                  st.sec == 25 and st.nanosec == 0,
                  f"stamp={st.sec}.{st.nanosec:09d}")
    except Exception as e:  # noqa: BLE001
        check("场景C scan 时戳未被覆盖时不发布（避免贴错值）", False,
              f"{type(e).__name__}: {e}")

    # --- 场景 D：TF 完全查不到 → 不崩溃、也不贴名义值 ---
    try:
        node = mod.GimbalStabilizer()
        node._buffer.mode = "raise"
        scan = _make_scan(30, 0)
        node._scan_cb(scan)
        for _ in range(50):
            node._timer_cb()
        check("场景D TF 全挂时不崩溃", True)
        check("场景D TF 全挂时【不】贴名义值（避免错值污染）",
              len(node._broadcaster.sent) == 0,
              f"实际发了 {len(node._broadcaster.sent)} 帧")
    except Exception as e:  # noqa: BLE001
        check("场景D TF 全挂时不崩溃", False, f"{type(e).__name__}: {e}")

    # --- 场景 E：连发 300 次 scan，无异常累积 ---
    try:
        node = mod.GimbalStabilizer()
        node._buffer.mode = "ok"
        for i in range(300):
            node._scan_cb(_make_scan(40 + i // 10, (i % 10) * 100000000))
        check("场景E 连续 300 帧 scan 全部按各自时戳发布",
              len(node._broadcaster.sent) == 300,
              f"共发出 {len(node._broadcaster.sent)} 帧")
        ok = True
        for i, m in enumerate(node._broadcaster.sent):
            want_sec, want_ns = 40 + i // 10, (i % 10) * 100000000
            if m.header.stamp.sec != want_sec or \
               m.header.stamp.nanosec != want_ns:
                ok = False
                break
        check("场景E 时戳与 scan 严格一一对应", ok)
    except Exception as e:  # noqa: BLE001
        check("场景E 连续 300 帧 scan 全部按各自时戳发布", False,
              f"{type(e).__name__}: {e}")

    # --- 场景 F：崩溃外壳确实生效（注入必炸的 buffer）---
    try:
        node = mod.GimbalStabilizer()

        class Boom:
            def get_latest_common_time(self, *a):
                raise SystemError("注入的硬崩溃")

            def lookup_transform(self, *a, **k):
                raise SystemError("注入的硬崩溃")

        node._buffer = Boom()
        node._timer_cb()   # 不应抛出
        node._scan_cb(_make_scan(50, 0))
        check("场景F 硬崩溃被 _tick 外壳接住", True)
    except Exception as e:  # noqa: BLE001
        check("场景F 硬崩溃被 _tick 外壳接住", False,
              f"异常外泄 {type(e).__name__}: {e}")


def main():
    print("=" * 66)
    print("gimbal_stabilizer 离线单元测试")
    print(f"目标文件：{TARGET}")
    print("=" * 66)
    static_audit()
    dynamic_test()
    print("\n" + "=" * 66)
    if FAIL:
        print(f"❌ 失败 {len(FAIL)} 项：")
        for f in FAIL:
            print(f"   - {f}")
        print("=" * 66)
        return 1
    print("✅ 全部通过。可以放心跑仿真。")
    print("=" * 66)
    return 0


if __name__ == "__main__":
    sys.exit(main())
