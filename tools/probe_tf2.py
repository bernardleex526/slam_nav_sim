#!/usr/bin/env python3
"""探测 tf2_ros.Buffer 的可用 API（用于确认时戳对齐方案是否可行）。"""
import tf2_ros

b = tf2_ros.Buffer()
names = [m for m in dir(b) if "time" in m.lower() or "stamp" in m.lower()]
print("含 time/stamp 的方法：")
for n in names:
    print("  ", n)
print()
print("get_latest_common_time 存在:", hasattr(b, "get_latest_common_time"))
print("lookup_transform 存在:", hasattr(b, "lookup_transform"))

# 检查 TransformListener 是否要求显式 node 参数（Humble 起必须传）
import inspect
try:
    sig = inspect.signature(tf2_ros.TransformListener.__init__)
    print("\nTransformListener.__init__ 签名:", sig)
except Exception as e:
    print("签名读取失败:", e)
