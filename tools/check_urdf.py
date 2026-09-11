#!/usr/bin/env python3
"""校验展开后的 URDF：link/joint 完整性、插件、传感器挂载、悬空关节。

用法: python3 check_urdf.py <urdf 路径>
"""
import sys
import xml.etree.ElementTree as ET

path = sys.argv[1]
root = ET.parse(path).getroot()
links = [l.get("name") for l in root.findall("link")]
joints = root.findall("joint")

print("robot name :", root.get("name"))
print("links      :", len(links), " joints:", len(joints))
print()

print("=== 关键 link ===")
for k in ("base_footprint", "base", "base_link", "lidar_link", "imu_link",
          "camera_link", "camera_depth_frame", "FL_foot", "RR_foot"):
    print("  %-22s %s" % (k, "OK" if k in links else "**缺失**"))
print()

print("=== Gazebo 插件 ===")
for g in root.findall("gazebo"):
    for p in g.findall("plugin"):
        print("  %-20s -> %s" % (p.get("name"), p.get("filename")))
print()

print("=== 传感器挂载 ===")
for j in joints:
    c, par = j.find("child"), j.find("parent")
    if c is None or par is None:
        continue
    if c.get("link") in ("lidar_link", "imu_link", "camera_link"):
        o = j.find("origin")
        print("  %-14s <- %-14s xyz=%s"
              % (c.get("link"), par.get("link"),
                 o.get("xyz") if o is not None else None))
print()

missing = [j.get("name") for j in joints
           if j.find("parent") is None or j.find("parent").get("link") not in links]
print("=== 悬空关节 ===", missing if missing else "无")
print()

print("=== 关节类型统计 ===")
from collections import Counter
print(" ", dict(Counter(j.get("type") for j in joints)))
