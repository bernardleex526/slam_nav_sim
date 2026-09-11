#!/usr/bin/env python3
"""探测当前 ROS 图里 slam_toolbox 的 SaveMap 服务到底叫什么名字。

为什么写这个工具：
  stage_a_runner 一直用 `map_saver_cli` 存图，它靠**订阅 /map 话题**拿地图。
  而 slam_toolbox 的 /map 是 transient_local（latched）——订阅时机一旦错开
  就拿不到数据，map_saver 会一直卡到超时，且报错只有一行
  "map_saver lifecycle node launched"，完全指不到真正原因。

  slam_toolbox 自带的 SaveMap 服务【直接序列化它内部的地图对象】，
  不存在这个时序问题，是更可靠的存图路径。但要用它，先得知道服务名 ——
  而 `ros2 service list` 在实验负载高时经常返回空（discovery 超时，
  实测 ros2 topic hz 正常但 service list 是空的），不能作为依据。

  所以这里用 `get_service_names_and_types()` 按类型反查，一次问清楚。

用法：
    python3 probe_slam_services.py          # 查 slam 相关服务
    python3 probe_slam_services.py --all    # 列出全部服务（调试发现用）
"""

import sys
import time

import rclpy
from rclpy.node import Node


def main():
    show_all = "--all" in sys.argv
    rclpy.init()
    node = rclpy.create_node("slam_service_probe")

    # discovery 需要几秒才稳，等一下再问。单次查询经常拿到不全的结果，
    # 所以连问 3 次取并集 —— 这不是重试，是 discovery 本身渐进式收敛。
    svcs = {}
    for i in range(3):
        try:
            names_types = node.get_service_names_and_types()
        except Exception as e:
            print(f"  [第{i+1}次查询失败] {type(e).__name__}: {e}")
            names_types = []
        for n, types in names_types:
            svcs.setdefault(n, set(types))
        if i < 2:
            time.sleep(2.0)
            rclpy.spin_once(node, timeout_sec=0.1)

    if not svcs:
        print("❌ 一个服务都没发现。可能：(1) 仿真没在跑 "
              "(2) ROS_DOMAIN_ID 不一致 (3) discovery 尚未收敛。")
        node.destroy_node()
        rclpy.shutdown()
        return 1

    print(f"共发现 {len(svcs)} 个服务\n")

    print("=== slam_toolbox 相关服务 ===")
    hit = False
    for n in sorted(svcs):
        if "slam" in n.lower():
            hit = True
            print(f"  {n}")
            for t in sorted(svcs[n]):
                print(f"      {t}")
    if not hit:
        print("  （没有名字里带 slam 的服务）")

    print("\n=== SaveMap 服务（按类型反查）===")
    hit = False
    for n in sorted(svcs):
        if any("SaveMap" in t for t in svcs[n]):
            hit = True
            print(f"  ✅ {n}   {sorted(svcs[n])}")
    if not hit:
        print("  ❌ 没有找到类型为 SaveMap 的服务")

    if show_all:
        print("\n=== 全部服务 ===")
        for n in sorted(svcs):
            print(f"  {n}  {sorted(svcs[n])}")

    node.destroy_node()
    rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
