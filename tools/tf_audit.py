#!/usr/bin/env python3
"""tf_audit.py —— 记录并审计 map->odom 与 odom->base 的时间序列

用途（定位"场外幽灵墙"）：
  机器人用 /odom 看始终在场地内，但地图却长到 y=-59m。这只可能是
  map->odom 发生了累计漂移（slam_toolbox 把 map 原点挪走了）。
  本节点以固定频率采样三样东西并写 CSV：
      t, map_odom_x, map_odom_y, map_odom_yaw, base_x, base_y, base_yaw
  跑完用 --analyze 模式读 CSV，报出：
      · map->odom 的累计位移范围
      · 相邻采样的最大跳变（>0.5m 是重定位/回环的标志）
      · base 在 map 下的位姿范围（判断 map 坐标系是否被拖走）

用法：
    # 采样（另开终端，仿真已在跑）
    python3 tools/tf_audit.py --duration 300 --out /tmp/tf.csv
    # 分析
    python3 tools/tf_audit.py --analyze /tmp/tf.csv
"""
import argparse
import csv
import math
import sys
import time

import rclpy
from rclpy.node import Node
from tf2_ros import Buffer, TransformListener


def quat_to_yaw(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


class TFaudit(Node):
    def __init__(self, out, duration, child="base_footprint"):
        super().__init__("tf_audit")
        self._buf = Buffer()
        self._lis = TransformListener(self._buf, self)
        self._out = out
        self._child = child
        self._t0 = time.time()
        self._duration = duration
        self._rows = []
        self._err = {}
        self._timer = self.create_timer(0.1, self._tick)

    def _lookup(self, src, dst):
        """查 TF，返回 (x, y, yaw) 或 None。

        ⚠️ 这里必须用 Time(seconds=0)（= "最新可用"），不能用
        rclpy.time.Time() 的默认构造再靠位置参数混淆 —— 本工具第一版就是
        栽在这儿：整个 240s 采样全是空值，看起来像"map->odom 根本不存在"，
        实际是查询方式不对，白白误导了一轮排查。

        另外：TransformListener 需要时间填充 buffer，启动后头几十毫秒查不到
        是正常的，所以这里把异常按类型计数，便于区分"一直查不到"和"刚开始查不到"。
        """
        try:
            t = self._buf.lookup_transform(
                src, dst, rclpy.time.Time(seconds=0))
            return (t.transform.translation.x,
                    t.transform.translation.y,
                    quat_to_yaw(t.transform.rotation))
        except Exception as e:
            self._err[type(e).__name__] = self._err.get(type(e).__name__, 0) + 1
            return None

    def _tick(self):
        el = time.time() - self._t0
        if el > self._duration:
            self._save()
            rclpy.shutdown()
            return
        rec = {"t": round(el, 3)}
        mo = self._lookup("map", "odom")
        if mo:
            rec["mo_x"], rec["mo_y"], rec["mo_yaw"] = \
                (round(mo[0], 4), round(mo[1], 4), round(mo[2], 4))
        else:
            rec.update(mo_x=None, mo_y=None, mo_yaw=None)
        b = self._lookup("map", self._child)
        if b:
            rec["b_x"], rec["b_y"], rec["b_yaw"] = \
                (round(b[0], 4), round(b[1], 4), round(b[2], 4))
        else:
            rec.update(b_x=None, b_y=None, b_yaw=None)
        self._rows.append(rec)

    def _save(self):
        if not self._rows:
            print("没有采到数据")
            return
        keys = ["t", "mo_x", "mo_y", "mo_yaw", "b_x", "b_y", "b_yaw"]
        with open(self._out, "w", newline="\n", encoding="utf-8") as f:
            w = csv.DictWriter(f, fieldnames=keys)
            w.writeheader()
            for r in self._rows:
                w.writerow(r)
        n_mo = sum(1 for r in self._rows if r.get("mo_x") is not None)
        n_b = sum(1 for r in self._rows if r.get("b_x") is not None)
        print(f"已写 {len(self._rows)} 行 -> {self._out}")
        print(f"  map->odom 成功 {n_mo}/{len(self._rows)}")
        print(f"  map->base 成功 {n_b}/{len(self._rows)}")
        if self._err:
            print("  查询异常统计:")
            for k, v in sorted(self._err.items(), key=lambda x: -x[1]):
                print(f"    {k}: {v}")


def analyze(path):
    with open(path, encoding="utf-8") as f:
        rows = list(csv.DictReader(f))

    def col(k):
        out = []
        for r in rows:
            v = r.get(k, "")
            out.append(float(v) if v not in ("", "None") else None)
        return out

    def rng(vals):
        v = [x for x in vals if x is not None]
        return (min(v), max(v)) if v else (None, None)

    print("=" * 70)
    print(f"采样 {len(rows)} 点，时长 {rows[-1]['t']}s")

    for name, keys in (("map->odom", ("mo_x", "mo_y", "mo_yaw")),
                       ("map->base", ("b_x", "b_y", "b_yaw"))):
        print(f"\n{name}:")
        for k in keys:
            lo, hi = rng(col(k))
            if lo is None:
                print(f"   {k}: 无数据")
            else:
                print(f"   {k}: [{lo:8.3f}, {hi:8.3f}]   跨度 {hi-lo:7.3f}")

    # map->odom 的跳变
    mx, my = col("mo_x"), col("mo_y")
    jumps = []
    for i in range(1, len(mx)):
        if mx[i] is None or mx[i - 1] is None:
            continue
        d = math.hypot(mx[i] - mx[i - 1], my[i] - my[i - 1])
        jumps.append((d, float(rows[i]["t"])))
    if jumps:
        jumps.sort(reverse=True)
        print("\nmap->odom 相邻采样位移 Top5（0.1s 间隔，正常应 ~0）:")
        for d, t in jumps[:5]:
            flag = "  ❗跳变" if d > 0.05 else ""
            print(f"   {d:.4f} m  t={t:.1f}s{flag}")
        tot = sum(d for d, _ in jumps)
        print(f"   累计 |Δ(map->odom)| = {tot:.2f} m")
        if tot > 1.0:
            print("   ⇒ map->odom 在持续漂移 —— 这就是地图长到场外的原因")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--out", default="/tmp/tf_audit.csv")
    ap.add_argument("--duration", type=float, default=300.0)
    ap.add_argument("--analyze", default=None)
    a = ap.parse_args()

    if a.analyze:
        analyze(a.analyze)
        return 0

    rclpy.init()
    n = TFaudit(a.out, a.duration)
    try:
        rclpy.spin(n)
    except KeyboardInterrupt:
        n._save()
    finally:
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
