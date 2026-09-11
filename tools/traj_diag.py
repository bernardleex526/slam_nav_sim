#!/usr/bin/env python3
"""traj_diag.py —— 探索轨迹体检

回答三个问题：
  1. 机器人有没有跑出场地？（用 /odom，不是 GT）
  2. 有没有异常的帧间跳变？（位姿发散会在轨迹上留痕）
  3. 轨迹是否覆盖了全部路点区域？（有没有整条扫描线没走）

用法：
    python3 tools/traj_diag.py <traj.csv> [world_x_max] [world_y_max]
"""
import csv
import math
import sys


def main():
    if len(sys.argv) < 2:
        print(__doc__)
        return 2
    path = sys.argv[1]
    wx = float(sys.argv[2]) if len(sys.argv) > 2 else 10.0
    wy = float(sys.argv[3]) if len(sys.argv) > 3 else 10.0

    ts, xs, ys = [], [], []
    with open(path, encoding="utf-8") as f:
        for row in csv.DictReader(f):
            try:
                ts.append(float(row["t"]))
                xs.append(float(row["x"]))
                ys.append(float(row["y"]))
            except (KeyError, ValueError):
                continue

    if not xs:
        print("轨迹为空")
        return 1

    print(f"点数 {len(xs)}   时间跨度 {ts[0]:.0f} ~ {ts[-1]:.0f} s "
          f"（{ts[-1] - ts[0]:.0f}s）")
    print(f"范围  x[{min(xs):.2f}, {max(xs):.2f}]  y[{min(ys):.2f}, {max(ys):.2f}]")

    dist = sum(math.hypot(xs[i] - xs[i - 1], ys[i] - ys[i - 1])
               for i in range(1, len(xs)))
    print(f"总路程 {dist:.1f} m   平均速度 {dist / max(1e-6, ts[-1] - ts[0]):.3f} m/s")

    # ---- 越界检查 ----
    out = [(t, x, y) for t, x, y in zip(ts, xs, ys)
           if abs(x) > wx or abs(y) > wy]
    if out:
        print(f"❗越界点数 {len(out)}/{len(xs)}")
        print(f"   首次越界 t={out[0][0]:.0f}s ({out[0][1]:.2f}, {out[0][2]:.2f})")
        print(f"   最远超界 x={max(abs(v[1]) for v in out):.2f} "
              f"y={max(abs(v[2]) for v in out):.2f}")
    else:
        print("✅ 全程未越界")

    # ---- 帧间跳变 ----
    jumps = []
    for i in range(1, len(xs)):
        d = math.hypot(xs[i] - xs[i - 1], ys[i] - ys[i - 1])
        jumps.append((d, i))
    jumps.sort(reverse=True)
    print("\n最大帧间位移 Top5（正常应 <0.2m；>1m 说明位姿发散）:")
    for d, i in jumps[:5]:
        flag = "  ❗发散" if d > 1.0 else ""
        print(f"   {d:.3f} m  第{i}点  t={ts[i]:.0f}s  "
              f"({xs[i]:.2f}, {ys[i]:.2f}){flag}")

    # ---- 覆盖率（按 1m 网格）----
    cell = 1.0
    seen = set()
    for x, y in zip(xs, ys):
        seen.add((int(math.floor(x / cell)), int(math.floor(y / cell))))
    nx = int(2 * wx / cell)
    ny = int(2 * wy / cell)
    print(f"\n1m 网格覆盖 {len(seen)}/{nx * ny} = "
          f"{100.0 * len(seen) / (nx * ny):.1f}%")
    return 0


if __name__ == "__main__":
    sys.exit(main())
