#!/usr/bin/env python3
"""单测 make_lawnmower_path：确认环形遍历覆盖全部 lane。

复现旧 bug：start 在正中时旧实现只返回后半段 lane。
"""
import math
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'src/sim_benchmark/scripts'))


def ref_impl(bbox, lane_spacing, start):
    """与 exploration_runner.make_lawnmower_path 同步的参考实现（不含 ROS 依赖）。"""
    x0, y0, x1, y1 = bbox
    n_lanes = max(1, int(math.floor((y1 - y0) / lane_spacing)) + 1)
    ys = [y0 + (i + 0.5) * (y1 - y0) / n_lanes for i in range(n_lanes)]

    path = []
    for i, y in enumerate(ys):
        if i % 2 == 0:
            path.append((x0, y, 0.0)); path.append((x1, y, math.pi))
        else:
            path.append((x1, y, math.pi)); path.append((x0, y, 0.0))

    sx, sy = start
    best = min(range(len(path)), key=lambda k: (path[k][0] - sx) ** 2 + (path[k][1] - sy) ** 2)
    lane_idx = best // 2

    order = [((lane_idx + k) % n_lanes) for k in range(n_lanes)]
    trimmed = []
    for i in order:
        if i % 2 == 0:
            trimmed.append((x0, ys[i], 0.0)); trimmed.append((x1, ys[i], math.pi))
        else:
            trimmed.append((x1, ys[i], math.pi)); trimmed.append((x0, ys[i], 0.0))
    if best % 2 == 1:
        trimmed[0], trimmed[1] = trimmed[1], trimmed[0]
    return trimmed, ys, lane_idx, n_lanes


def main():
    # 复现 simple 环境的参数
    bbox = (-9.05, -9.05, 9.15, 9.15)
    spacing = 1.2
    wp, ys, lane_idx, n = ref_impl(bbox, spacing, (0.0, 0.0))

    print("n_lanes = %d   lane_idx(起点所在) = %d" % (n, lane_idx))
    print("路点数 = %d" % len(wp))

    covered = sorted({round(p[1], 3) for p in wp})
    covered_c = sorted({round(y, 3) for y in ys})
    print("")
    print("覆盖到的 lane y 值：")
    for y in covered:
        mark = " <-- 起点 lane" if abs(y - round(ys[lane_idx], 3)) < 1e-6 else ""
        print("  %7.3f%s" % (y, mark))

    print("")
    print("覆盖 lane 数: %d / 期望 %d" % (len(covered), n))
    print("覆盖 y 范围: %.3f .. %.3f" % (min(covered), max(covered)))
    print("期望 y 范围: %.3f .. %.3f" % (min(covered_c), max(covered_c)))

    ok = covered == covered_c
    print("")
    print("结果: %s" % ("PASS —— 全部 lane 都被覆盖" if ok else "FAIL —— 有 lane 被丢弃"))
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
