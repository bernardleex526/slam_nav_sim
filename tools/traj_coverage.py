#!/usr/bin/env python3
"""分析探索轨迹对每个墙格的观测几何：
对 GT 中每个墙格，找轨迹上距它最近的点，统计该距离的分布。

如果大量墙格的最小距离落在盲区（<0.6m）之外但仍然没被建出来，
说明问题不在盲区；如果大量墙格的最近通过距离 > 雷达有效距离或角度受限，
才是几何问题。

用法：python3 traj_coverage.py <traj.csv> <gt.yaml> [blind=0.6]
"""
import math
import sys

import numpy as np
import yaml


def read_pgm(path):
    with open(path, "rb") as f:
        d = f.read()
    idx = 2
    vals = []
    while len(vals) < 3:
        while d[idx:idx + 1].isspace():
            idx += 1
        if d[idx:idx + 1] == b"#":
            while d[idx:idx + 1] != b"\n":
                idx += 1
            continue
        s = idx
        while not d[idx:idx + 1].isspace():
            idx += 1
        vals.append(int(d[s:idx]))
    idx += 1
    w, h, mv = vals
    return w, h, np.frombuffer(d[idx:idx + w * h], dtype=np.uint8).reshape(h, w)


def main():
    traj_csv, gt_yaml = sys.argv[1], sys.argv[2]
    blind = float(sys.argv[3]) if len(sys.argv) > 3 else 0.6

    gt = yaml.safe_load(open(gt_yaml))
    import os
    _w, _h, img = read_pgm(os.path.join(os.path.dirname(gt_yaml), gt["image"]))
    res = gt["resolution"]
    ox, oy = gt["origin"][0], gt["origin"][1]
    h, w = img.shape
    # ROS 约定：黑(0)=墙
    walls = np.argwhere(img < 128)
    print("GT 墙格数 = %d" % len(walls))

    # 轨迹
    pts = []
    with open(traj_csv) as f:
        next(f)
        for line in f:
            p = line.strip().split(",")
            if len(p) >= 3:
                pts.append((float(p[1]), float(p[2])))
    tr = np.array(pts)
    print("轨迹点数 = %d" % len(tr))

    # 墙格 -> 世界坐标
    wx = ox + (walls[:, 1] + 0.5) * res
    wy = oy + (h - 1 - walls[:, 0] + 0.5) * res

    # 每个墙格到轨迹的最小距离（分块算，避免内存爆）
    dmin = np.full(len(wx), 1e9)
    CH = 4000
    for i in range(0, len(wx), CH):
        ax = wx[i:i + CH][:, None]
        ay = wy[i:i + CH][:, None]
        d = np.sqrt((ax - tr[None, :, 0]) ** 2 + (ay - tr[None, :, 1]) ** 2)
        dmin[i:i + CH] = d.min(axis=1)

    print("")
    print("=== 墙格到轨迹的最近通过距离分布 ===")
    buckets = [(0, 0.6), (0.6, 1.0), (1.0, 1.5), (1.5, 2.0), (2.0, 3.0),
               (3.0, 5.0), (5.0, 10.0), (10.0, 1e9)]
    for lo, hi in buckets:
        c = int(((dmin >= lo) & (dmin < hi)).sum())
        bar = "#" * int(60.0 * c / len(dmin))
        print("  %5.1f-%5.1f m: %6d  (%.1f%%)  %s" % (
            lo, min(hi, 99.9), c, 100.0 * c / len(dmin), bar))

    inside = int((dmin < blind).sum())
    print("")
    print("落在盲区(<%.2fm)内的墙格 = %d (%.1f%%)" % (blind, inside, 100.0 * inside / len(dmin)))
    print("盲区外可观测的墙格     = %d (%.1f%%)" % (
        len(dmin) - inside, 100.0 * (len(dmin) - inside) / len(dmin)))


if __name__ == "__main__":
    main()
