#!/usr/bin/env python3
"""按连通域分解 GT 的墙，看每块墙的大小分布。
用来判断：召回率低是因为"丢了大块墙"还是"丢了小块柱子"。

用法：python3 gt_components.py <gt.yaml>
"""
import os
import sys

import numpy as np
import yaml
from scipy import ndimage


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
    return w, h, np.frombuffer(d[idx:idx + w * h], dtype=np.uint8).reshape(h, w), mv


def main():
    gt_yaml = sys.argv[1]
    cfg = yaml.safe_load(open(gt_yaml))
    w, h, img, mv = read_pgm(os.path.join(os.path.dirname(gt_yaml), cfg["image"]))
    res = cfg["resolution"]

    v = img.astype(np.float32) / float(mv)
    occ = v if cfg.get("negate", 0) else (1.0 - v)
    wall = occ > cfg.get("occupied_thresh", 0.65)

    lab, n = ndimage.label(wall, structure=np.ones((3, 3), dtype=int))
    print("GT 墙格总数 = %d   连通域数 = %d" % (wall.sum(), n))
    print("")
    print("各连通域（按大小降序，最多 25 个）：")
    sizes = ndimage.sum(wall, lab, range(1, n + 1))
    order = np.argsort(-sizes)
    covered = 0
    for k, i in enumerate(order[:25]):
        cid = i + 1
        sz = int(sizes[i])
        covered += sz
        ys, xs = np.where(lab == cid)
        # 世界坐标包围盒
        wx0 = cfg["origin"][0] + xs.min() * res
        wx1 = cfg["origin"][0] + (xs.max() + 1) * res
        wy0 = cfg["origin"][1] + (h - 1 - ys.max()) * res
        wy1 = cfg["origin"][1] + (h - ys.min()) * res
        print("  #%2d  size=%5d  bbox x[%6.2f,%6.2f] y[%6.2f,%6.2f]  w=%.2f h=%.2f" % (
            k + 1, sz, wx0, wx1, wy0, wy1, wx1 - wx0, wy1 - wy0))

    print("")
    print("前 25 块覆盖 %d / %d = %.1f%% 的墙格" % (covered, wall.sum(), 100.0 * covered / wall.sum()))

    # 大小分布
    print("")
    print("连通域大小分布：")
    for lo, hi in [(1, 4), (5, 20), (21, 100), (101, 500), (501, 10 ** 9)]:
        c = int(((sizes >= lo) & (sizes <= hi)).sum())
        cells = int(sizes[(sizes >= lo) & (sizes <= hi)].sum())
        print("  size %4d-%-8d : %4d 块, 合计 %6d 格 (%.1f%%)" % (
            lo, min(hi, 999999), c, cells, 100.0 * cells / wall.sum()))


if __name__ == "__main__":
    main()
