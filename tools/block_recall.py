#!/usr/bin/env python3
"""逐块（GT 连通域）统计召回情况。

目的：回答"召回率 0.24 到底丢在哪"。
  GT 的墙被分成若干连通域（外框 / 4 根横墙 / 4 个柱子）。
  对每一块，统计它有多少比例的格子在估计地图里也被判为占据。

如果外框召回高、柱子召回低 → 问题在小目标（栅格化 + 阈值）
如果所有块都低 → 问题在位姿（整张图系统性错位 / 拖影）
如果只有某几块低 → 问题在该区域的可观测性（被遮挡、走得太远）

用法：python3 block_recall.py <gt.yaml> <est.yaml> [--dump] [--shift dx dy]
      --shift 用来测试"如果整体平移 (dx,dy) 米，召回会不会变好"
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


def load(yaml_path):
    cfg = yaml.safe_load(open(yaml_path))
    p = os.path.join(os.path.dirname(os.path.abspath(yaml_path)), cfg["image"])
    w, h, img, mv = read_pgm(p)
    v = img.astype(np.float32) / float(mv)
    neg = bool(cfg.get("negate", 0))
    occ = v if neg else (1.0 - v)
    th = cfg.get("occupied_thresh", 0.65)
    return {
        "occ": occ > th,
        "res": cfg["resolution"],
        "ox": cfg["origin"][0],
        "oy": cfg["origin"][1],
        "w": w, "h": h,
    }


def world_to_idx(m, wx, wy):
    """世界坐标 -> (row, col)，越界返回 None。"""
    col = int((wx - m["ox"]) / m["res"])
    row = m["h"] - 1 - int((wy - m["oy"]) / m["res"])
    if 0 <= col < m["w"] and 0 <= row < m["h"]:
        return row, col
    return None


def main():
    args = [a for a in sys.argv[1:] if not a.startswith("--")]
    shift = (0.0, 0.0)
    if "--shift" in sys.argv:
        i = sys.argv.index("--shift")
        shift = (float(sys.argv[i + 1]), float(sys.argv[i + 2]))
    gt = load(args[0])
    est = load(args[1])

    lab, n = ndimage.label(gt["occ"], structure=np.ones((3, 3), dtype=int))
    sizes = ndimage.sum(gt["occ"], lab, range(1, n + 1))
    order = np.argsort(-sizes)

    print("=== 逐块召回（GT 连通域 → 在 EST 中的命中率）===")
    if shift != (0.0, 0.0):
        print("（已施加整体平移 dx=%.2f dy=%.2f m）" % shift)
    print("")
    print(" %-4s %-7s %-9s %-8s %-9s %s" % ("#", "size", "recall", "hit", "miss", "bbox(世界坐标)"))
    tot_hit = 0
    tot_miss = 0
    rows = []
    for k, i in enumerate(order):
        cid = i + 1
        sz = int(sizes[i])
        ys, xs = np.where(lab == cid)
        hit = miss = 0
        for r, c in zip(ys, xs):
            # 该 GT 格中心的世界坐标
            wx = gt["ox"] + (c + 0.5) * gt["res"] + shift[0]
            wy = gt["oy"] + (gt["h"] - r - 0.5) * gt["res"] + shift[1]
            ri = world_to_idx(est, wx, wy)
            if ri is not None and est["occ"][ri]:
                hit += 1
            else:
                miss += 1
        tot_hit += hit
        tot_miss += miss
        wx0 = gt["ox"] + xs.min() * gt["res"]
        wx1 = gt["ox"] + (xs.max() + 1) * gt["res"]
        wy0 = gt["oy"] + (gt["h"] - 1 - ys.max()) * gt["res"]
        wy1 = gt["oy"] + (gt["h"] - ys.min()) * gt["res"]
        rows.append((k + 1, sz, hit / float(sz) if sz else 0.0, hit, miss,
                     "x[%6.2f,%6.2f] y[%6.2f,%6.2f]" % (wx0, wx1, wy0, wy1)))

    for r in rows:
        print(" %-4d %-7d %-9.4f %-8d %-9d %s" % r)

    print("")
    print("总召回 = %d / %d = %.4f" % (tot_hit, tot_hit + tot_miss,
                                     tot_hit / float(tot_hit + tot_miss)))

    # 按大小分档看：
    print("")
    print("=== 按墙块大小分档的召回 ===")
    big_h = sum(r[3] for r in rows if r[1] >= 500)
    big_m = sum(r[4] for r in rows if r[1] >= 500)
    mid_h = sum(r[3] for r in rows if 100 <= r[1] < 500)
    mid_m = sum(r[4] for r in rows if 100 <= r[1] < 500)
    sml_h = sum(r[3] for r in rows if r[1] < 100)
    sml_m = sum(r[4] for r in rows if r[1] < 100)
    for name, h_, m_ in [("大块 >=500", big_h, big_m),
                         ("中块 100-499", mid_h, mid_m),
                         ("小块 <100", sml_h, sml_m)]:
        t = h_ + m_
        print("  %-14s : %5d / %5d = %.4f" % (name, h_, t, h_ / float(t) if t else 0.0))

    # ---- 最简单也最有力的诊断：EST 里到底有多少墙格？----
    print("")
    print("=== EST 墙格体量 ===")
    print("  EST 总占据格 = %d" % int(est["occ"].sum()))
    print("  GT  总占据格 = %d" % int(gt["occ"].sum()))
    print("  比值 = %.2f" % (est["occ"].sum() / float(gt["occ"].sum())))


if __name__ == "__main__":
    main()
