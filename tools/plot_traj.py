#!/usr/bin/env python3
"""把探索轨迹画成 PNG，叠加 GT 墙，直观看路径是否规整。

若轨迹是规整的平行直线 → 路径正常，地图崩坏来自 SLAM 侧；
若轨迹本身乱 → 底盘/里程计侧的问题。

用法：python3 plot_traj.py <traj.csv> <gt.yaml> <out.png> [scale]
"""
import csv
import os
import sys

import numpy as np
import yaml
from PIL import Image


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
    traj_csv, gt_yaml, out_png = sys.argv[1], sys.argv[2], sys.argv[3]
    scale = int(sys.argv[4]) if len(sys.argv) > 4 else 4

    cfg = yaml.safe_load(open(gt_yaml))
    gw, gh, img, mv = read_pgm(os.path.join(os.path.dirname(gt_yaml), cfg["image"]))
    res = cfg["resolution"]
    ox, oy = cfg["origin"][0], cfg["origin"][1]

    v = img.astype(np.float32) / float(mv)
    occ = v if cfg.get("negate", 0) else (1.0 - v)
    base = np.full((gh, gw, 3), 255, dtype=np.uint8)
    base[occ > 0.65] = (170, 170, 170)   # 墙：灰

    im = Image.fromarray(base).resize((gw * scale, gh * scale), Image.NEAREST)
    px = im.load()

    pts = []
    with open(traj_csv) as f:
        for row in csv.DictReader(f):
            pts.append((float(row["x"]), float(row["y"])))

    def w2p(wx, wy):
        col = (wx - ox) / res
        row = gh - 1 - (wy - oy) / res
        return int(col * scale), int(row * scale)

    # 轨迹画成折线（粗一点）
    for i in range(1, len(pts)):
        x0, y0 = w2p(*pts[i - 1])
        x1, y1 = w2p(*pts[i])
        n = max(abs(x1 - x0), abs(y1 - y0), 1)
        for k in range(n + 1):
            x = int(x0 + (x1 - x0) * k / n)
            y = int(y0 + (y1 - y0) * k / n)
            for dy in (-1, 0, 1):
                for dx in (-1, 0, 1):
                    if 0 <= x + dx < im.width and 0 <= y + dy < im.height:
                        px[x + dx, y + dy] = (200, 40, 40)

    # 起点
    sx, sy = w2p(*pts[0])
    for dy in range(-4, 5):
        for dx in range(-4, 5):
            if dx * dx + dy * dy <= 16 and 0 <= sx + dx < im.width and 0 <= sy + dy < im.height:
                px[sx + dx, sy + dy] = (30, 120, 30)

    im.save(out_png)
    print("轨迹点数 %d -> %s" % (len(pts), out_png))
    print("x: %.2f..%.2f  y: %.2f..%.2f" % (
        min(p[0] for p in pts), max(p[0] for p in pts),
        min(p[1] for p in pts), max(p[1] for p in pts)))


if __name__ == "__main__":
    main()
