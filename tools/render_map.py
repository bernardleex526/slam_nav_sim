#!/usr/bin/env python3
"""把单张地图渲染成 PNG（占据=黑、空闲=白、未知=灰），并可放大查看局部。

用法：python3 render_map.py <map.yaml> <out.png> [crop_x0 crop_y0 crop_x1 crop_y1] [scale]
"""
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
    return np.frombuffer(d[idx:idx + w * h], dtype=np.uint8).reshape(h, w), mv


def main():
    map_yaml = sys.argv[1]
    out_png = sys.argv[2]
    scale = int(sys.argv[3]) if len(sys.argv) > 3 else 2

    cfg = yaml.safe_load(open(map_yaml))
    img_path = os.path.join(os.path.dirname(os.path.abspath(map_yaml)), cfg["image"])
    arr, mv = read_pgm(img_path)
    h, w = arr.shape

    occ_th = cfg.get("occupied_thresh", 0.65)
    free_th = cfg.get("free_thresh", 0.25)
    negate = bool(cfg.get("negate", 0))

    v = arr.astype(np.float32) / float(mv)
    occ = v if negate else (1.0 - v)

    rgb = np.zeros((h, w, 3), dtype=np.uint8)
    rgb[...] = (128, 128, 128)                       # unknown = 灰
    rgb[occ < free_th] = (255, 255, 255)             # free = 白
    rgb[occ > occ_th] = (0, 0, 0)                    # occupied = 黑

    im = Image.fromarray(rgb).resize((w * scale, h * scale), Image.NEAREST)
    im.save(out_png)

    n_occ = int((occ > occ_th).sum())
    n_free = int((occ < free_th).sum())
    tot = w * h
    print("%s: %dx%d  occ=%d (%.1f%%)  free=%d (%.1f%%)  unknown=%d (%.1f%%)" % (
        os.path.basename(map_yaml), w, h,
        n_occ, 100.0 * n_occ / tot, n_free, 100.0 * n_free / tot,
        tot - n_occ - n_free, 100.0 * (tot - n_occ - n_free) / tot))
    print("-> %s (scale x%d)" % (out_png, scale))


if __name__ == "__main__":
    main()
