#!/usr/bin/env python3
"""把估计地图的"墙格"叠到 GT 上，看糊在哪里。

核心判据：
  EST 墙格数 / GT 墙格数 这个比值是关键。
    接近 1  → 几何基本正确，只差栅格化误差
    2~3     → 轻微膨胀（雷达噪声、角点圆化）
    >5      → 严重糊化，几乎必然有位姿错误（地图被"涂开"）
  本项目实测 7.65，属于第三类。

为什么要按"到 GT 墙的距离"分箱：
  把 EST 的每个占据格，算它到最近的 GT 墙格的距离。
    距离 <= 0.3m 的占多数 → 只是膨胀/噪声，几何对
    距离 > 1.0m 的占多数 → 地图被涂到了不该有墙的地方（位姿错）
  这个直方图比 recall/precision 更能说明"错在哪"。

用法：python3 blur_diag.py <gt.yaml> <est.yaml>
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
        "occ": occ > th, "res": cfg["resolution"],
        "ox": cfg["origin"][0], "oy": cfg["origin"][1],
        "w": w, "h": h,
    }


def est_to_gt_grid(m, gt):
    """把 EST 重采样到 GT 网格上（最近邻），返回同形状 bool。"""
    out = np.zeros((gt["h"], gt["w"]), dtype=bool)
    covered = np.zeros((gt["h"], gt["w"]), dtype=bool)
    # GT 网格的世界坐标范围
    # 遍历 GT 格，反查 EST
    gx = gt["ox"] + (np.arange(gt["w"]) + 0.5) * gt["res"]
    gy = gt["oy"] + (gt["h"] - np.arange(gt["h"]) - 0.5) * gt["res"]
    col = ((gx - m["ox"]) / m["res"]).astype(int)
    row = (m["h"] - 1 - (gy - m["oy"]) / m["res"]).astype(int)
    ok_c = (col >= 0) & (col < m["w"])
    ok_r = (row >= 0) & (row < m["h"])
    for r in range(gt["h"]):
        if not ok_r[r]:
            continue
        rr = row[r]
        cols = col[ok_c]
        out[r, np.where(ok_c)[0]] = m["occ"][rr, cols]
        covered[r, np.where(ok_c)[0]] = True
    return out, covered


def main():
    gt = load(sys.argv[1])
    est = load(sys.argv[2])

    e2g, cov = est_to_gt_grid(est, gt)

    # 到最近 GT 墙的距离（米）
    dist = ndimage.distance_transform_edt(~gt["occ"]) * gt["res"]

    est_cells = e2g & cov
    n_est = int(est_cells.sum())
    n_gt = int(gt["occ"].sum())
    print("GT 墙格 = %d" % n_gt)
    print("EST 墙格(投影到GT网格) = %d" % n_est)
    print("比值 = %.2f" % (n_est / float(n_gt)))
    print("")
    print("=== EST 墙格 到最近 GT 墙 的距离分布 ===")
    d = dist[est_cells]
    if len(d) == 0:
        print("  (EST 无墙格)")
        return
    for lo, hi in [(0, 0.15), (0.15, 0.3), (0.3, 0.5), (0.5, 1.0),
                   (1.0, 2.0), (2.0, 5.0), (5.0, 1e9)]:
        c = int(((d >= lo) & (d < hi)).sum())
        print("  %4.2f-%-5.2f m : %6d 格 (%.1f%%)" % (lo, min(hi, 99.0), c, 100.0 * c / len(d)))
    print("  中位数 = %.3f m   均值 = %.3f m" % (np.median(d), d.mean()))

    # 反方向：GT 墙到最近 EST 墙的距离 —— 看有多少 GT 墙"旁边根本没建"
    dist_gt = ndimage.distance_transform_edt(~est_cells) * gt["res"]
    dg = dist_gt[gt["occ"]]
    print("")
    print("=== GT 墙格 到最近 EST 墙 的距离分布 ===")
    for lo, hi in [(0, 0.15), (0.15, 0.3), (0.3, 0.5), (0.5, 1.0),
                   (1.0, 2.0), (2.0, 5.0), (5.0, 1e9)]:
        c = int(((dg >= lo) & (dg < hi)).sum())
        print("  %4.2f-%-5.2f m : %6d 格 (%.1f%%)" % (lo, min(hi, 99.0), c, 100.0 * c / len(dg)))
    print("  中位数 = %.3f m   均值 = %.3f m" % (np.median(dg), dg.mean()))

    # ---- 空间分布：把 GT 分成 4x4 块，看每块的 EST 墙密度 ----
    print("")
    print("=== 4x4 分块：每块内 EST 墙格数 / GT 墙格数（看哪一区糊得最重）===")
    print("     （行从上到下 = 世界 y 从高到低；列从左到右 = 世界 x 从低到高）")
    H, W = gt["h"], gt["w"]
    bh, bw = H // 4, W // 4
    for bi in range(4):
        line = "  "
        for bj in range(4):
            r0, r1 = bi * bh, (bi + 1) * bh
            c0, c1 = bj * bw, (bj + 1) * bw
            g = int(gt["occ"][r0:r1, c0:c1].sum())
            e = int(est_cells[r0:r1, c0:c1].sum())
            line += "%7s" % ("%d/%d" % (e, g))
        print(line)
    print("")
    print("  读法：数值远大于 1 = 该区域被过度涂满（伪迹堆积）；接近 0 = 该区域没建出来")


if __name__ == "__main__":
    main()
