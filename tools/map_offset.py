#!/usr/bin/env python3
"""检查估计地图相对 GT 是否存在系统性偏移。

做法：在 ±N 格的搜索窗内平移估计地图，找 IoU 最大的偏移量。
若最优偏移显著非零，说明地图"内容对但位置偏了"，
这会让逐格 IoU 被严重低估（墙只有 1 格宽，偏半格就全错开）。

用法：python3 map_offset.py <gt.yaml> <est.yaml> [search_cells=20]
"""
import os
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
    return w, h, np.frombuffer(d[idx:idx + w * h], dtype=np.uint8).reshape(h, w), mv


def load_occ(yaml_path, thresh=0.65):
    cfg = yaml.safe_load(open(yaml_path))
    img_path = os.path.join(os.path.dirname(os.path.abspath(yaml_path)), cfg["image"])
    w, h, img, mv = read_pgm(img_path)
    negate = bool(cfg.get("negate", 0))
    v = img.astype(np.float32) / float(mv)
    occ = v if negate else (1.0 - v)
    return (occ > thresh), cfg["resolution"], cfg["origin"][0], cfg["origin"][1]


def main():
    gt_yaml, est_yaml = sys.argv[1], sys.argv[2]
    search = int(sys.argv[3]) if len(sys.argv) > 3 else 20

    gt_occ, gt_res, gt_ox, gt_oy = load_occ(gt_yaml)
    est_occ, est_res, est_ox, est_oy = load_occ(est_yaml)

    gh, gw = gt_occ.shape
    eh, ew = est_occ.shape
    print("GT  %dx%d res=%.3f origin=(%.2f,%.2f)  occ=%d" % (
        gw, gh, gt_res, gt_ox, gt_oy, gt_occ.sum()))
    print("EST %dx%d res=%.3f origin=(%.2f,%.2f)  occ=%d" % (
        ew, eh, est_res, est_ox, est_oy, est_occ.sum()))

    # 基准对齐：把 GT 的每个格中心映射到 EST 网格索引
    # GT (row, col) 中心的世界坐标
    rows, cols = np.mgrid[0:gh, 0:gw]
    wx = gt_ox + (cols + 0.5) * gt_res
    wy = gt_oy + (gh - rows - 0.5) * gt_res
    ec = ((wx - est_ox) / est_res).astype(np.int32)
    er = (eh - 1 - ((wy - est_oy) / est_res)).astype(np.int32)

    gt_mask = gt_occ

    def iou_at(dr, dc):
        r2 = er + dr
        c2 = ec + dc
        valid = (r2 >= 0) & (r2 < eh) & (c2 >= 0) & (c2 < ew)
        est_hit = np.zeros_like(gt_mask)
        est_hit[valid] = est_occ[r2[valid], c2[valid]]
        inter = (gt_mask & est_hit).sum()
        union = (gt_mask | est_hit).sum()
        return inter / float(union) if union else 0.0, inter

    best = (None, -1, -1)
    for dr in range(-search, search + 1, 2):
        for dc in range(-search, search + 1, 2):
            v, _ = iou_at(dr, dc)
            if v > best[1]:
                best = ((dr, dc), v, _)

    base_iou, base_inter = iou_at(0, 0)
    print("")
    print("零偏移 IoU      = %.4f  (命中 %d)" % (base_iou, base_inter))
    print("最优偏移 (%d,%d) IoU = %.4f" % (best[0][0], best[0][1], best[1]))
    print("最优偏移对应世界坐标位移 = (%.3f, %.3f) m" % (
        -best[0][1] * est_res, -best[0][0] * est_res))
    print("")
    if best[1] > base_iou * 1.5:
        print("结论：存在显著系统性偏移 —— 地图内容对但位置偏了")
        print("      逐格 IoU 被严重低估，应改用「按障碍距离」或「膨胀后 IoU」评估")
    else:
        print("结论：无明显系统性偏移，漏报是真实的几何缺失")

    # 用最优偏移重新算 recall/precision
    dr, dc = best[0]
    r2 = er + dr
    c2 = ec + dc
    valid = (r2 >= 0) & (r2 < eh) & (c2 >= 0) & (c2 < ew)
    est_hit = np.zeros_like(gt_mask)
    est_hit[valid] = est_occ[r2[valid], c2[valid]]
    hit = (gt_mask & est_hit).sum()
    miss = (gt_mask & ~est_hit).sum()
    fp = (~gt_mask & est_hit).sum()
    print("")
    print("按最优偏移重算：")
    print("  recall    = %.4f" % (hit / float(hit + miss)))
    print("  precision = %.4f" % (hit / float(hit + fp)))
    print("  IoU       = %.4f" % (hit / float(hit + miss + fp)))


if __name__ == "__main__":
    main()
