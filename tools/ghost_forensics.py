#!/usr/bin/env python3
"""ghost_forensics.py —— 幽灵墙成因取证

区分两种截然不同的成因（对应完全不同的修法）：

  A. 「整块子图被单次误匹配拖走」
     → 幽灵区是【一块连续、形状规整、边界锐利】的区域，
       里面还保留着墙的轮廓（只是位置整体平移/旋转了）。
       修法：收紧 scan matching（scan_buffer / link_scan_maximum_distance），
             或提高匹配响应阈值。

  B. 「全程累积漂移 / 尺度错误」
     → 幽灵区【随距离渐远逐渐发散】，近处贴得紧、远处越跑越歪，
       且难以看出完整墙形。
       修法：查 odom/TF 是否多源、时间戳是否错位、雷达外参是否对。

本脚本用三个量区分：
  1. 幽灵连通块数量与大小 —— A 通常是"1~2 个大块"，B 是"碎成一片"
  2. 幽灵块的内部纹理 —— 统计幽灵区里占据格的"线状指数"（是否有墙形）
  3. 溢出距离 vs 与场地的关系 —— A 的边界锐利，B 是渐变

用法：
    python3 tools/ghost_forensics.py --est <map.yaml> --gt <gt.yaml>
"""
import argparse
import math
import os

import numpy as np

# ---- 自研 PGM(P5) 读取器（不依赖 PIL）----


def read_pgm(path):
    with open(path, "rb") as f:
        data = f.read()
    # 解析头：P5 <w> <h> <maxval>\n
    tokens = []
    i = 0
    while len(tokens) < 4:
        # 跳过空白与注释
        while i < len(data) and data[i:i + 1].isspace():
            i += 1
        if data[i:i + 1] == b"#":
            while i < len(data) and data[i:i + 1] != b"\n":
                i += 1
            continue
        j = i
        while j < len(data) and not data[j:j + 1].isspace():
            j += 1
        tokens.append(data[i:j])
        i = j
    if tokens[0] != b"P5":
        raise ValueError(f"只支持 P5 二进制 PGM，得到 {tokens[0]}")
    w, h, maxv = int(tokens[1]), int(tokens[2]), int(tokens[3])
    i += 1  # 跳过分隔的单个空白
    dt = np.uint8 if maxv < 256 else np.uint16
    img = np.frombuffer(data, dtype=dt, count=w * h, offset=i).reshape(h, w)
    return img, w, h


def load_map(yaml_path):
    import yaml
    with open(yaml_path) as f:
        meta = yaml.safe_load(f)
    d = os.path.dirname(os.path.abspath(yaml_path))
    img, w, h = read_pgm(os.path.join(d, meta["image"]))
    res = float(meta["resolution"])
    ox, oy = float(meta["origin"][0]), float(meta["origin"][1])
    occ_th = float(meta.get("occupied_thresh", 0.65))
    free_th = float(meta.get("free_thresh", 0.196))
    negate = int(meta.get("negate", 0))

    # 归一化占据概率 p
    p = img.astype(np.float64) / (255.0 if img.dtype == np.uint8 else 65535.0)
    if negate:
        p = 1.0 - p
    else:
        p = 1.0 - p
    occupied = p > occ_th
    free = p < free_th
    # PGM 行 0 在顶部，对应世界 y 最大；转成"行 0 = y 最小"便于和世界坐标一致
    occupied = occupied[::-1]
    free = free[::-1]
    return dict(occupied=occupied, free=free, res=res,
                origin=(ox, oy), w=w, h=h)


def world_of(idx, res, origin):
    """栅格索引 -> 世界坐标（左下角）"""
    return (origin[0] + idx[0] * res, origin[1] + idx[1] * res)


def label_components(mask):
    """4 邻域连通块标记（自己实现，不依赖 scipy）。返回 (labels, n)。"""
    h, w = mask.shape
    labels = np.zeros((h, w), dtype=np.int32)
    cur = 0

    # 迭代式 DFS，避免递归爆栈
    for si in range(h):
        for sj in range(w):
            if not mask[si, sj] or labels[si, sj]:
                continue
            cur += 1
            stack = [(si, sj)]
            labels[si, sj] = cur
            while stack:
                i, j = stack.pop()
                for di, dj in ((1, 0), (-1, 0), (0, 1), (0, -1)):
                    ni, nj = i + di, j + dj
                    if 0 <= ni < h and 0 <= nj < w and mask[ni, nj] \
                            and not labels[ni, nj]:
                        labels[ni, nj] = cur
                        stack.append((ni, nj))
            if cur > 400 and si == 0:   # 防御：异常情况早停
                pass
    return labels, cur


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--est", required=True)
    ap.add_argument("--gt", required=True)
    ap.add_argument("--thumb", type=int, default=60)
    args = ap.parse_args()

    est = load_map(args.est)
    gt = load_map(args.gt)

    e_occ, g_occ = est["occupied"], gt["occupied"]
    eres, eorg = est["res"], est["origin"]
    gres, gorg = gt["res"], gt["origin"]

    print("=" * 78)
    print(f"EST  {est['w']}x{est['h']} @ {eres}m  origin {eorg}")
    print(f"GT   {gt['w']}x{gt['h']} @ {gres}m  origin {gorg}")

    # GT 已知范围（世界）
    gt_x0 = gorg[0]
    gt_y0 = gorg[1]
    gt_x1 = gorg[0] + gt["w"] * gres
    gt_y1 = gorg[1] + gt["h"] * gres
    print(f"GT 世界范围 x[{gt_x0:.2f},{gt_x1:.2f}] y[{gt_y0:.2f},{gt_y1:.2f}]")

    # 把 EST 的每个占据格转成世界坐标，判断是否在 GT 范围内
    ii, jj = np.nonzero(e_occ)
    wx = eorg[0] + ii * eres
    wy = eorg[1] + jj * eres
    inside = (wx >= gt_x0) & (wx <= gt_x1) & (wy >= gt_y0) & (wy <= gt_y1)
    n_tot = len(wx)
    n_in = int(inside.sum())
    n_out = n_tot - n_in
    print("-" * 78)
    print(f"EST 占据格 {n_tot}   场内 {n_in} ({100.0*n_in/max(1,n_tot):.1f}%)   "
          f"场外 {n_out} ({100.0*n_out/max(1,n_tot):.1f}%)")

    # ---- 幽灵连通块分析（在 EST 栅格上做，只保留场外格）----
    ghost = e_occ.copy()
    gx = eorg[0] + np.arange(est["w"])[None, :] * eres
    gy = (eorg[1] + np.arange(est["h"])[:, None] * eres)
    inb = ((gx >= gt_x0) & (gx <= gt_x1)) & ((gy >= gt_y0) & (gy <= gt_y1))
    ghost = ghost & (~inb)

    if ghost.sum() > 0:
        labels, n = label_components(ghost)
        sizes = np.bincount(labels.ravel())
        sizes[0] = 0
        order = np.argsort(-sizes)
        print(f"\n幽灵连通块数 {n}   最大块 {sizes[order[0]]} 格 "
              f"({100.0*sizes[order[0]]/max(1,ghost.sum()):.1f}% of ghost)")
        print("Top5 块：")
        for k in order[:5]:
            if sizes[k] == 0:
                continue
            m = labels == k
            iis, jjs = np.nonzero(m)
            bx = eorg[0] + iis * eres
            by = eorg[1] + jjs * eres
            # 细长度：用包围盒长宽比与填充率粗略刻画
            bw = bx.max() - bx.min() + eres
            bh = by.max() - by.min() + eres
            fill = sizes[k] * eres * eres / max(1e-9, bw * bh)
            print(f"   块{k}: {sizes[k]:6d} 格  包围盒 {bw:5.1f}x{bh:5.1f}m  "
                  f"填充率 {fill:.3f}  质心 ({bx.mean():6.2f},{by.mean():6.2f})")

        # ---- 判断：是"1~2 个规整大块"(A 型) 还是"碎成一片"(B 型) ----
        big = sizes[order[0]] / max(1, ghost.sum())
        top2 = (sizes[order[0]] + (sizes[order[1]] if n > 1 else 0)) \
            / max(1, ghost.sum())
        print("\n" + "=" * 78)
        print("成因判读：")
        if top2 > 0.5:
            print(f"  ⇒ 前 2 大块占幽灵区 {100*top2:.0f}% —— 倾向【A 型："
                  f"整块子图被单次误匹配拖走】")
            print("     修法：收紧 scan matching；查 scan_buffer_maximum_scan_distance")
        else:
            print(f"  ⇒ 最大块仅占 {100*big:.0f}%、块数 {n} 偏多 —— 倾向【B 型："
                  f"累积漂移/多源位姿】")
            print("     修法：查 TF 多源、时间戳错位、雷达外参")
    else:
        print("\n✅ 无幽灵格")

    # ---- 溢出距离分桶 ----
    if n_out > 0:
        ox, oy = wx[~inside], wy[~inside]
        dx = np.maximum(np.maximum(gt_x0 - ox, ox - gt_x1), 0)
        dy = np.maximum(np.maximum(gt_y0 - oy, oy - gt_y1), 0)
        d = np.hypot(dx, dy)
        print("\n溢出距离分桶：")
        for lo, hi, name in ((0, 0.5, "<0.5m"), (0.5, 1, "0.5-1m"),
                             (1, 2, "1-2m"), (2, 5, "2-5m"),
                             (5, 10, "5-10m"), (10, 1e9, ">10m")):
            m = (d >= lo) & (d < hi)
            if m.sum():
                print(f"   {name:8s} {int(m.sum()):6d}  "
                      f"({100.0*m.sum()/n_out:5.1f}%) "
                      f"{'#' * int(30*m.sum()/n_out)}")


if __name__ == "__main__":
    main()
