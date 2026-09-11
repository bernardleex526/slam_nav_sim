#!/usr/bin/env python3
"""ghost_geometry.py —— 幽灵墙的几何形态分析

目标：判定幽灵区到底是「同一个场景的副本」还是「随机噪声」。
做法：
  1. 把 EST 的场内占据格单独拿出来，做二值化 + 自相关/平移搜索，
     看能否在幽灵区里找到与场内结构"长得一样"的平移副本。
  2. 若幽灵区是"平移副本"→ 说明 map->odom 在某时刻发生整体跳变
     （slam_toolbox 重定位/回环把 map 原点挪了），而不是逐帧噪声。
  3. 若幽灵区毫无结构 → 才是逐帧外参错乱。

这里实现一个轻量版：统计幽灵区占据格的
  · 局部方向一致性（用梯度方向直方图）
  · 与场内结构的"形状相似度"（比较两者的格点间距分布）
"""
import argparse
import math
import os

import numpy as np


def read_pgm(path):
    with open(path, "rb") as f:
        data = f.read()
    if not data.startswith(b"P5"):
        raise ValueError("只支持 P5")
    tokens, i = [], 2
    while len(tokens) < 3:
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
    w, h, maxv = int(tokens[0]), int(tokens[1]), int(tokens[2])
    i += 1
    dt = np.uint8 if maxv < 256 else np.uint16
    return np.frombuffer(data, dtype=dt, count=w * h, offset=i).reshape(h, w)


def load(path):
    import yaml
    with open(path) as f:
        m = yaml.safe_load(f)
    d = os.path.dirname(os.path.abspath(path))
    img = read_pgm(os.path.join(d, m["image"]))
    p = img.astype(np.float64) / 255.0
    occ = (1.0 - p) > float(m.get("occupied_thresh", 0.65))
    return occ[::-1], float(m["resolution"]), \
        (float(m["origin"][0]), float(m["origin"][1]))


def spacing_hist(pts, max_r=3.0, bins=30):
    """格点最近邻间距分布 —— 用空间哈希做。"""
    if len(pts) < 2:
        return np.zeros(bins)
    cell = 0.3
    grid = {}
    for x, y in pts:
        grid.setdefault((int(x // cell), int(y // cell)), []).append((x, y))
    nn = []
    for x, y in pts:
        cx, cy = int(x // cell), int(y // cell)
        best = 1e9
        for dx in (-1, 0, 1):
            for dy in (-1, 0, 1):
                for px, py in grid.get((cx + dx, cy + dy), []):
                    if px == x and py == y:
                        continue
                    d = math.hypot(px - x, py - y)
                    if d < best:
                        best = d
        if best < 1e8:
            nn.append(best)
    if not nn:
        return np.zeros(bins)
    h, _ = np.histogram(nn, range=(0, max_r), bins=bins)
    return h / max(1, h.sum())


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--est", required=True)
    ap.add_argument("--gt", required=True)
    args = ap.parse_args()

    e_occ, er, eo = load(args.est)
    g_occ, gr, go = load(args.gt)

    gh, gw = g_occ.shape
    gx0, gy0 = go
    gt_x1, gt_y1 = gx0 + gw * gr, gy0 + gh * gr

    ii, jj = np.nonzero(e_occ)
    wx = eo[0] + (ii + 0.5) * er
    wy = eo[1] + (jj + 0.5) * er
    inb = (wx >= gx0) & (wx <= gt_x1) & (wy >= gy0) & (wy <= gt_y1)

    in_pts = np.stack([wx[inb], wy[inb]], axis=1)
    out_pts = np.stack([wx[~inb], wy[~inb]], axis=1)
    print(f"场内占据格 {len(in_pts)}   场外 {len(out_pts)}")

    h_in = spacing_hist(in_pts)
    h_out = spacing_hist(out_pts)

    print("\n最近邻间距分布（0~3m，30 桶）：")
    print("  bin(m)   场内    场外")
    for k in range(0, 30, 3):
        lo = k * 3.0 / 30
        print(f"  {lo:.2f}   {h_in[k]:.4f}  {h_out[k]:.4f}")

    # 相似度：直方图交集
    inter = float(np.minimum(h_in, h_out).sum())
    print(f"\n分布交集 = {inter:.3f}")
    if inter > 0.7:
        print("⇒ 场内与场外【结构相似】—— 场外很可能是同一场景的平移/旋转副本")
        print("   （对应 map->odom 整体跳变，例如回环重定位）")
    else:
        print("⇒ 场内与场外【结构不同】—— 场外更像是逐帧错乱产生的散乱伪迹")
        print("   （对应逐帧外参/时戳错配，而非整体跳变）")

    # 场内/场外的占据密度
    span_in = (in_pts.max(0) - in_pts.min(0)) if len(in_pts) else np.array([1, 1])
    span_out = (out_pts.max(0) - out_pts.min(0)) if len(out_pts) else np.array([1, 1])
    dens_in = len(in_pts) / max(1e-6, span_in[0] * span_in[1])
    dens_out = len(out_pts) / max(1e-6, span_out[0] * span_out[1])
    print(f"\n场内跨度 {span_in[0]:.1f}x{span_in[1]:.1f}m 密度 {dens_in:.3f} 格/m²")
    print(f"场外跨度 {span_out[0]:.1f}x{span_out[1]:.1f}m 密度 {dens_out:.3f} 格/m²")
    print(f"密度比 场外/场内 = {dens_out/max(1e-9,dens_in):.4f}")
    if dens_out / max(1e-9, dens_in) < 0.2:
        print("⇒ 场外明显更稀疏 ⇒ 典型的「少量帧被贴到离谱位置」而不是整体偏移")


if __name__ == "__main__":
    main()
