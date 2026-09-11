#!/usr/bin/env python3
"""
map_ghost_diag.py —— 诊断估计地图中场外占据格（幽灵墙）的空间分布

为什么需要这个脚本：
  幽灵墙是"建图质量"最隐蔽的失效模式 —— 地图看着有墙、覆盖率也能到 1.0，
  但一大半占据格其实落在场地【外面】。原因是 scan matching 某次误匹配后
  整块子图被拖走，或者局部扫描链横跨太长、一帧错帧带动一大片。
  只看 recall/precision 是发现不了的（分母里混了幽灵格）。

输出：
  1. 世界坐标下的 EST 占据格统计（总数 / 场内 / 场外 / 场外占比）
  2. 场外格的空间范围（告诉你被拖到多远）
  3. ASCII 缩略图（一眼看出是"整块被拖走"还是"散点噪声"）
  4. 场外格按距离分桶（近场 vs 远场，区分"边界溢出"与"整块漂移"）

用法：
  python3 tools/map_ghost_diag.py \
      --gt src/sim_worlds/maps/simple.yaml \
      --est /home/xxx/.ros/sim_stage_a/.../map.yaml
"""
import argparse
import os

import numpy as np
import yaml


def load_pgm(path):
    """极简 PGM(P5) 读取器 —— 避免依赖 PIL/opencv。"""
    with open(path, "rb") as f:
        data = f.read()
    # 解析 header: P5 <w> <h> <maxval>\n
    tokens = []
    i = 0
    while len(tokens) < 4:
        # 跳过空白与注释
        while data[i:i + 1].isspace():
            i += 1
        if data[i:i + 1] == b"#":
            while data[i:i + 1] not in (b"\n", b""):
                i += 1
            continue
        j = i
        while not data[j:j + 1].isspace():
            j += 1
        tokens.append(data[i:j])
        i = j
    magic, w, h, maxval = tokens[0], int(tokens[1]), int(tokens[2]), int(tokens[3])
    i += 1  # 单个空白分隔符
    if magic == b"P5":
        img = np.frombuffer(data, dtype=np.uint8, count=w * h, offset=i).reshape(h, w)
    else:  # P2 ASCII
        img = np.array(data[i:].split(), dtype=np.uint8)[:w * h].reshape(h, w)
    return img


def load_map(yaml_path):
    d = yaml.safe_load(open(yaml_path, encoding="utf-8"))
    img_path = d["image"]
    if not os.path.isabs(img_path):
        img_path = os.path.join(os.path.dirname(os.path.abspath(yaml_path)), img_path)
    img = load_pgm(img_path)
    res = float(d["resolution"])
    ox, oy = float(d["origin"][0]), float(d["origin"][1])
    negate = int(d.get("negate", 0))
    return img, res, ox, oy, negate


def occupied_mask(img, negate):
    """返回占据格布尔掩码。PGM: 0=黑=占据(通常)，255=白=空闲，205=未知。"""
    if negate:
        return img > 200
    return img < 100


def world_extent(img, res, ox, oy):
    h, w = img.shape
    return ox, ox + w * res, oy, oy + h * res


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--gt", required=True)
    ap.add_argument("--est", required=True)
    ap.add_argument("--thumb", type=int, default=80,
                    help="ASCII 缩略图宽度（列数）")
    args = ap.parse_args()

    gimg, gres, gox, goy, gneg = load_map(args.gt)
    eimg, eres, eox, eoy, eneg = load_map(args.est)

    gx0, gx1, gy0, gy1 = world_extent(gimg, gres, gox, goy)
    print("=" * 78)
    print("GT  范围: x[%.2f, %.2f]  y[%.2f, %.2f]   分辨率 %.3f  网格 %dx%d"
          % (gx0, gx1, gy0, gy1, gres, gimg.shape[1], gimg.shape[0]))
    print("EST 范围: x[%.2f, %.2f]  y[%.2f, %.2f]   分辨率 %.3f  网格 %dx%d"
          % (eox, eox + eimg.shape[1] * eres, eoy, eoy + eimg.shape[0] * eres,
             eres, eimg.shape[1], eimg.shape[0]))

    eocc = occupied_mask(eimg, eneg)
    eh, ew = eocc.shape
    eys, exs = np.where(eocc)
    if not len(eys):
        print("EST 地图里没有占据格")
        return

    # 栅格中心的世界坐标
    ewx = eox + (exs + 0.5) * eres
    ewy = eoy + (eh - eys - 0.5) * eres

    inb = (ewx >= gx0) & (ewx <= gx1) & (ewy >= gy0) & (ewy <= gy1)
    n_tot = len(eys)
    n_in = int(inb.sum())
    n_out = n_tot - n_in

    print("-" * 78)
    print("EST 占据格总数 : %d" % n_tot)
    print("  场内         : %d  (%.1f%%)" % (n_in, 100.0 * n_in / n_tot))
    print("  场外(幽灵)   : %d  (%.1f%%)" % (n_out, 100.0 * n_out / n_tot))

    if n_out == 0:
        print("  ✅ 没有场外幽灵格")
        return

    ox_, oy_ = ewx[~inb], ewy[~inb]
    print("-" * 78)
    print("场外格空间范围: x[%.2f, %.2f]  y[%.2f, %.2f]"
          % (ox_.min(), ox_.max(), oy_.min(), oy_.max()))
    print("       质心   : (%.2f, %.2f)" % (ox_.mean(), oy_.mean()))

    # 场外格距 GT 边界的溢出距离（区分"贴边溢出"和"整块拖走"）
    dx = np.maximum(np.maximum(gx0 - ox_, ox_ - gx1), 0.0)
    dy = np.maximum(np.maximum(gy0 - oy_, oy_ - gy1), 0.0)
    d = np.hypot(dx, dy)
    print("-" * 78)
    print("场外格溢出距离分桶:")
    edges = [0.0, 0.5, 1.0, 2.0, 5.0, 10.0, 1e9]
    names = ["<0.5m", "0.5-1m", "1-2m", "2-5m", "5-10m", ">10m"]
    for k in range(len(names)):
        c = int(((d >= edges[k]) & (d < edges[k + 1])).sum())
        bar = "#" * int(40.0 * c / max(1, n_out))
        print("   %-8s %6d (%5.1f%%) %s"
              % (names[k], c, 100.0 * c / n_out, bar))

    # 场内 vs 场外 的栅格密度对比（看是"薄薄一层"还是"实心块"）
    print("-" * 78)
    print("提示：")
    if n_out / n_tot > 0.3:
        if ((d > 2.0).sum() / n_out) > 0.5:
            print("  ⚠️  大部分幽灵格溢出 >2m ⇒ 典型的【整块子图被位姿发散拖走】。")
            print("     排查方向: scan matching 参数 / 局部链跨度 / TF 是否有多源冲突")
        else:
            print("  ⚠️  幽灵格集中贴边(<2m) ⇒ 更像【栅格边界处理或雷达远端噪声】。")
    else:
        print("  ✅ 场外占比 <30%%，属可接受的边界溢出。")

    # ---- ASCII 缩略图 ----
    print("-" * 78)
    print("ASCII 缩略图（. = 空  o = 场内占据  X = 场外幽灵  | = GT 边界）")
    W = args.thumb
    # 用 EST 的全范围作为画布
    ax0 = min(eox, gx0)
    ax1 = max(eox + eimg.shape[1] * eres, gx1)
    ay0 = min(eoy, gy0)
    ay1 = max(eoy + eimg.shape[0] * eres, gy1)
    sys_w = ax1 - ax0
    sys_h = ay1 - ay0
    H = max(8, int(W * sys_h / max(1e-6, sys_w) * 0.5))

    canvas = [[" "] * W for _ in range(H)]
    for i in range(H):
        for j in range(W):
            canvas[i][j] = "."
    # 投放占据格
    for wx, wy, isin in zip(ewx[::max(1, n_tot // 60000)],
                            ewy[::max(1, n_tot // 60000)],
                            inb[::max(1, n_tot // 60000)]):
        j = int((wx - ax0) / sys_w * (W - 1))
        i = int((ay1 - wy) / sys_h * (H - 1))
        if 0 <= i < H and 0 <= j < W:
            canvas[i][j] = "o" if isin else "X"
    # 画 GT 边界框
    for (bx0, by0, bx1, by1) in [(gx0, gy0, gx1, gy1)]:
        for j in range(W):
            wx = ax0 + j / (W - 1) * sys_w
            if abs(wx - bx0) < sys_w / W or abs(wx - bx1) < sys_w / W:
                for i in range(H):
                    if canvas[i][j] == ".":
                        canvas[i][j] = "|"
        for i in range(H):
            wy = ay1 - i / (H - 1) * sys_h
            if abs(wy - by0) < sys_h / H or abs(wy - by1) < sys_h / H:
                for j in range(W):
                    if canvas[i][j] == ".":
                        canvas[i][j] = "-"
    print("   +" + "-" * W + "+")
    for row in canvas:
        print("   |" + "".join(row) + "|")
    print("   +" + "-" * W + "+")
    print("   画布范围 x[%.1f, %.1f]  y[%.1f, %.1f]" % (ax0, ax1, ay0, ay1))


if __name__ == "__main__":
    main()
