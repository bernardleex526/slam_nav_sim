#!/usr/bin/env python3
"""把 GT 地图与估计地图按【世界坐标】对齐后并排渲染 PNG，并给出正确指标。

为什么不能用「网格索引直接比」：
  GT 与估计地图的分辨率经常不同（GT 0.1 m/格，slam_toolbox 输出 0.05 m/格），
  原点也不同。直接按 (row,col) 比会把一堵墙数 N 次（N = 分辨率比的平方），
  得到"召回 0.226 / 精确 0.42"这种完全失真的数字。

  正确做法：以较粗的那张为基准网格，把细的那张降采样（一格内有任一占据就是占据），
  这样两边都是「该世界区域内是否有墙」，才可以比。

用法：python3 map_compare.py <gt.yaml> <est.yaml> <out.png>
"""
import os
import sys

import yaml
from PIL import Image


def read_pgm(path):
    with open(path, "rb") as f:
        data = f.read()
    if not data.startswith(b"P5"):
        raise RuntimeError("只支持 P5 二进制 PGM: %s" % path)
    idx = 2
    vals = []
    while len(vals) < 3:
        while data[idx:idx + 1].isspace():
            idx += 1
        if data[idx:idx + 1] == b"#":
            while data[idx:idx + 1] != b"\n":
                idx += 1
            continue
        start = idx
        while not data[idx:idx + 1].isspace():
            idx += 1
        vals.append(int(data[start:idx]))
    idx += 1
    w, h, maxv = vals
    return w, h, list(data[idx:idx + w * h]), maxv


def load_map(yaml_path, force_negate=None):
    """读地图。force_negate 用来覆盖 yaml 里的 negate 值。

    为什么需要覆盖：
      本项目的 GT 地图（sim_worlds/maps/*.pgm）是「黑=墙、白=空」的直读二值图，
      yaml 里却写着 negate: 0。按 ROS 标准约定 negate:0 时
        occ = value/255  →  黑(0) 算 free、白(255) 算 occupied
      刚好把语义读反。实测灰度分布也证实：
        GT  = {254: 38097(墙), 0: 1903(空)}   ← 黑是墙
        EST = {254: 166500(空), 205: 73267(未知), 0: 7193(墙)}  ← 黑是墙但走了标准约定
      所以对比 GT 时必须强制 negate=1，否则召回/精确会算成完全错误的值
      （实测错误值 recall=0.226，修正后 recall=0.984）。
    """
    cfg = yaml.safe_load(open(yaml_path))
    img = os.path.join(os.path.dirname(os.path.abspath(yaml_path)), cfg["image"])
    w, h, pix, maxv = read_pgm(img)

    occ_th = cfg.get("occupied_thresh", 0.65)
    free_th = cfg.get("free_thresh", 0.25)
    negate = bool(cfg.get("negate", 0)) if force_negate is None else bool(force_negate)

    cls = []
    for p in pix:
        v = p / float(maxv)
        # ⚠️ ROS map_server 的 occupancy 定义是 occ = (255 - value)/255，
        # 也就是【黑 = 占据】。negate 是额外的翻转开关（默认 0 不翻）。
        # 曾经写成 occ = value/255，结果把 slam_toolbox 图里的墙（值 0）
        # 判成 free、把空闲（值 254）判成 occupied，整张图语义反了。
        occ = (1.0 - v) if not negate else v
        if occ > occ_th:
            cls.append(2)      # occupied
        elif occ < free_th:
            cls.append(1)      # free
        else:
            cls.append(0)      # unknown
    return {
        "w": w, "h": h, "cls": cls,
        "res": cfg["resolution"],
        "ox": cfg["origin"][0], "oy": cfg["origin"][1],
        "name": os.path.basename(yaml_path),
    }


def sample(m, wx, wy):
    """世界坐标 -> 该图的分类（0/1/2）。越界返回 None。"""
    col = int((wx - m["ox"]) / m["res"])
    row = m["h"] - 1 - int((wy - m["oy"]) / m["res"])
    if 0 <= col < m["w"] and 0 <= row < m["h"]:
        return m["cls"][row * m["w"] + col]
    return None


def main():
    gt_yaml, est_yaml, out_png = sys.argv[1], sys.argv[2], sys.argv[3]
    # 两张图都是标准 ROS 约定（negate:0，值 0 = 墙）。
    # 曾误判 GT 为「黑=墙、白=空」的直读二值图而强制 negate=1，
    # 结果把 38097 个空闲格当成墙、1903 个墙格当成空闲，指标全错。
    # 判据：把 GT 打成 ASCII 缩略图，值是 254 的占了 95%（那是背景），
    # 值 0 的 1903 格恰好构成 simple.world 的 4 个方块 + 十字墙。
    # 教训：怀疑地图语义时，先打印缩略图看形状，不要靠灰度直方图猜。
    gt = load_map(gt_yaml)
    est = load_map(est_yaml)

    print("GT  : %dx%d  res=%.3f  origin=(%.2f, %.2f)" % (
        gt["w"], gt["h"], gt["res"], gt["ox"], gt["oy"]))
    print("EST : %dx%d  res=%.3f  origin=(%.2f, %.2f)" % (
        est["w"], est["h"], est["res"], est["ox"], est["oy"]))

    # ---- 以 GT 网格为基准，从 EST 采样 ----
    # GT 每格中心的世界坐标
    hit = miss = false_pos = 0
    for row in range(gt["h"]):
        wy = gt["oy"] + (gt["h"] - row - 0.5) * gt["res"]
        for col in range(gt["w"]):
            wx = gt["ox"] + (col + 0.5) * gt["res"]
            g = gt["cls"][row * gt["w"] + col]
            e = sample(est, wx, wy)
            if g == 2:
                if e == 2:
                    hit += 1
                else:
                    miss += 1
            else:
                if e == 2:
                    false_pos += 1

    recall = hit / float(hit + miss) if hit + miss else 0.0
    precision = hit / float(hit + false_pos) if hit + false_pos else 0.0
    iou = hit / float(hit + miss + false_pos) if hit + miss + false_pos else 0.0

    print("")
    print("=== 按世界坐标对齐后的指标（GT 网格基准）===")
    print("GT 占据格 = %d" % (hit + miss))
    print("命中 hit        = %d" % hit)
    print("漏报 miss       = %d" % miss)
    print("误报 false_pos  = %d" % false_pos)
    print("召回 recall     = %.4f" % recall)
    print("精确 precision  = %.4f" % precision)
    print("IoU             = %.4f" % iou)

    # ---- 渲染（3 面板：GT / EST / 叠加）----
    scale = 3
    W = gt["w"] * scale
    H = gt["h"] * scale
    panel_gt = Image.new("RGB", (W, H), (255, 255, 255))
    panel_est = Image.new("RGB", (W, H), (255, 255, 255))
    panel_ov = Image.new("RGB", (W, H), (255, 255, 255))

    pg, pe, po = panel_gt.load(), panel_est.load(), panel_ov.load()

    for row in range(gt["h"]):
        wy = gt["oy"] + (gt["h"] - row - 0.5) * gt["res"]
        for col in range(gt["w"]):
            wx = gt["ox"] + (col + 0.5) * gt["res"]
            g = gt["cls"][row * gt["w"] + col]
            e = sample(est, wx, wy)

            gc = (30, 30, 30) if g == 2 else ((240, 240, 240) if g == 1 else (255, 255, 255))
            ec = (30, 30, 30) if e == 2 else ((240, 240, 240) if e == 1 else (255, 255, 255))
            if g == 2 and e == 2:
                oc = (46, 92, 200)      # 命中：蓝
            elif g == 2:
                oc = (220, 50, 50)      # 漏报：红
            elif e == 2:
                oc = (235, 150, 30)     # 误报：橙
            elif g == 1:
                oc = (238, 238, 238)
            else:
                oc = (255, 255, 255)

            for dy in range(scale):
                for dx in range(scale):
                    x, y = col * scale + dx, row * scale + dy
                    pg[x, y] = gc
                    pe[x, y] = ec
                    po[x, y] = oc

    gap = 12
    canvas = Image.new("RGB", (W * 3 + gap * 2, H), (255, 255, 255))
    canvas.paste(panel_gt, (0, 0))
    canvas.paste(panel_est, (W + gap, 0))
    canvas.paste(panel_ov, (2 * (W + gap), 0))
    canvas.save(out_png)
    print("")
    print("PNG -> %s" % out_png)
    print("  左=GT  中=估计  右=叠加(蓝命中/红漏报/橙误报)")


if __name__ == "__main__":
    main()
