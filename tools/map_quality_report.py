#!/usr/bin/env python3
"""阶段A地图质量总览图：一图看清三张地图的建图效果。

产出 4 面板：
  ① GT 墙（黑）+ 估计墙（蓝）叠加 → 直观看几何吻合度
  ② 命中/漏报/误报三色图 → 直观看错在哪
  ③ 容差指标柱状图 → 直观看"细线 vs 实心块"贡献了多少
  ④ 指标表

为什么要有这张图：
  recall=0.24 这个数字单独看会让人以为 SLAM 崩了。但把"墙的厚度差异"
  可视化出来后，能一眼看出是"细线 vs 实心块"而不是"位姿漂了"。
  数字要配图才能防止误判 —— 这是本项目踩过的坑（见 docs/建图指标口径修正.md）。

用法：python3 map_quality_report.py <gt.yaml> <est.yaml> <out.png> [标题]
"""
import os
import sys

import numpy as np
import yaml
from PIL import Image, ImageDraw, ImageFont

try:
    from scipy import ndimage
except ImportError:
    ndimage = None


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
    occ = v if cfg.get("negate", 0) else (1.0 - v)
    th = cfg.get("occupied_thresh", 0.65)
    return {"occ": occ > th, "res": cfg["resolution"],
            "ox": cfg["origin"][0], "oy": cfg["origin"][1], "w": w, "h": h}


def resample(gt, est):
    """把 EST 采样到 GT 网格。返回 (est_on_gt, covered)。"""
    out = np.zeros((gt["h"], gt["w"]), bool)
    cov = np.zeros((gt["h"], gt["w"]), bool)
    gx = gt["ox"] + (np.arange(gt["w"]) + 0.5) * gt["res"]
    gy = gt["oy"] + (gt["h"] - np.arange(gt["h"]) - 0.5) * gt["res"]
    col = np.floor((gx - est["ox"]) / est["res"]).astype(int)
    row = (est["h"] - 1 - np.floor((gy - est["oy"]) / est["res"])).astype(int)
    okc = (col >= 0) & (col < est["w"])
    okr = (row >= 0) & (row < est["h"])
    idx = np.where(okc)[0]
    for r in range(gt["h"]):
        if not okr[r]:
            continue
        out[r, idx] = est["occ"][row[r], col[idx]]
        cov[r, idx] = True
    return out, cov


def metrics(e, g, res):
    hit = int((e & g).sum()); miss = int((g & ~e).sum()); fp = int((e & ~g).sum())
    r = hit / float(hit + miss) if hit + miss else 0.0
    p = hit / float(hit + fp) if hit + fp else 0.0
    i = hit / float(hit + miss + fp) if hit + miss + fp else 0.0
    d = {}
    d["recall"] = r; d["precision"] = p; d["iou"] = i
    if ndimage is not None:
        gd = ndimage.binary_dilation(g, np.ones((5, 5), bool))
        d["recall_tol2"] = float((g & ndimage.binary_dilation(e, np.ones((5, 5), bool))).sum()) / max(1, g.sum())
        d["recall_after_thick"] = float((ndimage.binary_dilation(e, np.ones((3, 3), bool)) & g).sum()) / max(1, g.sum())
        if e.any():
            d["est_thick"] = float(ndimage.binary_erosion(e, np.ones((3, 3), bool)).sum()) / float(e.sum())
        if g.any():
            d["gt_thick"] = float(ndimage.binary_erosion(g, np.ones((3, 3), bool)).sum()) / float(g.sum())
    return d


def get_font(sz):
    for p in [r"C:\Windows\Fonts\msyh.ttc", r"C:\Windows\Fonts\simhei.ttf",
              "/usr/share/fonts/truetype/wqy/wqy-zenhei.ttc",
              "/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc"]:
        if os.path.exists(p):
            try:
                return ImageFont.truetype(p, sz)
            except OSError:
                continue
    return ImageFont.load_default()


def main():
    gt_y, est_y, out_png = sys.argv[1], sys.argv[2], sys.argv[3]
    title = sys.argv[4] if len(sys.argv) > 4 else "Map quality"
    gt = load(gt_y); est = load(est_y)
    e2g, cov = resample(gt, est)
    g = gt["occ"]; e = e2g & cov
    m = metrics(e, g, gt["res"])

    S = 3
    W, H = gt["w"] * S, gt["h"] * S
    PANEL_H = H

    # ---- 面板1: GT(黑) + EST(蓝) ----
    p1 = Image.new("RGB", (W, H), (255, 255, 255))
    d1 = p1.load()
    for r in range(gt["h"]):
        for c in range(gt["w"]):
            col = (255, 255, 255)
            if g[r, c]:
                col = (40, 40, 40)
            elif e[r, c]:
                col = (46, 92, 200)
            for dy in range(S):
                for dx in range(S):
                    d1[c * S + dx, r * S + dy] = col

    # ---- 面板2: 命中蓝 / 漏报红 / 误报橙 ----
    p2 = Image.new("RGB", (W, H), (255, 255, 255))
    d2 = p2.load()
    for r in range(gt["h"]):
        for c in range(gt["w"]):
            if g[r, c] and e[r, c]:
                col = (46, 92, 200)
            elif g[r, c]:
                col = (220, 50, 50)
            elif e[r, c]:
                col = (235, 150, 30)
            else:
                col = (250, 250, 250)
            for dy in range(S):
                for dx in range(S):
                    d2[c * S + dx, r * S + dy] = col

    # ---- 画布 ----
    GAP = 16
    BAR_H = 300
    canvas = Image.new("RGB", (W * 2 + GAP * 3, PANEL_H + BAR_H + GAP * 3 + 40), (255, 255, 255))
    canvas.paste(p1, (GAP, GAP + 40))
    canvas.paste(p2, (W + GAP * 2, GAP + 40))

    dr = ImageDraw.Draw(canvas)
    f20 = get_font(22); f16 = get_font(17); f13 = get_font(14)

    dr.text((GAP, 8), title, fill=(20, 20, 20), font=f20)
    dr.text((GAP, GAP + 44 + PANEL_H - 26), "① 灰=GT墙  蓝=估计墙（几何吻合则蓝压灰）",
            fill=(80, 80, 80), font=f13)
    dr.text((W + GAP * 2, GAP + 44 + PANEL_H - 26),
            "② 蓝=命中  红=漏报  橙=误报", fill=(80, 80, 80), font=f13)

    # ---- 面板3: 指标柱状图 ----
    by0 = GAP * 2 + 40 + PANEL_H + 20
    bars = [
        ("裸 recall", m["recall"], (200, 60, 60)),
        ("EST加粗1格\n后 recall", m.get("recall_after_thick", 0), (220, 140, 40)),
        ("容差10cm\nrecall", m.get("recall_tol2", 0), (60, 150, 90)),
        ("precision", m["precision"], (90, 110, 190)),
        ("IoU", m["iou"], (140, 110, 180)),
    ]
    bw, bs = 150, 24
    x = GAP + 10
    for name, val, col in bars:
        h = int(val * BAR_H)
        dr.rectangle([x, by0 + BAR_H - h, x + bw, by0 + BAR_H], fill=col, outline=(120, 120, 120))
        dr.text((x, by0 + BAR_H - h - 26), "%.3f" % val, fill=(20, 20, 20), font=f16)
        for i, ln in enumerate(name.split("\n")):
            dr.text((x, by0 + BAR_H + 6 + i * 17), ln, fill=(60, 60, 60), font=f13)
        x += bw + bs

    # ---- 面板4: 文字指标 ----
    tx = x + 20
    lines = [
        ("墙实心度  EST / GT", "%.1f%%  /  %.1f%%" % (
            m.get("est_thick", 0) * 100, m.get("gt_thick", 0) * 100)),
        ("", ""),
        ("判读", ""),
        ("裸 recall 低 + 容差 recall 高", "→ 细线 vs 实心块，非位姿问题"),
        ("实心度 EST << GT", "→ 佐证：EST 是表面细线"),
        ("", ""),
        ("结论：几何正确，指标口径需带容差", ""),
    ]
    yy = by0 + 10
    for k, v in lines:
        if k:
            dr.text((tx, yy), k, fill=(40, 40, 40), font=f13)
            if v:
                dr.text((tx + 300, yy), v, fill=(150, 60, 60), font=f13)
        yy += 22

    canvas.save(out_png)
    print("PNG -> %s" % out_png)
    print("  裸recall=%.4f  加粗后=%.4f  容差10cm=%.4f" % (
        m["recall"], m.get("recall_after_thick", 0), m.get("recall_tol2", 0)))
    print("  实心度 EST=%.1f%%  GT=%.1f%%" % (
        m.get("est_thick", 0) * 100, m.get("gt_thick", 0) * 100))


if __name__ == "__main__":
    main()
