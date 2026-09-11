#!/usr/bin/env python3
"""
evaluate_vs_gt
==============
离线评测：把「跑出来的地图 + 轨迹」和「ground-truth」对比，出一份硬指标报告。

为什么必须有这一步：
  在 rviz 里看地图"像不像"，只能发现数量级级别的错误。0.3m 的层间错位、
  5% 的墙体重影、覆盖率差 10% —— 肉眼全看不出来，但这些恰恰是算法调优时
  最需要量化的东西。

指标口径（与 robot_interface_spec/config/interface.yaml 的 metrics 段一致）：

  建图类
    coverage_ratio    对 GT 可通行区域的覆盖率   —— 漏建了多少
    precision         估计为占据的格里，确实是障碍的比例 —— 建了多少假墙
    recall            真实障碍里被建出来的比例
    iou               占据区域交并比，综合指标
    grid_accuracy     逐格一致率（含空闲区域），最能反映"地图整体对不对"

  轨迹类
    ate_rmse / ate_max        绝对轨迹误差（含 Umeyama 对齐与未对齐两版）
    rpe_trans_rmse            相对位姿误差（平移），反映局部漂移而非全局偏移
    path_length_*             路径长度

用法：
  python3 evaluate_vs_gt.py \\
      --gt-map  <sim_worlds>/maps/complex.yaml \\
      --est-map ~/.ros/sim_eval/run1/final_map.yaml \\
      --gt-traj  ~/.ros/sim_eval/run1/trajectory_gt.csv \\
      --est-traj ~/.ros/sim_eval/run1/trajectory_est.csv \\
      --out      ~/.ros/sim_eval/run1/report.json
"""

import argparse
import csv
import json
import os
import sys

import numpy as np

try:
    import yaml
except ImportError:
    yaml = None

try:
    from scipy import ndimage as _ndimage
except ImportError:
    _ndimage = None


# ---------------------------------------------------------------------------
# PGM / 占据栅格读写
# ---------------------------------------------------------------------------


def read_pgm(path):
    """读 P5 二进制 PGM。返回 (img[H,W] uint8, maxval)。"""
    with open(path, "rb") as f:
        raw = f.read()

    if not raw.startswith(b"P5"):
        raise ValueError(f"{path} 不是 P5 二进制 PGM")

    # 头部分三段：magic / 宽高 / maxval，字段间可能夹注释
    tokens, i = [], 2
    while len(tokens) < 3:
        while i < len(raw) and raw[i:i + 1].isspace():
            i += 1
        if raw[i:i + 1] == b"#":
            while i < len(raw) and raw[i:i + 1] != b"\n":
                i += 1
            continue
        j = i
        while j < len(raw) and not raw[j:j + 1].isspace():
            j += 1
        tokens.append(raw[i:j])
        i = j
    i += 1  # 跳过分隔空白

    w, h, maxval = int(tokens[0]), int(tokens[1]), int(tokens[2])
    img = np.frombuffer(raw, dtype=np.uint8, count=w * h, offset=i)
    return img.reshape(h, w), maxval


def load_occupancy(map_yaml_path):
    """读 map_server 格式的地图。返回 (grid, meta)。

    grid 取值约定（和 map_server 一致）：
      -1 = unknown, 0 = free, 100 = occupied
    """
    with open(map_yaml_path, "r") as f:
        meta = yaml.safe_load(f)

    pgm_path = os.path.join(os.path.dirname(os.path.abspath(map_yaml_path)),
                            meta["image"])
    img, _ = read_pgm(pgm_path)

    occ_th = float(meta.get("occupied_thresh", 0.65))
    free_th = float(meta.get("free_thresh", 0.196))
    negate = int(meta.get("negate", 0))

    p = (255.0 - img.astype(np.float64)) / 255.0
    if negate:
        p = 1.0 - p

    # map_server 的三段判定，不能写成区间：
    #   p > occupied_thresh -> 占据
    #   p < free_thresh     -> 自由
    #   其余                -> unknown
    # 写成 "free_th <= p <= occ_th" 会把 254（自由）判成 unknown ——
    # 后果是覆盖率分子恒为空、永远报 0%，而逐格一致率却因为只剩占据格可比
    # 而显示 100%，看起来一切正常。这个坑必须显式写清楚。
    grid = np.full(img.shape, -1, dtype=np.int8)
    grid[p > occ_th] = 100
    grid[p < free_th] = 0

    meta["_pgm_path"] = pgm_path
    return grid, meta


def align_and_crop(gt, est, gt_meta, est_meta):
    """把两张栅格对齐到**以 GT 范围为基准**的公共世界网格。

    为什么基准选 GT 而不是两者的交集：
      用交集的话，估计地图只要覆盖住自己那一小块，其内部覆盖率就是 100%，
      评估结果会掩盖"机器人压根没走到一半区域"这个事实。
      以 GT 为基准、估计地图范围外的格子记为 unknown，覆盖率才真正反映
      探索是否完整。

    为什么必须重采样而不是直接逐格相减：
      两张图分辨率和原点通常不同（GT 是 0.10m，slam_toolbox 可能出 0.05m）。
      直接相减是纯噪声，但它会给出一个看起来"挺合理"的百分比，非常能骗人。
    """
    res = min(float(gt_meta["resolution"]), float(est_meta["resolution"]))
    hg, wg = gt.shape
    rg = float(gt_meta["resolution"])
    gx0, gy0 = float(gt_meta["origin"][0]), float(gt_meta["origin"][1])

    nx = int(round(wg * rg / res))
    ny = int(round(hg * rg / res))

    def resample_to(grid, meta):
        h, w = grid.shape
        r = float(meta["resolution"])
        ox, oy = float(meta["origin"][0]), float(meta["origin"][1])

        xs = gx0 + (np.arange(nx) + 0.5) * res
        ys = gy0 + (np.arange(ny) + 0.5) * res

        ix = np.floor((xs - ox) / r).astype(np.int64)
        iy_world = np.floor((ys - oy) / r).astype(np.int64)
        iy_img = h - 1 - iy_world        # 图像第 0 行 = 最大 y，必须翻回来

        out = np.full((ny, nx), -1, dtype=np.int8)   # 范围外 = unknown
        ok_x = (ix >= 0) & (ix < w)
        ok_y = (iy_world >= 0) & (iy_world < h)
        mask = ok_y[:, None] & ok_x[None, :]
        sub = grid[np.ix_(np.clip(iy_img, 0, h - 1), np.clip(ix, 0, w - 1))]
        out[mask] = sub[mask]
        return out

    return (resample_to(gt, gt_meta), resample_to(est, est_meta),
            {"resolution": res, "origin": [gx0, gy0, 0.0], "width": nx, "height": ny,
             "note": "以 GT 范围为公共网格；估计地图未覆盖处记为 unknown",
             # 幽灵墙统计需要原始（未裁剪）网格与元信息，挂在这里传下去
             "_raw_grid": est,
             "_raw_grid_meta": {
                 "res": float(est_meta["resolution"]),
                 "ox": float(est_meta["origin"][0]),
                 "oy": float(est_meta["origin"][1]),
             },
             })


def attach_ghost_context(gt_meta, gt_grid, meta_a):
    """把 GT 原始形状/分辨率/原点塞进 gt_meta，供 map_metrics 算幽灵墙。

    align_and_crop 的返回值里已经带了 EST 的原始网格，但 GT 的原始形状
    需要另外传 —— GT 的已知范围就是"幽灵墙"判定的边界。
    """
    h, w = gt_grid.shape
    gt_meta["_shape"] = (h, w)
    gt_meta["_res"] = float(gt_meta["resolution"])
    gt_meta["_ox"] = float(gt_meta["origin"][0])
    gt_meta["_oy"] = float(gt_meta["origin"][1])
    meta_a["_gt_ctx"] = True
    return gt_meta


# ---------------------------------------------------------------------------
# 地图指标
# ---------------------------------------------------------------------------


def map_metrics(gt, est, gt_meta=None, est_meta=None):
    """逐格对比。只在 GT 有明确标注（free/occupied）的格子上算，unknown 排除。

    排除 unknown 很关键：GT 里未标注的区域如果算进分母，覆盖率会被
    "机器人本来就不该去的地方"稀释掉。

    gt_meta / est_meta 传原始（裁剪前）的地图元信息，用于统计"幽灵墙"
    （估计地图落在 GT 范围外的占据格）。不传则跳过该项。
    """
    known = gt >= 0
    if not np.any(known):
        raise RuntimeError("GT 地图里没有任何已知格子，检查 PGM 是否读对了")

    gt_occ = (gt == 100) & known
    gt_free = (gt == 0) & known
    est_occ = (est == 100) & known

    total_known = int(known.sum())

    # 覆盖率：GT 可通行区域里，估计地图有多少被确认为"已观测"
    # （估计地图中 free 或 occupied 都算已观测）
    est_observed = (est >= 0) & known
    coverage = float((est_observed & gt_free).sum()) / max(1, int(gt_free.sum()))

    tp = int((gt_occ & est_occ).sum())
    fp = int((~gt_occ & est_occ & known).sum())
    fn = int((gt_occ & ~est_occ).sum())

    precision = tp / (tp + fp) if (tp + fp) else None
    recall = tp / (tp + fn) if (tp + fn) else None
    iou = tp / (tp + fp + fn) if (tp + fp + fn) else None

    # 逐格一致率：估计占据与 GT 占据是否同号（含 free 区域）
    est_class = np.where(est == 100, 1, np.where(est == 0, 0, -1))
    gt_class = np.where(gt == 100, 1, 0)
    comparable = known & (est_class >= 0)
    grid_acc = float((est_class[comparable] == gt_class[comparable]).sum()) / \
        max(1, int(comparable.sum()))

    # ---- 幽灵墙：估计地图里落在 GT 已知范围【之外】的占据格 ----
    #
    # 为什么单独统计这一项（本项目实测踩坑）：
    #   simple 环境 slam_toolbox 跑完后，估计地图有 14550 个占据格，
    #   但只有 5229 个落在 20x20m 的世界范围内，其余 9321 个（64%）
    #   分布在 x 到 28.5m、y 到 -16.5m 的场外空间 —— 那是位姿发散时
    #   把墙"复制"到场外的产物。
    #
    #   注意：map_metrics 的精度/召回只统计 GT 已知格，所以这些幽灵墙
    #   【不会】污染 precision/recall。但它们也不会出现在任何指标里，
    #   导致"地图明明炸了却看不出来"。
    #   显式报出来，才能区分"精度低是因为墙细"和"地图跑飞了"。
    #
    #   必须在 align_and_crop 【之前】用原始网格算：裁剪会把范围外的格
    #   变成 unknown(-1)，之后就分辨不出来了。
    ghost = {}
    if gt_meta is not None and est_meta is not None:
        raw = est_meta.get("_raw_grid")
        rmeta = est_meta.get("_raw_grid_meta")
        gshape = gt_meta.get("_shape")
        if raw is not None and rmeta is not None and gshape is not None:
            eh, ew = raw.shape
            er, eox, eoy = rmeta["res"], rmeta["ox"], rmeta["oy"]
            gr, gox, goy = gt_meta["_res"], gt_meta["_ox"], gt_meta["_oy"]
            ggh, ggw = gshape
            eys, exs = np.where(raw == 100)
            if len(eys):
                ewx = eox + (exs + 0.5) * er
                ewy = eoy + (eh - eys - 0.5) * er
                inb = ((ewx >= gox) & (ewx <= gox + ggw * gr) &
                       (ewy >= goy) & (ewy <= goy + ggh * gr))
                n_out = int((~inb).sum())
                n_tot = int(len(eys))
                ghost = {
                    "est_occupied_total": n_tot,
                    "est_occupied_outside_gt": n_out,
                    "ghost_wall_ratio": round(n_out / float(n_tot), 4) if n_tot else None,
                }
                if n_out / float(n_tot) > 0.1:
                    ghost["ghost_wall_warning"] = (
                        f"{n_out}/{n_tot}（{100.0*n_out/n_tot:.0f}%）的占据格落在"
                        " GT 已知范围之外，说明有整块子图被拖到场外，此图【不可直接"
                        "交给 Nav2】——全局代价地图会吃到大片虚假障碍、堵死可通行区。"
                        " 按以下顺序排查（本项目的实测归因）："
                        " ①【先查环境】ros2 topic info /map 是否 Publisher count > 1、"
                        "ros2 node list 是否有重名节点（多套仿真串扰会让位姿被随机拉扯，"
                        " 用 tools/clean_sim.sh 清场后重跑）；"
                        " ② 再查 scan_buffer_maximum_scan_distance 是否被调大到远超"
                        " 场地尺寸（官方默认 10.0，调大 = 局部链横跨全场，单次误匹配"
                        " 会拖动大段地图）；"
                        " ③ 最后查 spawn 高度与雷达实际高度是否一致。")
    if ghost:
        pass

    return {
        "gt_known_cells": total_known,
        "gt_free_cells": int(gt_free.sum()),
        "gt_occupied_cells": int(gt_occ.sum()),
        "coverage_ratio": round(coverage, 4),
        "occupied_precision": None if precision is None else round(precision, 4),
        "occupied_recall": None if recall is None else round(recall, 4),
        "occupied_iou": None if iou is None else round(iou, 4),
        "grid_accuracy": round(grid_acc, 4),
        "false_wall_cells": fp,
        "missed_wall_cells": fn,
        **ghost,
    }


def tolerance_metrics(gt, est, res):
    """带定位容差的占据指标 —— 用来把「墙太细」和「位姿漂了」分开。

    为什么需要这一层（本项目踩过的真实坑）：
      只有精确逐格 recall 时，simple 环境 slam_toolbox 拿到的数字是
        recall=0.24  precision=0.35  IoU=0.17
      单看这组数字会得出"SLAM 很差"的结论。但把它拆开后完全是另一回事：
        · EST 的占据格，70% 落在 GT 墙 0.15m 以内，中位距离 0.10m
          → 几何位置是准的，没有系统性错位
        · 把 EST 的墙做 3x3 膨胀（等效允许 0.1m 定位误差）后
          recall 从 0.25 跳到 0.69
        · 把 EST 做形态学腐蚀 1 格后只剩 0.9%
          → EST 的墙是 1~2 格宽的"表面细线"
      而 GT 是"实心体块"：墙 0.3m 厚、柱子 0.5x0.5m 实心。
      激光雷达在物理上就只能看到墙面、看不到墙内部，
      所以拿实心体块当 GT 去算精确逐格 recall，会系统性低估建图质量——
      分母里 2/3 的格（墙的内部）是任何射线都打不到的。

    三个容差档位的含义：
      tol=0   精确逐格。报告"字面保真度"，适合版本间回归对比。
      tol=1格 GT 墙膨胀 1 格后 EST 是否命中。0.05 网格上即允许 5cm 误差，
              覆盖雷达噪声与栅格化抖动，是最贴近"几何对不对"的口径。
      tol=2格 允许 10cm。这一档若仍低，才说明真的漂了或丢墙。

    同时给出 est_thick_ratio（EST 墙"实心度"）：
      对 EST 做 1 格腐蚀后剩余的格数占比。真的实心墙块（GT 那种）
      腐蚀后剩 ~12%，而细线只有 ~1%。这个数字直接量化"细线 vs 实心块"，
      是解释 recall 偏低的第一手证据，比猜"是不是位姿问题"可靠得多。
    """
    out = {}
    if _ndimage is None:
        out["note"] = "未安装 scipy，跳过容差指标（pip install scipy）"
        return out

    gt_occ = (gt == 100)
    est_occ = (est == 100)
    st = np.ones((3, 3), dtype=bool)
    st2 = np.ones((5, 5), dtype=bool)

    for tol, structure in ((1, st), (2, st2)):
        gt_d = _ndimage.binary_dilation(gt_occ, structure=structure)
        est_d = _ndimage.binary_dilation(est_occ, structure=structure)
        # precision：EST 的墙附近有 GT 墙就算命中
        p_hit = int((est_occ & gt_d).sum())
        p_tot = int(est_occ.sum())
        # recall：GT 的墙附近有 EST 墙就算建到
        r_hit = int((gt_occ & est_d).sum())
        r_tot = int(gt_occ.sum())
        prec = p_hit / p_tot if p_tot else None
        rec = r_hit / r_tot if r_tot else None
        out[f"precision_tol{tol}"] = None if prec is None else round(prec, 4)
        out[f"recall_tol{tol}"] = None if rec is None else round(rec, 4)
        out[f"tol{tol}_meters"] = round(tol * res, 3)

    # EST 墙的"实心度"：腐蚀 1 格后还剩多少
    if est_occ.any():
        er = _ndimage.binary_erosion(est_occ, structure=st)
        out["est_thick_ratio"] = round(float(er.sum()) / float(est_occ.sum()), 4)
    if gt_occ.any():
        er = _ndimage.binary_erosion(gt_occ, structure=st)
        out["gt_thick_ratio"] = round(float(er.sum()) / float(gt_occ.sum()), 4)

    # EST 墙格到最近 GT 墙的距离中位数（米）—— 直接量化"错位了多少"
    if est_occ.any() and gt_occ.any():
        dist = _ndimage.distance_transform_edt(~gt_occ) * res
        d = dist[est_occ]
        out["est_to_gt_median_dist_m"] = round(float(np.median(d)), 3)
        out["est_to_gt_far_ratio"] = round(float((d > 0.5).sum()) / float(len(d)), 4)

    # 形态学解释：把「墙厚差异」单独剥掉后，几何还能剩下多少
    # （回答"如果 EST 就是细线、GT 就是实心块，那理论上的 recall 上限是多少"）
    if est_occ.any():
        # 把 EST 加粗到与 GT 同等厚度（+1 格），再看逐格 recall
        est_thick = _ndimage.binary_dilation(est_occ, structure=st)
        hit = int((est_thick & gt_occ).sum())
        tot = int(gt_occ.sum())
        out["recall_after_thickening"] = round(hit / float(tot), 4) if tot else None
        out["recall_after_thickening_note"] = (
            "把 EST 墙加粗 1 格（0.05m）后重算的精确 recall。"
            "与裸 recall 的差距 = 纯粹由「细线 vs 实心块」造成的损失，"
            "不是位姿或漏建问题。")

    return out


# ---------------------------------------------------------------------------
# 轨迹指标
# ---------------------------------------------------------------------------


def read_traj(path):
    if not path or not os.path.isfile(path):
        return None
    rows = []
    with open(path, "r", newline="") as f:
        for r in csv.DictReader(f):
            try:
                rows.append((float(r["t"]), float(r["x"]), float(r["y"]), float(r.get("yaw", 0.0))))
            except (KeyError, ValueError):
                continue
    return np.asarray(rows, dtype=np.float64) if rows else None


def umeyama(src, dst):
    """求把 src 对齐到 dst 的相似变换 (R, t, s)。用于 ATE 的轨迹对齐。

    不做对齐的 ATE 反映的是"整体偏移"，几十厘米是正常的；
    对齐后的 ATE 才反映"形状像不像"。两个都要看，所以报告里都出。
    """
    mu_s, mu_d = src.mean(axis=0), dst.mean(axis=0)
    s_c, d_c = src - mu_s, dst - mu_d
    cov = d_c.T @ s_c / len(src)
    u, d, vt = np.linalg.svd(cov)
    s = np.eye(2)
    if np.linalg.det(u) * np.linalg.det(vt) < 0:
        s[-1, -1] = -1
    r = u @ s @ vt
    var_s = (s_c ** 2).sum() / len(src)
    scale = 1.0 if var_s < 1e-12 else float((d * np.diag(s)).sum() / var_s)
    t = mu_d - scale * (r @ mu_s)
    return r, t, scale


def match_by_time(gt, est):
    """按时间最近邻配对两条轨迹。前提是两者采样时刻有重叠。"""
    if gt is None or est is None:
        return None
    t0 = max(gt[0, 0], est[0, 0])
    t1 = min(gt[-1, 0], est[-1, 0])
    if t1 <= t0:
        return None
    idx = np.searchsorted(gt[:, 0], est[:, 0])
    idx = np.clip(idx, 0, len(gt) - 1)
    left = np.maximum(idx - 1, 0)
    idx = np.where(np.abs(gt[left, 0] - est[:, 0]) < np.abs(gt[idx, 0] - est[:, 0]), left, idx)
    ok = (est[:, 0] >= t0) & (est[:, 0] <= t1) & (np.abs(gt[idx, 0] - est[:, 0]) < 0.5)
    if ok.sum() < 10:
        return None
    return gt[idx[ok]][:, 1:3], est[ok][:, 1:3], est[ok][:, 0]


def traj_metrics(gt, est):
    m = {}
    if gt is not None:
        m["gt_points"] = len(gt)
        m["gt_path_length"] = round(float(np.sum(np.hypot(np.diff(gt[:, 1]), np.diff(gt[:, 2])))), 3)
    if est is not None:
        m["est_points"] = len(est)
        m["est_path_length"] = round(float(np.sum(np.hypot(np.diff(est[:, 1]), np.diff(est[:, 2])))), 3)

    matched = match_by_time(gt, est)
    if matched is None:
        m["trajectory_note"] = "两条轨迹没有足够的时间重叠，跳过 ATE/RPE"
        return m

    g, e, _ = matched

    # --- 未对齐 ATE：反映整体偏移（含初始位姿差）---
    d_raw = np.hypot(g[:, 0] - e[:, 0], g[:, 1] - e[:, 1])
    m["ate_rmse_raw"] = round(float(np.sqrt(np.mean(d_raw ** 2))), 4)
    m["ate_max_raw"] = round(float(d_raw.max()), 4)

    # --- Umeyama 对齐后的 ATE：反映轨迹形状误差 ---
    try:
        r, t, s = umeyama(e, g)
        e_a = (s * (r @ e.T)).T + t
        d_al = np.hypot(g[:, 0] - e_a[:, 0], g[:, 1] - e_a[:, 1])
        m["ate_rmse_aligned"] = round(float(np.sqrt(np.mean(d_al ** 2))), 4)
        m["ate_max_aligned"] = round(float(d_al.max()), 4)
        m["ate_scale"] = round(float(s), 4)
        # 尺度明显偏离 1 说明里程计在系统性虚增/虚减里程，是个独立的失效模式
        if abs(s - 1.0) > 0.02:
            m["ate_scale_warning"] = (
                f"对齐尺度 {s:.4f} 偏离 1 超过 2%：里程计存在系统性尺度误差，"
                "检查雷达外参和时间同步，而不是去调滤波器增益"
            )
    except np.linalg.LinAlgError:
        pass

    # --- RPE：相邻帧位移的一致性，反映局部漂移 ---
    if len(g) > 3:
        dg = np.diff(g, axis=0)
        de = np.diff(e, axis=0)
        rpe = np.hypot(dg[:, 0] - de[:, 0], dg[:, 1] - de[:, 1])
        m["rpe_trans_rmse"] = round(float(np.sqrt(np.mean(rpe ** 2))), 5)
        m["rpe_trans_max"] = round(float(rpe.max()), 5)

    m["matched_pairs"] = int(len(g))
    return m


# ---------------------------------------------------------------------------
# evo 集成（可选）
# ---------------------------------------------------------------------------


def try_evo(gt_csv, est_csv, out_dir):
    """如果装了 evo，顺手生成一张轨迹对比图。没装就跳过，不影响主流程。"""
    try:
        from evo.core import metrics as evo_metrics, sync
        from evo.tools import file_interface, plot
    except ImportError:
        return None, "未安装 evo（pip install evo --upgrade），跳过轨迹绘图"

    try:
        import matplotlib
        matplotlib.use("Agg")

        traj_ref = file_interface.read_tum_trajectory_file(gt_csv)
        traj_est = file_interface.read_tum_trajectory_file(est_csv)
        traj_ref, traj_est = sync.associate_trajectories(
            traj_ref, traj_est, max_diff=0.5)
        traj_est.align(traj_ref, correct_scale=False, correct_only_scale=False)

        ape = evo_metrics.APE(evo_metrics.PoseRelation.translation_part)
        ape.process_data((traj_ref, traj_est))
        import matplotlib.pyplot as plt
        out_png = os.path.join(out_dir, "trajectory_compare.png")
        fig = plt.figure(figsize=(10, 5))
        ax1 = fig.add_subplot(121)
        plot.traj(ax1, plot.PlotMode.xy, traj_ref, style="--", color="red", label="GT")
        plot.traj(ax1, plot.PlotMode.xy, traj_est, style="-", color="blue", label="估计")
        ax1.legend()
        ax2 = fig.add_subplot(122)
        plot.error_array(ax2, ape.error, title="APE (m)", xlabel="index")
        fig.tight_layout()
        fig.savefig(out_png, dpi=130)
        plt.close(fig)
        return out_png, f"evo 对齐后 ATE RMSE = {ape.get_statistic(evo_metrics.StatisticsType.rmse):.4f} m"
    except Exception as e:  # evo 内部异常不该拖垮主评测
        return None, f"evo 绘图失败（已忽略）：{type(e).__name__}: {e}"


# ---------------------------------------------------------------------------
# 主流程
# ---------------------------------------------------------------------------


def main():
    ap = argparse.ArgumentParser(description="对照 GT 评测建图与导航结果")
    ap.add_argument("--gt-map", required=True, help="GT 地图 yaml（sim_worlds/maps/*.yaml）")
    ap.add_argument("--est-map", default="", help="估计地图 yaml（map_saver 产出）")
    ap.add_argument("--gt-traj", default="", help="真值轨迹 CSV")
    ap.add_argument("--est-traj", default="", help="估计轨迹 CSV")
    ap.add_argument("--out", required=True, help="输出 JSON 报告路径")
    ap.add_argument("--quiet", action="store_true")
    args = ap.parse_args()

    if yaml is None:
        print("需要 pyyaml：pip install pyyaml", file=sys.stderr)
        sys.exit(2)

    report = {}

    # ---------- 地图 ----------
    if args.est_map:
        gt_grid, gt_meta = load_occupancy(args.gt_map)
        est_grid, est_meta = load_occupancy(args.est_map)
        gt_a, est_a, meta_a = align_and_crop(gt_grid, est_grid, gt_meta, est_meta)
        attach_ghost_context(gt_meta, gt_grid, meta_a)
        # est_meta 要带上原始网格与元信息，map_metrics 才能统计幽灵墙
        est_meta_for_metrics = dict(est_meta)
        est_meta_for_metrics["_raw_grid"] = est_grid
        est_meta_for_metrics["_raw_grid_meta"] = meta_a.get("_raw_grid_meta")
        mm = map_metrics(gt_a, est_a, gt_meta, est_meta_for_metrics)
        # 带容差的指标：把「墙细」与「位姿漂」分开，避免把 SLAM 质量看低
        mm.update(tolerance_metrics(gt_a, est_a, meta_a["resolution"]))
        report["map"] = mm
        report["map"]["aligned_grid"] = {k: v for k, v in meta_a.items()
                                         if not k.startswith("_")}
        report["map"]["gt_source"] = os.path.abspath(args.gt_map)
        report["map"]["est_source"] = os.path.abspath(args.est_map)
    else:
        report["map"] = {"note": "未提供 --est-map，跳过地图指标"}

    # ---------- 轨迹 ----------
    gt_traj = read_traj(args.gt_traj)
    est_traj = read_traj(args.est_traj)
    report["trajectory"] = traj_metrics(gt_traj, est_traj)

    # ---------- evo ----------
    if args.gt_traj and args.est_traj:
        png, msg = try_evo(args.gt_traj, args.est_traj, os.path.dirname(os.path.abspath(args.out)))
        report["evo"] = {"plot": png, "message": msg}

    os.makedirs(os.path.dirname(os.path.abspath(args.out)), exist_ok=True)
    with open(args.out, "w", encoding="utf-8") as f:
        json.dump(report, f, ensure_ascii=False, indent=2)

    if not args.quiet:
        print(f"\n{'='*66}\n评测报告  ->  {args.out}\n{'='*66}")
        mp = report["map"]
        if "coverage_ratio" in mp:
            print("\n[建图]")
            print(f"  覆盖率 coverage_ratio : {mp['coverage_ratio']*100:6.2f} %   "
                  f"（GT 可通行区的已观测比例，低说明漏建）")
            print(f"  占据 IoU              : {mp['occupied_iou']*100:6.2f} %   "
                  f"（综合指标）")
            print(f"  占据精确率 precision  : {mp['occupied_precision']*100:6.2f} %   "
                  f"（低=假墙多，常见于动态障碍污染）")
            print(f"  占据召回率 recall     : {mp['occupied_recall']*100:6.2f} %   "
                  f"（低=漏建墙，常见于视场受限）")
            print(f"  逐格一致率            : {mp['grid_accuracy']*100:6.2f} %")
            print(f"  假墙格 {mp['false_wall_cells']}  漏建格 {mp['missed_wall_cells']}")

            if "recall_tol1" in mp:
                print("\n  [容差指标] 把「墙画得细」与「位姿漂了」分开看：")
                for tol in (1, 2):
                    rk, pk = f"recall_tol{tol}", f"precision_tol{tol}"
                    if rk in mp and mp[rk] is not None:
                        print(f"    允许 {mp[f'tol{tol}_meters']*100:4.1f}cm 定位误差："
                              f"recall={mp[rk]*100:6.2f}%  precision={mp[pk]*100:6.2f}%")
                if "recall_after_thickening" in mp:
                    gain = mp["recall_after_thickening"] - (mp["occupied_recall"] or 0)
                    print(f"    EST 墙加粗 1 格后 recall : "
                          f"{mp['recall_after_thickening']*100:6.2f}%  (+{gain*100:.2f}pt)")
                    print(f"      → 这部分差距纯粹来自「细线 vs 实心块」，不是漏建")
                if "est_thick_ratio" in mp:
                    print(f"    EST 墙实心度 {mp['est_thick_ratio']*100:.2f}%  "
                          f"(GT 墙实心度 {mp.get('gt_thick_ratio', 0)*100:.2f}%)")
                if "est_to_gt_median_dist_m" in mp:
                    print(f"    EST 墙到最近 GT 墙的中位距离 : "
                          f"{mp['est_to_gt_median_dist_m']:.3f} m"
                          f"   远于 0.5m 的伪迹占比 {mp.get('est_to_gt_far_ratio', 0)*100:.2f}%")

            if mp.get("ghost_wall_ratio") is not None:
                print(f"\n  [幽灵墙] 估计占据格总数 {mp['est_occupied_total']}，"
                      f"落在 GT 范围外 {mp['est_occupied_outside_gt']} "
                      f"({mp['ghost_wall_ratio']*100:.1f}%)")
                if mp.get("ghost_wall_warning"):
                    print(f"    ⚠️ {mp['ghost_wall_warning']}")
        else:
            print(f"\n[建图] {mp.get('note')}")

        tp = report["trajectory"]
        print("\n[轨迹]")
        for k, v in tp.items():
            print(f"  {k:22s}: {v}")

        if "evo" in report:
            print(f"\n[evo] {report['evo']['message']}")
        print()


if __name__ == "__main__":
    main()
