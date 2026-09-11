#!/usr/bin/env python3
"""crop_map_to_traj.py —— 把建图结果裁剪到探索轨迹的活动范围内。

为什么需要这个工具（本次 simple 轮的实测教训）：
  探索轨迹只在 20x20m 房间内活动（x[-9.6,9.6] y[-8.7,8.8]），但
  slam_toolbox 产出的地图却是 60x84m：少数 scan matching 失败的节点
  把位姿拉到房间外几十米，再从错误位置把整束激光射线画出来，形成
  放射状"幽灵墙"（29853/33937 个占据格落在 GT 之外，ghost_wall_ratio
  0.88）。这种地图直接交给 Nav2，全局代价地图会被虚假障碍堵死。

  两层修复中的【治标层】：
    治本 = 收紧 slam_toolbox 的 scan matching 参数（correlation 搜索空间
           / 匹配响应阈值），让失败匹配不被信任 —— 见 slam_toolbox_params.yaml。
    治本改完必须重跑（20 分钟/轮）；而裁剪只依赖两个已有产物
    （map.yaml + exploration_traj.csv），几秒钟就能把已有的废图救成可用图。

  裁剪的诚实性说明：
    轨迹包围盒（+安全边距）之外的区域机器人从未到过，那里既不该有墙、
    也不该有可通行空间 —— 全部改回 unknown(205) 是与事实一致的表示，
    不是"美化分数"。包围盒之内一个格子都不动。

用法：
    python3 tools/crop_map_to_traj.py <run目录>
    python3 tools/crop_map_to_traj.py <run目录> --margin 2.0

产物：
    <run目录>/map.pgm / map.yaml   原地裁剪（原图备份为 map.pgm.uncropped）
    并打印裁剪前后的占据格统计。
"""

import argparse
import os
import sys

import numpy as np
import yaml

# ---- PGM(P5) 读写（自研，不依赖 PIL，与项目其他工具一致）----


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
    w, h, _ = vals
    # ⚠️ frombuffer 产出的是只读视图，后续要原地改写，必须 copy
    arr = np.frombuffer(d[idx:idx + w * h], dtype=np.uint8).reshape(h, w).copy()
    return arr


def write_pgm(path, arr):
    h, w = arr.shape
    with open(path, "wb") as f:
        f.write(f"P5\n{w} {h}\n255\n".encode("ascii"))
        f.write(arr.tobytes())


def read_traj_xy(csv_path):
    """读 exploration_traj.csv 的 (x, y) 列。

    列约定与 traj_brief 一致：0=时间戳, 1=x, 2=y（其余忽略）。
    """
    xs, ys = [], []
    with open(csv_path, encoding="utf-8") as f:
        for i, line in enumerate(f):
            if i == 0:
                continue
            parts = line.strip().split(",")
            if len(parts) < 3:
                continue
            try:
                xs.append(float(parts[1]))
                ys.append(float(parts[2]))
            except ValueError:
                continue
    return np.array(xs), np.array(ys)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("run_dir", help="含 map.yaml 与 exploration_traj.csv 的目录")
    ap.add_argument("--margin", type=float, default=1.5,
                    help="轨迹包围盒外扩的安全边距（米）")
    args = ap.parse_args()

    map_yaml = os.path.join(args.run_dir, "map.yaml")
    traj_csv = os.path.join(args.run_dir, "exploration_traj.csv")
    map_pgm = os.path.join(args.run_dir, "map.pgm")
    for p in (map_yaml, traj_csv, map_pgm):
        if not os.path.isfile(p):
            print(f"❌ 缺少 {p}")
            return 1

    meta = yaml.safe_load(open(map_yaml, encoding="utf-8"))
    res = float(meta["resolution"])
    ox, oy = float(meta["origin"][0]), float(meta["origin"][1])

    img = read_pgm(map_pgm)
    h, w = img.shape

    xs, ys = read_traj_xy(traj_csv)
    if len(xs) == 0:
        print("❌ 轨迹为空，拒绝裁剪（没有依据的裁剪是瞎剪）")
        return 1
    x0, x1 = xs.min() - args.margin, xs.max() + args.margin
    y0, y1 = ys.min() - args.margin, ys.max() + args.margin
    print(f"轨迹 {len(xs)} 点  包围盒 x[{xs.min():.2f},{xs.max():.2f}] "
          f"y[{ys.min():.2f},{ys.max():.2f}]")
    print(f"裁剪窗（含 {args.margin}m 边距）: x[{x0:.2f},{x1:.2f}] "
          f"y[{y0:.2f},{y1:.2f}]")

    # ---- 像素 → 世界坐标 ----
    # ROS map_server 约定：图像第一行对应 y 最大处。
    # world_x = ox + (col + 0.5) * res
    # world_y = oy + (h - row - 0.5) * res
    cols = np.arange(w)
    rows = np.arange(h)
    wx = ox + (cols + 0.5) * res              # (w,)
    wy = oy + (h - rows - 0.5) * res          # (h,)
    inside_x = (wx >= x0) & (wx <= x1)
    inside_y = (wy >= y0) & (wy <= y1)
    keep = inside_y[:, None] & inside_x[None, :]   # (h, w) True=保留

    occ = (img == 0)
    free = (img == 254)
    occ_before, free_before = int(occ.sum()), int(free.sum())

    # 包围盒之外：一切"已观测"信息都是幽灵射线的产物 → 全部回退为 unknown
    img[~keep] = 205

    occ_after = int((img == 0).sum())
    free_after = int((img == 254).sum())
    print(f"占据格: {occ_before} -> {occ_after}  （删除 {occ_before - occ_after}）")
    print(f"自由格: {free_before} -> {free_after}  （删除 {free_before - free_after}）")

    # 备份原版，写回裁剪版（yaml 无需改：分辨率/原点都不变）
    bak = map_pgm + ".uncropped"
    if not os.path.exists(bak):
        os.replace(map_pgm, bak)
        print(f"原图已备份 -> {bak}")
    write_pgm(map_pgm, img)
    print(f"已写回 -> {map_pgm}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
