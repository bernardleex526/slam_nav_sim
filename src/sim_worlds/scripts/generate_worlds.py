#!/usr/bin/env python3
"""
三张测试地图的参数化生成器
=====================================================================
核心思想：**同一个栅格表示，同时吐出 SDF world 和 GT 占据栅格。**

为什么不用手写 world：
  手工拼三张 SDF，你无法保证"简单 / 中等 / 复杂"是同一个难度梯度上的连续
  样本 —— 难度会随心情漂移，跨算法对比就失去意义。
  参数化生成才能保证：同一份代码、同一组参数、可复现。

为什么必须同时输出 GT 占据栅格：
  没有 GT，"建图效果好不好"只能靠肉眼看，无法量化。
  有了 GT 才能算覆盖率、栅格一致率、对 GT 的 RMSE —— 这才是需求 #4
  「方便进行对比」能真正落地的前提。

难度定义（按"要暴露什么失效模式"，而不是按面积）：
  simple  : 开阔空间 + 少量隔墙，特征充足，无窄通道
            → 算法正确性底线。跑不通就是实现有 bug，别怪环境。
  medium  : 多房间 + 走廊 + 门洞 + 动态障碍
            → 回环检测、重定位、动态干扰下的稳定性。
  complex : 迷宫 + 0.8m 窄通道 + 长直走廊(几何退化) + 空旷大厅(特征缺失)
            → 几何退化、空旷场景漂移、感知盲区下的鲁棒性。
              长直走廊和空旷大厅是 LIO 最容易翻车的两个场景。

产出：
  worlds/<name>.world         Gazebo SDF 世界
  maps/<name>.pgm + .yaml     GT 占据栅格（标准 map_server 格式）
  maps/<name>_meta.json       地图统计 + 推荐出生点 + 动态障碍清单

用法：
  python3 generate_worlds.py                    # 生成全部三张
  python3 generate_worlds.py --maps simple      # 只生成一张
  python3 generate_worlds.py --res 0.05         # 更高分辨率
"""

import argparse
import json
import os
import xml.etree.ElementTree as ET
from collections import deque

import numpy as np

FREE = 0
OCC = 100

# ---------------------------------------------------------------------------
# 栅格表示
# ---------------------------------------------------------------------------


class GridMap:
    """以"世界米"为坐标系构建的 2D 占据栅格。

    内部 grid[iy][ix]，iy=0 对应最小 y。
    世界坐标 -> 索引：ix = (x - ox) / res
    """

    def __init__(self, width_m, height_m, res, wall_height=1.5):
        self.res = res
        self.wall_height = wall_height
        self.nx = int(round(width_m / res))
        self.ny = int(round(height_m / res))
        # 地图以原点为中心，出生点默认可放 (0,0)
        self.ox = -width_m / 2.0
        self.oy = -height_m / 2.0
        self.width_m = width_m
        self.height_m = height_m
        self.grid = np.full((self.ny, self.nx), FREE, dtype=np.int8)

    # -- 坐标换算 --
    def ix(self, x):
        return int(round((x - self.ox) / self.res))

    def iy(self, y):
        return int(round((y - self.oy) / self.res))

    def cx(self, ix):
        """单元格中心的 x（世界坐标）"""
        return self.ox + (ix + 0.5) * self.res

    def cy(self, iy):
        return self.oy + (iy + 0.5) * self.res

    def _clip(self, i0, i1, n):
        return max(0, min(i0, n - 1)), max(0, min(i1, n - 1))

    # -- 绘制原语：全部用世界米作参数 --
    def fill(self, val):
        """整图填充。

        约定：地图构造必须是 **先 fill(OCC) 再挖出可通行空间**。
        反过来（默认全自由、只画墙线）会导致"墙外的外部区域"也被标成可通行，
        GT 说能走、机器人永远走不到 —— 覆盖率指标直接失真，而且不易察觉。
        """
        self.grid[:, :] = val

    def rect(self, x0, y0, x1, y1, val=OCC, fill=True):
        """在 [x0,x1] x [y0,y1] 区域绘制。fill=False 时只画边框。"""
        ix0, ix1 = self._clip(self.ix(min(x0, x1)), self.ix(max(x0, x1)), self.nx)
        iy0, iy1 = self._clip(self.iy(min(y0, y1)), self.iy(max(y0, y1)), self.ny)
        if fill:
            self.grid[iy0:iy1 + 1, ix0:ix1 + 1] = val
        else:
            self.grid[iy0:iy1 + 1, ix0] = val
            self.grid[iy0:iy1 + 1, ix1] = val
            self.grid[iy0, ix0:ix1 + 1] = val
            self.grid[iy1, ix0:ix1 + 1] = val

    def wall_h(self, x0, x1, y, thickness, val=OCC):
        """水平墙：沿 x 方向，中心线在 y。"""
        self.rect(x0, y - thickness / 2.0, x1, y + thickness / 2.0, val)

    def wall_v(self, x, y0, y1, thickness, val=OCC):
        """竖直墙：沿 y 方向，中心线在 x。"""
        self.rect(x - thickness / 2.0, y0, x + thickness / 2.0, y1, val)

    def pillar(self, x, y, size, val=OCC):
        self.rect(x - size / 2.0, y - size / 2.0, x + size / 2.0, y + size / 2.0, val)

    # -- 查询 --
    def is_free(self, x, y, pad=0.0):
        ix, iy = self.ix(x), self.iy(y)
        r = int(round(pad / self.res))
        ix0, ix1 = self._clip(ix - r, ix + r, self.nx)
        iy0, iy1 = self._clip(iy - r, iy + r, self.ny)
        return bool(np.all(self.grid[iy0:iy1 + 1, ix0:ix1 + 1] == FREE))

    def nearest_free(self, x, y, max_r=8.0):
        """从 (x,y) 螺旋外扩找最近的可通行点，避免出生点卡墙里。"""
        if self.is_free(x, y, pad=0.45):
            return x, y
        step = self.res * 4
        n = int(max_r / step)
        for ring in range(1, n + 1):
            for dx in range(-ring, ring + 1):
                for dy in range(-ring, ring + 1):
                    if max(abs(dx), abs(dy)) != ring:
                        continue
                    nx_, ny_ = x + dx * step, y + dy * step
                    if self.is_free(nx_, ny_, pad=0.45):
                        return nx_, ny_
        raise RuntimeError(f"在 ({x:.1f},{y:.1f}) 附近 {max_r}m 内找不到可通行点")

    def free_area_ratio(self):
        return float(np.count_nonzero(self.grid == FREE)) / float(self.grid.size)


# ---------------------------------------------------------------------------
# 贪心矩形分解：把占据栅格转成尽量少的 box（给 SDF 用）
# ---------------------------------------------------------------------------


def connectivity_report(gm, spawn):
    """从出生点做 8 邻域 flood fill，检查可通行区域是否全域连通。

    为什么必须查：一面"贯通到边"的隔墙会随手封死一个房间，产生永远走不到的
    区域。这种地图在仿真里表现正常，但导航测试会莫名其妙出现大量"目标不可达"，
    而你会去怀疑规划器 —— 实际上是地图根本不连通。
    在生成阶段就把它抓出来，成本最低。
    """
    free = gm.grid == FREE
    total = int(free.sum())
    if total == 0:
        return 0.0, 0

    si, sj = gm.ix(spawn[0]), gm.iy(spawn[1])
    if not free[sj, si]:
        return 0.0, total

    seen = np.zeros_like(free)
    seen[sj, si] = True
    queue = deque([(sj, si)])
    reachable = 1
    while queue:
        y, x = queue.popleft()
        for dy in (-1, 0, 1):
            for dx in (-1, 0, 1):
                ny, nx = y + dy, x + dx
                if 0 <= ny < gm.ny and 0 <= nx < gm.nx and free[ny, nx] and not seen[ny, nx]:
                    seen[ny, nx] = True
                    reachable += 1
                    queue.append((ny, nx))
    return reachable / float(total), total


def greedy_rects(mask):
    ny, nx = mask.shape
    used = np.zeros_like(mask, dtype=bool)
    rects = []
    for iy in range(ny):
        for ix in range(nx):
            if not mask[iy, ix] or used[iy, ix]:
                continue
            x1 = ix
            while x1 + 1 < nx and mask[iy, x1 + 1] and not used[iy, x1 + 1]:
                x1 += 1
            y1 = iy
            while y1 + 1 < ny:
                if np.all(mask[y1 + 1, ix:x1 + 1]) and not np.any(used[y1 + 1, ix:x1 + 1]):
                    y1 += 1
                else:
                    break
            used[iy:y1 + 1, ix:x1 + 1] = True
            rects.append((ix, iy, x1, y1))
    return rects


# ---------------------------------------------------------------------------
# 三张地图的构建
# ---------------------------------------------------------------------------


def build_simple(res, wall_h):
    """简单：开阔 + 少量隔墙 + 4 根柱子。无任何窄通道。"""
    gm = GridMap(20.0, 20.0, res, wall_h)
    t = 0.20
    # 先填满障碍、再挖出室内空间 —— 外墙厚度 0.2m，墙外一律判为不可通行
    gm.fill(OCC)
    gm.rect(-9.8, -9.8, 9.8, 9.8, FREE)

    # 内部隔墙：所有开口都 >= 3m，激光和规划器都毫无压力
    gm.wall_v(-3.0, -7.0, -1.0, t)      # 左下竖墙
    gm.wall_v(3.0, 1.0, 7.0, t)         # 右上竖墙
    gm.wall_h(-8.0, -4.0, 4.0, t)       # 左上横墙
    gm.wall_h(4.0, 8.0, -4.0, t)        # 右下横墙

    # 柱子：提供回环所需的结构特征，但不构成窄通道
    for (px, py) in [(-5.0, 5.0), (5.0, 5.0), (-5.0, -5.0), (5.0, -5.0)]:
        gm.pillar(px, py, 0.5)

    spawn = gm.nearest_free(0.0, 0.0)
    meta = {
        "dynamic_obstacles": 0,
        "description": "开阔空间 + 少量隔墙 + 4 根柱子，最小通道宽度 >= 3m",
        "expected_failure_modes": ["算法正确性底线：跑不通即实现有 bug"],
        "landmarks": 4,
    }
    return gm, spawn, meta


def build_medium(res, wall_h):
    """中等：多房间 + 走廊 + 1.2m 门洞 + 动态障碍。"""
    gm = GridMap(30.0, 24.0, res, wall_h)
    t = 0.20
    gm.fill(OCC)
    gm.rect(-14.8, -11.8, 14.8, 11.8, FREE)

    # 三条竖向隔墙，各留一个 1.2m 门洞（位置错开，逼出真实路径规划）
    def wall_with_door_v(x, y0, y1, door_y, door_w):
        gm.wall_v(x, y0, door_y - door_w / 2.0, t)
        gm.wall_v(x, door_y + door_w / 2.0, y1, t)

    wall_with_door_v(-10.0, -11.8, 11.8, -6.0, 1.2)
    wall_with_door_v(0.0, -11.8, 11.8, 2.0, 1.2)
    wall_with_door_v(10.0, -11.8, 11.8, 6.0, 1.2)

    # 一条横向隔断墙，制造"必须绕行"的拓扑。
    # 注意这里【故意不贯通到 x=-10】——如果贯通，左上房间就被封死了，
    # 变成一块永远走不到的区域，导航测试会出现"目标不可达"的假故障。
    # 留 1.8m 缺口，既保留绕行拓扑又保证连通。
    gm.wall_h(-14.8, -12.2, 0.0, t)
    gm.wall_h(12.2, 14.8, 0.0, t)

    # 家具块：局部窄化通道 + 增加结构特征
    furniture = [
        (-13.0, -9.0, 1.0, 0.6), (-12.0, -3.0, 0.8, 0.8),
        (-7.0, -8.0, 1.2, 0.6), (-6.0, 5.0, 0.8, 1.0),
        (-3.0, -2.0, 0.6, 0.6), (3.0, -8.0, 1.0, 0.8),
        (5.0, 4.0, 0.8, 1.2), (8.0, -3.0, 1.2, 0.6),
        (12.0, 8.0, 0.8, 0.8), (12.5, -8.0, 0.6, 1.0),
    ]
    for (fx, fy, fw, fd) in furniture:
        gm.rect(fx - fw / 2, fy - fd / 2, fx + fw / 2, fy + fd / 2, OCC)

    spawn = gm.nearest_free(-13.0, -10.5)
    meta = {
        "dynamic_obstacles": 2,
        "description": "多房间 + 走廊 + 1.2m 门洞 + 家具 + 2 个动态障碍",
        "expected_failure_modes": [
            "门洞处的回环检测",
            "重定位鲁棒性",
            "动态障碍造成的 map 污染",
        ],
        "landmarks": 10 + 3,
    }
    return gm, spawn, meta


def _carve_maze(gm, x0, y0, x1, y1, cell, passage, seed):
    """递归回溯法挖迷宫。墙厚 = 单元格间距，通道宽度 = passage。"""
    rng = np.random.default_rng(seed)
    ncols = int((x1 - x0) // cell)
    nrows = int((y1 - y0) // cell)
    if ncols < 2 or nrows < 2:
        return

    # 先整块填满，再挖通道
    gm.rect(x0, y0, x0 + ncols * cell, y0 + nrows * cell, OCC)

    visited = np.zeros((nrows, ncols), dtype=bool)
    stack = [(0, 0)]
    visited[0, 0] = True

    def carve(cx, cy):
        wx = x0 + cx * cell + cell / 2.0
        wy = y0 + cy * cell + cell / 2.0
        gm.rect(wx - passage / 2, wy - passage / 2,
                wx + passage / 2, wy + passage / 2, FREE)

    carve(0, 0)
    while stack:
        cx, cy = stack[-1]
        nbrs = []
        for dx, dy in ((1, 0), (-1, 0), (0, 1), (0, -1)):
            nx_, ny_ = cx + dx, cy + dy
            if 0 <= nx_ < ncols and 0 <= ny_ < nrows and not visited[ny_, nx_]:
                nbrs.append((nx_, ny_, dx, dy))
        if not nbrs:
            stack.pop()
            continue
        idx = int(rng.integers(0, len(nbrs)))
        nx_, ny_, dx, dy = nbrs[idx]
        visited[ny_, nx_] = True

        # 打通两格之间的墙
        wx = x0 + cx * cell + cell / 2.0 + dx * cell / 2.0
        wy = y0 + cy * cell + cell / 2.0 + dy * cell / 2.0
        if dx != 0:
            gm.rect(wx - cell / 2.0, wy - passage / 2, wx + cell / 2.0, wy + passage / 2, FREE)
        else:
            gm.rect(wx - passage / 2, wy - cell / 2.0, wx + passage / 2, wy + cell / 2.0, FREE)

        carve(nx_, ny_)
        stack.append((nx_, ny_))


def build_complex(res, wall_h):
    """复杂：迷宫(0.8m 窄通道) + 长直走廊(几何退化) + 空旷大厅(特征缺失) + 超薄隔板。"""
    gm = GridMap(36.0, 28.0, res, wall_h)
    gm.fill(OCC)

    # --- 左半：迷宫，通道 0.8m ---
    # 注意：0.8m 通道对 footprint 0.96m 的轮足是【不可通过】的，这是刻意的 ——
    # 检验导航规划器能否正确判定不可通行，而不是把机器人硬挤进墙里。
    # 对 0.6m 宽的小型机器人则可通过，同一张图对两类平台给出不同结论，正合适。
    _carve_maze(gm, -17.0, -13.0, -3.2, 13.0, cell=2.3, passage=0.8, seed=20260910)

    # --- 右上：14m x 1.4m 长直走廊 —— 几何退化场景 ---
    # 沿走廊轴向，激光几乎没有横向约束，LIO 会沿轴缓慢漂移。
    # 这是 3D LIO 最经典的失效场景，必须出现在测试集里。
    gm.rect(-3.2, 8.3, 13.0, 9.7, FREE)

    # --- 右下：9m x 9m 空旷大厅，无任何结构特征 —— 漂移测试 ---
    gm.rect(3.0, -13.0, 12.0, -4.0, FREE)

    # --- 竖向连廊：空旷大厅 <-> 长直走廊 ---
    gm.rect(2.0, -6.0, 4.0, 9.0, FREE)

    # --- 横向连廊：迷宫 <-> 竖向连廊（迷宫入口）---
    gm.rect(-8.0, -5.55, 3.4, -4.35, FREE)

    # --- 侧室：给超薄隔板留地方 ---
    gm.rect(-3.2, 1.0, 3.0, 7.0, FREE)

    # --- 超薄隔板（0.1m 厚）---
    # 模拟玻璃 / 薄板。真机上激光会穿透或产生双面墙，导致地图出现"重影"。
    # 仿真里它们是实体，所以能稳定复现"薄墙两侧点云错位"这类算法问题。
    gm.rect(-2.0, 2.5, 1.0, 2.6, OCC)
    gm.rect(-2.0, 5.0, 1.0, 5.1, OCC)

    spawn = gm.nearest_free(7.5, -8.5)
    meta = {
        "dynamic_obstacles": 4,
        "description": "迷宫(0.8m 通道) + 14m 长直走廊 + 9x9m 空旷大厅 + 0.1m 超薄隔板",
        "expected_failure_modes": [
            "长直走廊几何退化 → 沿走廊轴漂移",
            "空旷大厅特征缺失 → z 轴与 yaw 漂移",
            "0.8m 窄通道 → 规划器是否会正确判定不可通行",
            "超薄隔板 → 双面墙导致的地图重影",
        ],
        "landmarks": "dense",
    }
    return gm, spawn, meta


BUILDERS = {
    "simple": build_simple,
    "medium": build_medium,
    "complex": build_complex,
}


# ---------------------------------------------------------------------------
# 输出：SDF world
# ---------------------------------------------------------------------------

WORLD_TEMPLATE = """<?xml version="1.0" ?>
<sdf version="1.6">
  <world name="{name}">

    <physics name="default_physics" type="ode">
      <max_step_size>0.002</max_step_size>
      <real_time_factor>1.0</real_time_factor>
      <real_time_update_rate>500</real_time_update_rate>
    </physics>

    <scene>
      <ambient>0.45 0.45 0.45 1</ambient>
      <background>0.60 0.68 0.78 1</background>
      <shadows>true</shadows>
    </scene>

    <!-- 光源：室内场景给足散射光，避免相机深度图全黑 -->
    <light name="sun" type="directional">
      <cast_shadows>true</cast_shadows>
      <pose>0 0 20 0 0 0</pose>
      <diffuse>0.85 0.85 0.85 1</diffuse>
      <specular>0.2 0.2 0.2 1</specular>
      <attenuation>
        <range>100</range><constant>0.9</constant><linear>0.01</linear><quadratic>0.001</quadratic>
      </attenuation>
      <direction>-0.4 0.3 -0.9</direction>
    </light>
    <light name="fill" type="point">
      <pose>0 0 8 0 0 0</pose>
      <diffuse>0.5 0.5 0.5 1</diffuse>
      <specular>0.05 0.05 0.05 1</specular>
      <attenuation>
        <range>60</range><constant>0.4</constant><linear>0.02</linear><quadratic>0.002</quadratic>
      </attenuation>
    </light>

    <!-- 地面 -->
    <model name="ground_plane">
      <static>true</static>
      <link name="link">
        <collision name="collision">
          <geometry><plane><normal>0 0 1</normal><size>200 200</size></plane></geometry>
          <surface><friction><ode><mu>1.0</mu><mu2>1.0</mu2></ode></friction></surface>
        </collision>
        <visual name="visual">
          <geometry><plane><normal>0 0 1</normal><size>200 200</size></plane></geometry>
          <material>
            <script>
              <uri>file://media/materials/scripts/gazebo.material</uri>
              <name>Gazebo/Grey</name>
            </script>
          </material>
        </visual>
      </link>
    </model>
{walls}
{dynamics}
  </world>
</sdf>
"""

WALL_TEMPLATE = """    <model name="{name}">
      <static>true</static>
      <pose>{x:.4f} {y:.4f} {z:.4f} 0 0 0</pose>
      <link name="link">
        <collision name="collision">
          <geometry><box><size>{sx:.4f} {sy:.4f} {sz:.4f}</size></box></geometry>
        </collision>
        <visual name="visual">
          <geometry><box><size>{sx:.4f} {sy:.4f} {sz:.4f}</size></box></geometry>
          <material>
            <script>
              <uri>file://media/materials/scripts/gazebo.material</uri>
              <name>{mat}</name>
            </script>
          </material>
        </visual>
      </link>
    </model>
"""

DYN_TEMPLATE = """    <model name="dyn_obstacle_{idx}">
      <pose>{x:.4f} {y:.4f} {z:.4f} 0 0 0</pose>
      <link name="link">
        <inertial>
          <mass>3.0</mass>
          <inertia>
            <ixx>0.05</ixx><ixy>0</ixy><ixz>0</ixz>
            <iyy>0.05</iyy><iyz>0</iyz><izz>0.05</izz>
          </inertia>
        </inertial>
        <collision name="collision">
          <geometry><cylinder><radius>0.25</radius><length>0.8</length></cylinder></geometry>
          <surface><friction><ode><mu>0.6</mu><mu2>0.6</mu2></ode></friction></surface>
        </collision>
        <visual name="visual">
          <geometry><cylinder><radius>0.25</radius><length>0.8</length></cylinder></geometry>
          <material>
            <script>
              <uri>file://media/materials/scripts/gazebo.material</uri>
              <name>Gazebo/Red</name>
            </script>
          </material>
        </visual>
      </link>
    </model>
"""


def _pick_dyn_obstacles(gm, count, spawn, seed=42):
    """在可通行区域里随机撒动态障碍，且离出生点足够远。"""
    if count <= 0:
        return []
    rng = np.random.default_rng(seed)
    points = []
    tries = 0
    while len(points) < count and tries < 5000:
        tries += 1
        x = float(rng.uniform(gm.ox + 1.5, gm.ox + gm.width_m - 1.5))
        y = float(rng.uniform(gm.oy + 1.5, gm.oy + gm.height_m - 1.5))
        if not gm.is_free(x, y, pad=0.6):
            continue
        if (x - spawn[0]) ** 2 + (y - spawn[1]) ** 2 < 4.0:
            continue
        if any((x - px) ** 2 + (y - py) ** 2 < 2.5 for px, py in points):
            continue
        points.append((x, y))
    return points


def write_world(gm, out_path, name, dyn_points):
    mask = gm.grid == OCC
    rects = greedy_rects(mask)

    walls = []
    for i, (ix0, iy0, ix1, iy1) in enumerate(rects):
        sx = (ix1 - ix0 + 1) * gm.res
        sy = (iy1 - iy0 + 1) * gm.res
        cx = gm.ox + (ix0 + (ix1 - ix0 + 1) / 2.0) * gm.res
        cy = gm.oy + (iy0 + (iy1 - iy0 + 1) / 2.0) * gm.res
        mat = "Gazebo/Grey" if i % 2 else "Gazebo/DarkGrey"
        walls.append(
            WALL_TEMPLATE.format(
                name=f"wall_{i}", x=cx, y=cy, z=gm.wall_height / 2.0,
                sx=sx, sy=sy, sz=gm.wall_height, mat=mat,
            )
        )

    dyn = [
        DYN_TEMPLATE.format(idx=i, x=p[0], y=p[1], z=0.4)
        for i, p in enumerate(dyn_points)
    ]

    xml = WORLD_TEMPLATE.format(
        name=name, walls="\n".join(walls), dynamics="\n".join(dyn)
    )
    # 用 ElementTree 做一次格式合法性校验，早失败好过 Gazebo 里报错
    ET.fromstring(xml)

    # newline="\n" 是必须的：Windows 上默认会写 CRLF，而 world/yaml/脚本
    # 都要被 WSL 侧的 Gazebo 和 shell 读取。CRLF 会让 shell 脚本直接报
    # "没有那个文件或目录"，也会让某些解析器把 \r 当成数据。
    with open(out_path, "w", encoding="utf-8", newline="\n") as f:
        f.write(xml)

    return len(rects), len(dyn)


# ---------------------------------------------------------------------------
# 输出：GT 占据栅格
# ---------------------------------------------------------------------------


def write_occupancy_grid(gm, base_path):
    """写标准 map_server 格式：<name>.pgm + <name>.yaml

    注意 PGM 的行序：图像第 0 行对应 **最大 y**，所以要垂直翻转。
    这一点搞错的话，GT 和实际地图会上下镜像 —— 而且看起来"差不多"，
    非常难发现。这里显式处理并注释。
    """
    img = np.where(gm.grid == OCC, 0, 254).astype(np.uint8)  # 0=占据 254=空闲
    img = np.flipud(img)  # 行 0 -> 最大 y

    pgm_path = base_path + ".pgm"
    with open(pgm_path, "wb") as f:
        f.write(f"P5\n{gm.nx} {gm.ny}\n255\n".encode("ascii"))
        f.write(img.tobytes())

    yaml_path = base_path + ".yaml"
    with open(yaml_path, "w", encoding="utf-8", newline="\n") as f:
        f.write(
            f"image: {os.path.basename(pgm_path)}\n"
            f"resolution: {gm.res}\n"
            f"origin: [{gm.ox:.6f}, {gm.oy:.6f}, 0.0]\n"
            "negate: 0\n"
            "occupied_thresh: 0.65\n"
            "free_thresh: 0.196\n"
        )
    return pgm_path, yaml_path


# ---------------------------------------------------------------------------
# 主流程
# ---------------------------------------------------------------------------


def main():
    here = os.path.dirname(os.path.abspath(__file__))
    pkg_root = os.path.dirname(here)

    ap = argparse.ArgumentParser(description="生成三张测试地图（SDF + GT 栅格）")
    ap.add_argument("--maps", nargs="*", default=list(BUILDERS.keys()),
                    choices=list(BUILDERS.keys()), help="要生成哪些地图")
    ap.add_argument("--res", type=float, default=0.10, help="栅格分辨率 (m)")
    ap.add_argument("--wall-height", type=float, default=1.5, help="墙体高度 (m)")
    ap.add_argument("--out-worlds", default=os.path.join(pkg_root, "worlds"))
    ap.add_argument("--out-maps", default=os.path.join(pkg_root, "maps"))
    args = ap.parse_args()

    os.makedirs(args.out_worlds, exist_ok=True)
    os.makedirs(args.out_maps, exist_ok=True)

    summary = {}
    for name in args.maps:
        gm, spawn, meta = BUILDERS[name](args.res, args.wall_height)

        # 连通性自检：不连通的地图会在导航测试里伪装成"规划器故障"
        conn, _ = connectivity_report(gm, spawn)

        dyn_points = _pick_dyn_obstacles(
            gm, meta["dynamic_obstacles"], spawn, seed=hash(name) % 10000
        )

        n_walls, n_dyn = write_world(
            gm, os.path.join(args.out_worlds, f"{name}.world"), name, dyn_points
        )
        pgm, yml = write_occupancy_grid(gm, os.path.join(args.out_maps, name))

        meta.update(
            {
                "name": name,
                "resolution": args.res,
                "wall_height": args.wall_height,
                "width_m": gm.width_m,
                "height_m": gm.height_m,
                "grid_nx": gm.nx,
                "grid_ny": gm.ny,
                "origin": [gm.ox, gm.oy, 0.0],
                "wall_boxes": n_walls,
                "free_area_ratio": round(gm.free_area_ratio(), 4),
                "connectivity_from_spawn": round(conn, 4),
                "recommended_spawn": {
                    "x": round(float(spawn[0]), 3),
                    "y": round(float(spawn[1]), 3),
                    "yaw": 0.0,
                },
                "dynamic_obstacle_poses": [
                    {"x": round(p[0], 3), "y": round(p[1], 3)} for p in dyn_points
                ],
                "gt_map_pgm": os.path.basename(pgm),
                "gt_map_yaml": os.path.basename(yml),
            }
        )
        with open(os.path.join(args.out_maps, f"{name}_meta.json"), "w",
                  encoding="utf-8", newline="\n") as f:
            json.dump(meta, f, ensure_ascii=False, indent=2)

        summary[name] = meta

        warn = ""
        if conn < 0.999:
            warn = f"  ← 警告：{100*(1-conn):.1f}% 区域从出生点不可达（检查是否有隔墙贯通封死了房间）"
        print(
            f"[{name:8s}] {gm.width_m:.0f}x{gm.height_m:.0f}m  "
            f"可通行 {meta['free_area_ratio']*100:.1f}%  连通 {conn*100:.1f}%  "
            f"墙体 box {n_walls:3d}  动态障碍 {n_dyn}  "
            f"出生点 ({spawn[0]:.2f}, {spawn[1]:.2f}){warn}"
        )

    print(f"\n世界文件 -> {args.out_worlds}")
    print(f"GT 栅格  -> {args.out_maps}")
    print("\n提示：0.8m 窄通道对 footprint 0.96m 的轮足是【不可通过】的，这是刻意的，")
    print("      用来检验规划器能否正确判定不可通行，而不是硬挤。")


if __name__ == "__main__":
    main()
