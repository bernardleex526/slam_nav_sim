#!/usr/bin/env python3
"""traj_range.py —— 打印轨迹文件的范围与时间跨度。"""
import csv
import sys

for p in sys.argv[1:]:
    print("=" * 70)
    print(p)
    try:
        rows = list(csv.DictReader(open(p)))
    except Exception as e:
        print("  ERR:", e)
        continue
    if not rows:
        print("  empty")
        continue
    print("  cols:", list(rows[0].keys()), " n =", len(rows))
    for axis in ("x", "y", "z", "t"):
        for c in rows[0]:
            if c.strip().lower() == axis:
                try:
                    v = [float(r[c]) for r in rows if r.get(c) not in (None, "")]
                except ValueError:
                    continue
                if v:
                    print("  %-4s [%.3f, %.3f]  span=%.3f"
                          % (c, min(v), max(v), max(v) - min(v)))
                break
