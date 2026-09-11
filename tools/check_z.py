#!/usr/bin/env python3
"""检查真值轨迹的 z（高度）是否稳定，用于判断 spawn 高度是否正确。"""
import csv
import sys


def main(path):
    rows = list(csv.DictReader(open(path)))
    if not rows:
        print("empty")
        return
    cols = list(rows[0].keys())
    print("cols:", cols)
    print("n =", len(rows))
    for cand in ("z", "pos_z", "pz", "tz"):
        for c in cols:
            if c.strip().lower() == cand:
                vals = [float(r[c]) for r in rows if r.get(c) not in (None, "")]
                if not vals:
                    continue
                print("%-8s n=%-5d min=%.4f max=%.4f first=%.4f last=%.4f  span=%.4f"
                      % (c, len(vals), min(vals), max(vals), vals[0], vals[-1],
                         max(vals) - min(vals)))
                break


if __name__ == "__main__":
    main(sys.argv[1])
