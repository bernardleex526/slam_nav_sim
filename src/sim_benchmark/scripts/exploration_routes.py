#!/usr/bin/env python3
"""Deterministic GT-assisted coverage, with SciPy shortest-path connectors."""
import math
from pathlib import Path

import numpy as np
from scipy.ndimage import distance_transform_edt, label
from scipy.sparse import coo_matrix
from scipy.sparse.csgraph import dijkstra
import yaml


def coverage_cells(safe, start, lane_step):
    if lane_step < 1 or not safe[start]:
        raise ValueError("Invalid lane spacing or unsafe spawn")
    components, _ = label(safe)
    reachable = components == components[start]
    h, w = safe.shape
    ids = np.arange(h * w).reshape(h, w)
    edges = []
    for left, right, mask in (
            (ids[:, :-1], ids[:, 1:], reachable[:, :-1] & reachable[:, 1:]),
            (ids[:-1], ids[1:], reachable[:-1] & reachable[1:])):
        edges.append((left[mask], right[mask]))
    src = np.concatenate([x for x, y in edges] + [y for x, y in edges])
    dst = np.concatenate([y for x, y in edges] + [x for x, y in edges])
    graph = coo_matrix((np.ones(len(src)), (src, dst)), shape=(h*w, h*w)).tocsr()
    occupied_rows = np.flatnonzero(reachable.any(axis=1))
    lanes = list(range(int(occupied_rows[0]), int(occupied_rows[-1]) + 1, lane_step))
    if lanes[-1] != occupied_rows[-1]:
        lanes.append(int(occupied_rows[-1]))
    segments = []
    for y in lanes:
        xs = np.flatnonzero(reachable[y])
        for group in np.split(xs, np.flatnonzero(np.diff(xs) > 1) + 1):
            if len(group):
                segments.append(((y, int(group[0])), (y, int(group[-1]))))
    current = tuple(start)
    route = [current]
    while segments:
        index, reverse = min(
            ((i, rev) for i in range(len(segments)) for rev in (0, 1)),
            key=lambda item: (sum((a-b)**2 for a, b in zip(
                segments[item[0]][item[1]], current)), item))
        segment = segments.pop(index)
        entry, exit = segment[reverse], segment[1-reverse]
        _, previous = dijkstra(graph, directed=False, indices=ids[current], return_predecessors=True)
        cell = int(ids[entry])
        connector = [entry]
        while cell != ids[current]:
            cell = int(previous[cell])
            if cell < 0:
                raise RuntimeError("Disconnected connector in reachable component")
            connector.append(divmod(cell, w))
        route.extend(simplify_route(list(reversed(connector)), safe)[1:])
        if exit != entry:
            step = 1 if exit[1] > entry[1] else -1
            route.extend((entry[0], x) for x in range(entry[1]+step, exit[1]+step, step))
        current = exit
    # Keep every corner; never shortcut through an occupied cell.
    compact = [route[0]]
    for index in range(1, len(route)-1):
        before = np.subtract(route[index], route[index-1])
        after = np.subtract(route[index+1], route[index])
        if not np.array_equal(before, after):
            compact.append(route[index])
    if route[-1] != compact[-1]:
        compact.append(route[-1])
    return compact, int(reachable.sum())


def make_safe_path(gt_yaml, start, lane_spacing, clearance, read_pgm):
    path = Path(gt_yaml)
    meta = yaml.safe_load(path.read_text(encoding="utf-8"))
    if abs(float(meta["origin"][2])) > 1e-8:
        raise ValueError("Rotated maps are not supported by coverage planning")
    image = read_pgm(path.parent / meta["image"])
    resolution = float(meta["resolution"])
    probability = image / 255.0 if meta.get("negate", 0) else (255.0-image) / 255.0
    free = probability < float(meta.get("free_thresh", .196))
    # Include cell extent and tracking tolerance in the obstacle clearance.
    dist = distance_transform_edt(np.pad(free, 1))[1:-1, 1:-1] * resolution
    safe = free & (dist >= clearance + .1 + resolution / 2)
    ox, oy = meta["origin"][:2]
    h, w = image.shape
    cell = (h-1-int(math.floor((start[1]-oy)/resolution)),
            int(math.floor((start[0]-ox)/resolution)))
    if not (0 <= cell[0] < h and 0 <= cell[1] < w):
        raise ValueError("Spawn lies outside the map")
    cells, reachable = coverage_cells(safe, cell, max(1, int(round(lane_spacing/resolution))))
    points = [(ox + (x+.5)*resolution, oy + (h-y-.5)*resolution, 0.) for y, x in cells]
    return points, dict(safe_cells=int(safe.sum()), reachable_cells=reachable,
                        unreachable_safe_cells=int(safe.sum())-reachable,
                        methodology="GT-assisted coverage; not autonomous exploration")


def segment_safe(start, end, safe):
    count = max(2, int(math.ceil(math.dist(start, end)*2)) + 1)
    samples = np.linspace(start, end, count)
    lower, upper = np.floor(samples).astype(int), np.ceil(samples).astype(int)
    return all(np.all(safe[y, x]) for y in (lower[:, 0], upper[:, 0])
               for x in (lower[:, 1], upper[:, 1]))


def simplify_route(cells, safe):
    """Remove staircase corners only when the entire connector is safe."""
    if len(cells) < 3:
        return cells
    result = [cells[0]]
    index = 0
    while index < len(cells)-1:
        end = index+1
        while end+1 < len(cells) and segment_safe(cells[index], cells[end+1], safe):
            end += 1
        result.append(cells[end])
        index = end
    return result
