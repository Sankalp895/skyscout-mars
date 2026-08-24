"""B3 step 2 -- A* over the traversability grid."""
import heapq

import numpy as np

NEIGHBOURS = [(-1, 0, 1.0), (1, 0, 1.0), (0, -1, 1.0), (0, 1, 1.0),
              (-1, -1, np.sqrt(2)), (-1, 1, np.sqrt(2)),
              (1, -1, np.sqrt(2)), (1, 1, np.sqrt(2))]


def plan(tmap, start_xy, goal_xy):
    """Least-cost path from start to goal. Returns dict or None if unreachable."""
    start = tmap.to_grid(*start_xy)
    goal = tmap.to_grid(*goal_xy)
    cost, blocked, cell, n = tmap.cost, tmap.blocked, tmap.cell, tmap.n

    if blocked[start]:
        return {"ok": False, "reason": "start cell is blocked"}
    if blocked[goal]:
        return {"ok": False, "reason": "goal cell is blocked"}

    min_cost = float(cost[np.isfinite(cost)].min())

    def h(ij):
        return np.hypot(ij[0] - goal[0], ij[1] - goal[1]) * cell * min_cost

    g = {start: 0.0}
    parent = {}
    open_heap = [(h(start), 0.0, start)]
    closed = np.zeros((n, n), dtype=bool)
    expanded = 0

    while open_heap:
        f, gc, cur = heapq.heappop(open_heap)
        if closed[cur]:
            continue
        closed[cur] = True
        expanded += 1
        if cur == goal:
            break
        ci, cj = cur
        for di, dj, stepc in NEIGHBOURS:
            ni, nj = ci + di, cj + dj
            if not (0 <= ni < n and 0 <= nj < n) or blocked[ni, nj] or closed[ni, nj]:
                continue
            ng = gc + stepc * cell * cost[ni, nj]
            if ng < g.get((ni, nj), np.inf):
                g[(ni, nj)] = ng
                parent[(ni, nj)] = cur
                heapq.heappush(open_heap, (ng + h((ni, nj)), ng, (ni, nj)))

    if goal not in g:
        return {"ok": False, "reason": "no path exists (goal unreachable)",
                "expanded": expanded}

    cells = [goal]
    while cells[-1] != start:
        cells.append(parent[cells[-1]])
    cells.reverse()

    world = np.array([tmap.to_world(i, j) for i, j in cells])
    seg = np.linalg.norm(np.diff(world, axis=0), axis=1)
    straight = float(np.linalg.norm(np.asarray(goal_xy) - np.asarray(start_xy)))
    length = float(seg.sum())
    return {
        "ok": True, "cells": cells, "world": world,
        "length_m": length, "straight_m": straight,
        "detour_ratio": length / max(straight, 1e-9),
        "total_cost": float(g[goal]), "expanded": expanded,
        "min_clearance_m": float(min(tmap.clearance_m(x, y) for x, y in world)),
    }


def straight_line_blocked(tmap, start_xy, goal_xy, samples=400):
    """Does the naive straight line cross a blocked cell? Used to pick test cases where the planner."""
    a, b = np.asarray(start_xy, float), np.asarray(goal_xy, float)
    hits = 0
    for t in np.linspace(0, 1, samples):
        x, y = a + t * (b - a)
        if tmap.is_blocked(*tmap.to_grid(x, y)):
            hits += 1
    return hits, hits / samples
