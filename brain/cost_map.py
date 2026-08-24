"""B3 step 1 -- traversability cost map over the Mars terrain."""
from dataclasses import dataclass

import numpy as np
from scipy.ndimage import grey_opening, uniform_filter


@dataclass
class CostParams:
    slope_ref_deg: float = 12.0
    slope_max_deg: float = 25.0
    obs_ref_m: float = 0.5
    obs_max_m: float = 1.2
    w_slope: float = 1.0
    w_obs: float = 1.5
    inflate_cells: int = 2
    neighbourhood_m: float = 4.0


class TraversabilityMap:
    def __init__(self, height, cell, origin_xy, params: CostParams = None):
        """height: (n,n) metres indexed [ix, iy]; cell: metres/sample; origin_xy: world coords of index."""
        self.h = np.asarray(height, float)
        self.cell = float(cell)
        self.origin = np.asarray(origin_xy, float)
        self.p = params or CostParams()
        self.n = self.h.shape[0]

        gx, gy = np.gradient(self.h, self.cell)
        self.slope_deg = np.degrees(np.arctan(np.hypot(gx, gy)))

        w = max(3, int(round(self.p.neighbourhood_m / self.cell)) | 1)
        self.prominence = self.h - grey_opening(self.h, size=w)

        self.cost = (1.0
                     + self.p.w_slope * self.slope_deg / self.p.slope_ref_deg
                     + self.p.w_obs * self.prominence / self.p.obs_ref_m)

        blocked = ((self.slope_deg > self.p.slope_max_deg)
                   | (self.prominence > self.p.obs_max_m))
        # un-inflated: inflation is a planner margin, not a property of the ground
        self.blocked_raw = blocked.copy()
        if self.p.inflate_cells > 0:
            k = 2 * self.p.inflate_cells + 1
            blocked = uniform_filter(blocked.astype(float), size=k) > 1e-9
        self.blocked = blocked
        self.cost[self.blocked] = np.inf

    def to_grid(self, x, y):
        i = int(round((x - self.origin[0]) / self.cell))
        j = int(round((y - self.origin[1]) / self.cell))
        return (int(np.clip(i, 0, self.n - 1)), int(np.clip(j, 0, self.n - 1)))

    def to_world(self, i, j):
        return (self.origin[0] + i * self.cell, self.origin[1] + j * self.cell)

    def is_blocked(self, i, j):
        return bool(self.blocked[i, j])

    def clearance_m(self, x, y):
        """Distance from (x,y) to the nearest BLOCKED cell, in metres."""
        if not hasattr(self, "_dist"):
            from scipy.ndimage import distance_transform_edt
            self._dist = distance_transform_edt(~self.blocked) * self.cell
        i, j = self.to_grid(x, y)
        return float(self._dist[i, j])

    def summary(self):
        finite = np.isfinite(self.cost)
        return {
            "grid": f"{self.n}x{self.n}",
            "cell_m": round(self.cell, 3),
            "blocked_cells": int(self.blocked.sum()),
            "blocked_pct": round(100.0 * self.blocked.mean(), 2),
            "slope_deg_max": round(float(self.slope_deg.max()), 2),
            "prominence_m_max": round(float(self.prominence.max()), 2),
            "cost_min": round(float(self.cost[finite].min()), 3),
            "cost_max_finite": round(float(self.cost[finite].max()), 3),
        }
