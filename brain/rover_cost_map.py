"""B5 part A -- the ROVER's traversability, which is not the drone's."""
from dataclasses import dataclass

import numpy as np
from scipy.ndimage import uniform_filter

from cost_map import TraversabilityMap


@dataclass
class RoverParams:
    wheel_radius_m: float = 0.29
    max_slope_deg: float = 20.0
    slope_ref_deg: float = 8.0
    step_ref_m: float = 0.10
    w_slope: float = 1.0
    w_step: float = 2.0
    inflate_cells: int = 1
    unknown_blocks: bool = True

    @property
    def step_limit_m(self):
        return self.wheel_radius_m


def local_step(h, valid):
    """Largest height difference to any 8-neighbour, in metres.

    Not the drone's top-hat: a 4 m window answers "is there a boulder near",
    while a wheel meets whatever is under it. The rover limit is one wheel
    radius (0.29 m) against the drone's 1.2 m prominence.
    """
    H = np.where(valid, h, np.nan)
    out = np.zeros_like(H)
    for dy in (-1, 0, 1):
        for dx in (-1, 0, 1):
            if dx == 0 and dy == 0:
                continue
            sh = np.full_like(H, np.nan)
            ys, xs = H.shape
            sh[max(0, -dy):ys + min(0, -dy), max(0, -dx):xs + min(0, -dx)] = \
                H[max(0, dy):ys + min(0, dy), max(0, dx):xs + min(0, dx)]
            d = np.abs(H - sh)
            out = np.fmax(out, np.nan_to_num(d, nan=0.0))
    return out


def rover_cost_map(pmap, params: RoverParams = None, drone_threshold=0.5):
    """Build the rover's traversability from the DRONE's perceived map."""
    p = params or RoverParams()
    cell = pmap.cell
    known = pmap.known
    h = pmap.height_filled()

    gx, gy = np.gradient(h, cell)
    slope = np.degrees(np.arctan(np.hypot(gx, gy)))
    step = local_step(h, known)
    drone_hz = pmap.hazard(drone_threshold)

    too_steep = slope > p.max_slope_deg
    too_stepped = step > p.step_limit_m
    unknown = ~known

    blocked_raw = too_steep | too_stepped | drone_hz
    if p.unknown_blocks:
        blocked_raw = blocked_raw | unknown

    cost = (1.0
            + p.w_slope * slope / p.slope_ref_deg
            + p.w_step * step / p.step_ref_m)

    tm = TraversabilityMap(h, cell, pmap.origin)
    tm.slope_deg = slope
    tm.prominence = step
    tm.blocked_raw = blocked_raw.copy()
    blocked = blocked_raw
    if p.inflate_cells > 0:
        k = 2 * p.inflate_cells + 1
        blocked = uniform_filter(blocked.astype(float), size=k) > 1e-9
    tm.blocked = blocked
    tm.cost = np.where(blocked, np.inf, cost)
    if hasattr(tm, "_dist"):
        del tm._dist

    diag = {"too_steep": too_steep, "too_stepped": too_stepped,
            "unknown": unknown, "drone_hazard": drone_hz,
            "slope": slope, "step": step, "params": p}
    return tm, diag
