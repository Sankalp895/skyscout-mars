"""B4 step 3 -- the persistent world hazard map the drone builds and hands on."""
import numpy as np

from cost_map import TraversabilityMap, CostParams
from nav_params import MapParams


class PerceivedMap:
    def __init__(self, n, cell, origin_xy, sensor: MapParams = None,
                 p_lo=0.05, p_hi=0.95, l_max=None):
        self.n, self.cell = int(n), float(cell)
        self.origin = np.asarray(origin_xy, float)
        self.sensor = sensor or MapParams()
        self.p_lo, self.p_hi = p_lo, p_hi
        self.l_max = self.sensor.l_total_clamp if l_max is None else l_max

        self.logodds = np.zeros((n, n), float)
        self.count = np.zeros((n, n), int)
        self._h_sum = np.zeros((n, n), float)
        self._h_cnt = np.zeros((n, n), int)

    def fuse(self, score, observed, height=None):
        """Fold one frame's per-cell score into the map.

        Evidence per look comes from the MEASURED sensor model, NOT the raw
        score. The score is a fused severity whose decision point is 0.95;
        feeding it to a logit drove the false-alarm rate to 0.550 vs 0.023.
        """
        m = observed & np.isfinite(score)
        if not m.any():
            return 0
        mp = self.sensor
        l_hit = np.clip(np.log(mp.p_detect_given_hazard / mp.p_detect_given_safe),
                        -mp.l_per_obs_clamp, mp.l_per_obs_clamp)
        l_miss = np.clip(np.log((1 - mp.p_detect_given_hazard)
                                / (1 - mp.p_detect_given_safe)),
                         -mp.l_per_obs_clamp, mp.l_per_obs_clamp)
        detect = score[m] >= mp.detect_threshold
        self.logodds[m] += np.where(detect, l_hit, l_miss)
        np.clip(self.logodds, -self.l_max, self.l_max, out=self.logodds)
        self.count[m] += 1
        if height is not None:
            hm = m & np.isfinite(height)
            self._h_sum[hm] += height[hm]
            self._h_cnt[hm] += 1
        return int(m.sum())

    @property
    def known(self):
        return self.count > 0

    @property
    def prob(self):
        """P(hazard). NaN where unknown -- never 0.0, which would read safe."""
        p = 1.0 / (1.0 + np.exp(-self.logodds))
        return np.where(self.known, p, np.nan)

    def hazard(self, threshold=0.5):
        """Boolean hazard over KNOWN cells only."""
        return self.known & (self.prob >= threshold)

    @property
    def height(self):
        with np.errstate(invalid="ignore", divide="ignore"):
            h = np.where(self._h_cnt > 0, self._h_sum / np.maximum(self._h_cnt, 1),
                         np.nan)
        return h

    def height_filled(self):
        """Perceived height with unknown cells filled by the observed median."""
        h = self.height
        v = np.isfinite(h)
        if not v.any():
            return np.zeros((self.n, self.n))
        return np.where(v, h, np.median(h[v]))

    def to_traversability(self, params: CostParams = None, threshold=0.5,
                          unknown_blocks=True):
        """Perceived map -> TraversabilityMap for A*.

        unknown is blocked. It is a third state and must never read as safe,
        or the planner routes through unmapped ground.
        """
        p = params or CostParams()
        tm = TraversabilityMap(self.height_filled(), self.cell, self.origin, p)
        blocked = self.hazard(threshold)
        if unknown_blocks:
            blocked = blocked | (~self.known)
        tm.blocked_raw = blocked.copy()
        if p.inflate_cells > 0:
            from scipy.ndimage import uniform_filter
            k = 2 * p.inflate_cells + 1
            blocked = uniform_filter(blocked.astype(float), size=k) > 1e-9
        tm.blocked = blocked
        tm.cost = np.where(blocked, np.inf, tm.cost)
        if hasattr(tm, "_dist"):
            del tm._dist
        return tm

    def summary(self):
        k = self.known
        return {
            "known_cells": int(k.sum()),
            "known_pct": round(100.0 * k.mean(), 2),
            "hazard_cells": int(self.hazard().sum()),
            "mean_obs_per_known_cell": round(float(self.count[k].mean()), 2)
                                       if k.any() else 0.0,
            "max_obs": int(self.count.max()),
        }


def boundary_displacement(perceived_hazard, true_hazard, known, cell):
    """How far perceived hazard EDGES sit from true hazard edges, in metres."""
    from scipy.ndimage import distance_transform_edt
    out = {}
    if true_hazard.any():
        d_to_true = distance_transform_edt(~true_hazard) * cell
        m = perceived_hazard & known
        out["perceived_to_true_mean_m"] = float(d_to_true[m].mean()) if m.any() else 0.0
        out["perceived_to_true_p90_m"] = float(np.percentile(d_to_true[m], 90)) if m.any() else 0.0
    if perceived_hazard.any():
        d_to_perc = distance_transform_edt(~perceived_hazard) * cell
        m = true_hazard & known
        out["true_to_perceived_mean_m"] = float(d_to_perc[m].mean()) if m.any() else 0.0
        out["true_to_perceived_p90_m"] = float(np.percentile(d_to_perc[m], 90)) if m.any() else 0.0
    return out


def load_perceived(path="b4_perceived_map.npz", sensor: MapParams = None):
    """Rehydrate a PerceivedMap from the saved deliverable."""
    z = np.load(path, allow_pickle=False)
    n = int(z["logodds"].shape[0])
    pm = PerceivedMap(n, float(z["cell"]), tuple(z["origin"]), sensor=sensor)
    pm.logodds = z["logodds"].copy()
    pm.count = z["count"].copy()
    h = z["height"]
    pm._h_sum = np.where(np.isfinite(h), h, 0.0)
    pm._h_cnt = (pm.count > 0).astype(int) * np.isfinite(h)
    return pm, z
