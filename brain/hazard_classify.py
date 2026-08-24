"""B4 step 2 -- per-patch hazard classification from a depth frame."""
from dataclasses import dataclass

import numpy as np
from scipy.ndimage import grey_opening, uniform_filter, binary_erosion

import glcm


@dataclass
class PerceptParams:
    fine_div: int = 4
    slope_max_deg: float = 25.0
    obs_max_m: float = 1.2
    neighbourhood_m: float = 4.0
    glcm_levels: int = 8
    glcm_window: int = 9
    # 0.05 m, measured: at 0.35 m the hazard/safe separation was +0.001 (zero information)
    glcm_half_range_m: float = 0.05
    rough_lo: float = 0.90
    rough_hi: float = 2.40
    w_slope: float = 1.0
    w_prom: float = 1.0
    w_rough: float = 0.6
    # 0.95 not 0.5: the score saturates, all discrimination is in the top few percent
    threshold: float = 0.95
    min_pts_per_fine_cell: int = 1


def bin_points(pts, cell, origin_xy, n, min_pts=1):
    """Scatter projected points into a world-aligned raster of mean height."""
    p = np.asarray(pts, float).reshape(-1, 3)
    p = p[np.isfinite(p).all(1)]
    i = np.round((p[:, 0] - origin_xy[0]) / cell).astype(int)
    j = np.round((p[:, 1] - origin_xy[1]) / cell).astype(int)
    m = (i >= 0) & (i < n) & (j >= 0) & (j < n)
    i, j, z = i[m], j[m], p[m, 2]
    flat = i * n + j
    cnt = np.bincount(flat, minlength=n * n).astype(float)
    tot = np.bincount(flat, weights=z, minlength=n * n)
    with np.errstate(invalid="ignore", divide="ignore"):
        h = np.where(cnt >= min_pts, tot / np.maximum(cnt, 1), np.nan)
    return h.reshape(n, n), cnt.reshape(n, n)


def _fill_small_gaps(h, valid, max_iter=2):
    """Nearest-neighbour fill of isolated holes so gradients are not computed across NaN."""
    h = h.copy()
    v = valid.copy()
    for _ in range(max_iter):
        if v.all():
            break
        hs = np.where(v, h, 0.0)
        num = uniform_filter(hs, size=3, mode="constant") * 9
        den = uniform_filter(v.astype(float), size=3, mode="constant") * 9
        fillable = (~v) & (den >= 3)
        if not fillable.any():
            break
        h[fillable] = num[fillable] / den[fillable]
        v = v | fillable
    return h, v


def score_frame(pts, cell, origin_xy, n, params: PerceptParams = None):
    """Projected points -> per-cell hazard score on the B1 grid."""
    p = params or PerceptParams()
    d = p.fine_div
    fine = cell / d
    empty = lambda: {"score": np.full((n, n), np.nan),
                     "observed": np.zeros((n, n), bool),
                     "slope_deg": np.full((n, n), np.nan),
                     "prominence_m": np.full((n, n), np.nan),
                     "roughness": np.full((n, n), np.nan)}

    hc, cc = bin_points(pts, cell, origin_xy, n, p.min_pts_per_fine_cell)
    obs_c = np.isfinite(hc)
    if obs_c.sum() < 25:
        return empty()
    hc_fill, v_c = _fill_small_gaps(hc, obs_c)
    work_c = np.where(v_c, hc_fill, np.nanmedian(hc_fill[v_c]))

    gx, gy = np.gradient(work_c, cell)
    slope_deg = np.degrees(np.arctan(np.hypot(gx, gy)))
    w = max(3, int(round(p.neighbourhood_m / cell)) | 1)
    prominence = work_c - grey_opening(work_c, size=w)

    # erode by the analysis window: a top-hat across the data edge invents prominence
    pad = w // 2 + 1
    scorable = binary_erosion(v_c, structure=np.ones((3, 3)),
                              iterations=pad, border_value=0)
    if scorable.sum() < 10:
        return empty()

    hf, cf = bin_points(pts, fine, origin_xy, n * d, p.min_pts_per_fine_cell)
    obs_f = np.isfinite(hf)
    hf_fill, v_f = _fill_small_gaps(hf, obs_f)
    work_f = np.where(v_f, hf_fill, np.nanmedian(hf_fill[v_f]))
    contrast, _ = glcm.roughness(work_f, valid=v_f, levels=p.glcm_levels,
                                 win=p.glcm_window,
                                 half_range=p.glcm_half_range_m)
    with np.errstate(invalid="ignore"):
        rough_c = np.nanmax(
            np.where(v_f, contrast, np.nan)[:n*d, :n*d].reshape(n, d, n, d),
            axis=(1, 3))

    s_slope = np.clip(slope_deg / p.slope_max_deg, 0, 1)
    s_prom = np.clip(prominence / p.obs_max_m, 0, 1)
    s_rough = np.nan_to_num(
        np.clip((rough_c - p.rough_lo) / (p.rough_hi - p.rough_lo), 0, 1), nan=0.0)

    fused = 1.0 - ((1 - s_slope) ** p.w_slope
                   * (1 - s_prom) ** p.w_prom
                   * (1 - s_rough) ** p.w_rough)

    return {
        "score": np.where(scorable, fused, np.nan),
        "observed": scorable,
        "height": np.where(scorable, work_c, np.nan),
        "slope_deg": np.where(scorable, slope_deg, np.nan),
        "prominence_m": np.where(scorable, prominence, np.nan),
        "roughness": np.where(scorable, rough_c, np.nan),
    }


def confusion(score, truth, observed, threshold):
    """Confusion counts over OBSERVED cells only, plus the two rates that matter separately: miss r."""
    m = observed & np.isfinite(score)
    pred = score[m] >= threshold
    tru = truth[m].astype(bool)
    tp = int((pred & tru).sum()); fp = int((pred & ~tru).sum())
    fn = int((~pred & tru).sum()); tn = int((~pred & ~tru).sum())
    prec = tp / max(tp + fp, 1)
    rec = tp / max(tp + fn, 1)
    return {
        "tp": tp, "fp": fp, "fn": fn, "tn": tn,
        "precision": prec, "recall": rec,
        "f1": 2 * prec * rec / max(prec + rec, 1e-9),
        "miss_rate": fn / max(tp + fn, 1),          # real hazard called safe
        "false_alarm_rate": fp / max(fp + tn, 1),   # safe ground called hazard
        "iou": tp / max(tp + fp + fn, 1),
        "n_observed": int(m.sum()), "n_true_hazard": int(tru.sum()),
    }
