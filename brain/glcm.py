"""Grey-level co-occurrence matrix features, from scratch, sliding-window."""
import numpy as np
from scipy.ndimage import uniform_filter

OFFSETS = ((0, 1), (1, 0), (1, 1), (1, -1))


def _shift(a, dy, dx, fill=0):
    out = np.full_like(a, fill)
    H, W = a.shape
    out[max(0, -dy):H + min(0, -dy), max(0, -dx):W + min(0, -dx)] = \
        a[max(0, dy):H + min(0, dy), max(0, dx):W + min(0, dx)]
    return out


def _boxsum(a, win):
    return uniform_filter(np.asarray(a, float), size=win, mode="constant") * win * win


def detrend(height, win):
    """Remove the local mean so GLCM measures roughness, not slope."""
    h = np.asarray(height, float)
    return h - uniform_filter(h, size=win, mode="nearest")


def quantize(resid, levels, half_range):
    """Residual heights -> integer levels over a FIXED physical range, so the feature is comparable."""
    z = np.clip(resid / half_range, -1.0, 1.0)
    q = np.floor((z + 1.0) / 2.0 * levels).astype(np.int32)
    return np.clip(q, 0, levels - 1)


def glcm_features(q, levels, win, valid=None, want_energy=True):
    """Sliding-window GLCM features."""
    q = np.asarray(q, np.int32)
    valid = np.ones(q.shape, bool) if valid is None else np.asarray(valid, bool)

    acc = {k: np.zeros(q.shape, float) for k in
           ("contrast", "homogeneity", "correlation", "energy")}
    noff = 0
    for dy, dx in OFFSETS:
        b = _shift(q, dy, dx)
        vb = _shift(valid.astype(np.uint8), dy, dx).astype(bool)
        vm = (valid & vb).astype(float)
        n = _boxsum(vm, win)
        ok = n > 4
        nn = np.where(ok, n, 1.0)

        a = q.astype(float)
        bb = b.astype(float)
        d = a - bb

        contrast = _boxsum(vm * d * d, win) / nn
        homog = _boxsum(vm / (1.0 + d * d), win) / nn

        mu_a = _boxsum(vm * a, win) / nn
        mu_b = _boxsum(vm * bb, win) / nn
        va = _boxsum(vm * a * a, win) / nn - mu_a ** 2
        vb_ = _boxsum(vm * bb * bb, win) / nn - mu_b ** 2
        cov = _boxsum(vm * a * bb, win) / nn - mu_a * mu_b
        denom = np.sqrt(np.clip(va, 0, None) * np.clip(vb_, 0, None))
        corr = np.where(denom > 1e-9, cov / np.where(denom > 1e-9, denom, 1), 0.0)

        energy = np.zeros(q.shape, float)
        if want_energy:
            idx = q * levels + b
            for k in range(levels * levels):
                p = _boxsum(vm * (idx == k), win) / nn
                energy += p * p

        for k, v in (("contrast", contrast), ("homogeneity", homog),
                     ("correlation", corr), ("energy", energy)):
            acc[k] += np.where(ok, v, np.nan)
        noff += 1

    return {k: v / noff for k, v in acc.items()}


def roughness(height, valid=None, levels=8, win=9, half_range=0.35):
    """Convenience: detrended, quantized, GLCM contrast as a roughness field."""
    resid = detrend(height, win)
    q = quantize(resid, levels, half_range)
    f = glcm_features(q, levels, win, valid=valid)
    return f["contrast"], f
