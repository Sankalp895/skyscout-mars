"""Why does the classifier flag half of all safe ground, and why is GLCM inert? Hypothesis 1 (o."""
import numpy as np

import camera_rig
from camera_rig import DownwardCamera
from skyscout_sim import SkyScoutSim
from mars_world import build_heightmap, EXTENT, CELL
from cost_map import TraversabilityMap, CostParams
from hazard_classify import bin_points, _fill_small_gaps
import glcm
from scipy.ndimage import grey_opening

RES, FOV, ALT = (320, 240), 60.0, 12.0
ORIGIN = (-EXTENT/2, -EXTENT/2)

hmap, _, rocks = build_heightmap()
tm = TraversabilityMap(hmap, CELL, ORIGIN, CostParams())
n = tm.n

sim = SkyScoutSim(start_pos=(12.7, 14.2, ALT), logging_level="error",
                  camera_res=RES, camera_fov=FOV)
rig = DownwardCamera(sim, RES, FOV)
rig.warm_up()

print("=== H1: does sampling scale explain the slope inflation? ===")
print("  computing sensed slope at several raster resolutions, over the same")
print("  frame, and comparing to the TRUE slope on the 0.63 m cost-map grid\n")
print(f"  {'raster':>10} {'div':>4} {'sensed slope':>26} {'true slope (same cells)':>26}")
print(f"  {'':10} {'':4} {'mean':>8}{'p90':>9}{'max':>9} {'mean':>8}{'p90':>9}{'max':>9}")

pos = np.array([12.7, 14.2, ALT])
sim.drone.set_pos(pos)
depth, _, cp, la = rig.capture(pos)
pts = rig.project(depth, pos)

for div in (1, 2, 4, 8):
    fine = CELL / div
    nf = n * div
    hf, cf = bin_points(pts, fine, ORIGIN, nf, 1)
    obs = np.isfinite(hf)
    hfill, vfill = _fill_small_gaps(hf, obs)
    work = np.where(vfill, hfill, np.nanmedian(hfill[vfill]))
    gx, gy = np.gradient(work, fine)
    slope = np.degrees(np.arctan(np.hypot(gx, gy)))
    s = slope[vfill]
    coarse_obs = vfill[:n*div, :n*div].reshape(n, div, n, div).sum(axis=(1,3)) >= (div*div)//4
    t = tm.slope_deg[coarse_obs]
    print(f"  {fine:9.3f}m {div:4d} {s.mean():8.2f}{np.percentile(s,90):9.2f}"
          f"{s.max():9.2f} {t.mean():8.2f}{np.percentile(t,90):9.2f}{t.max():9.2f}")

print("\n  -> if sensed mean tracks true mean only at div=1, the fine-raster")
print("     slope was the over-flagging cause.")

print("\n=== H2: what does GLCM contrast actually range over? ===")
for div in (4,):
    fine = CELL / div
    hf, cf = bin_points(pts, fine, ORIGIN, n*div, 1)
    obs = np.isfinite(hf)
    hfill, vfill = _fill_small_gaps(hf, obs)
    work = np.where(vfill, hfill, np.nanmedian(hfill[vfill]))
    for hr in (0.05, 0.10, 0.20, 0.35):
        contrast, feats = glcm.roughness(work, valid=vfill, levels=8, win=9,
                                         half_range=hr)
        c = contrast[vfill & np.isfinite(contrast)]
        coarse_obs = vfill[:n*div,:n*div].reshape(n,div,n,div).sum(axis=(1,3)) >= (div*div)//4
        cc = np.full((n, n), np.nan)
        b = contrast[:n*div,:n*div].reshape(n,div,n,div)
        with np.errstate(invalid="ignore"):
            cc = np.nanmax(b, axis=(1,3))
        hz = coarse_obs & tm.blocked_raw
        sf = coarse_obs & ~tm.blocked_raw
        print(f"  half_range {hr:.2f} m: contrast p5 {np.nanpercentile(c,5):.3f} "
              f"p50 {np.nanpercentile(c,50):.3f} p95 {np.nanpercentile(c,95):.3f} "
              f"max {np.nanmax(c):.3f}")
        print(f"      on TRUE HAZARD cells  mean {np.nanmean(cc[hz]):.3f}")
        print(f"      on TRUE SAFE   cells  mean {np.nanmean(cc[sf]):.3f}"
              f"   separation {np.nanmean(cc[hz])-np.nanmean(cc[sf]):+.3f}")

print("\n=== H1b: prominence scale check ===")
for div in (1, 4):
    fine = CELL/div
    hf, cf = bin_points(pts, fine, ORIGIN, n*div, 1)
    obs = np.isfinite(hf)
    hfill, vfill = _fill_small_gaps(hf, obs)
    work = np.where(vfill, hfill, np.nanmedian(hfill[vfill]))
    w = max(3, int(round(4.0/fine)) | 1)
    prom = work - grey_opening(work, size=w)
    coarse_obs = vfill[:n*div,:n*div].reshape(n,div,n,div).sum(axis=(1,3)) >= (div*div)//4
    print(f"  div {div} (cell {fine:.3f} m, window {w}): sensed prom mean "
          f"{prom[vfill].mean():.3f} p90 {np.percentile(prom[vfill],90):.3f}   "
          f"true prom mean {tm.prominence[coarse_obs].mean():.3f} "
          f"p90 {np.percentile(tm.prominence[coarse_obs],90):.3f}")
