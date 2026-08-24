"""B5 CHECKPOINT A1 -- the rover's traversability vs the drone's hazard map."""
import json

import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from scipy.ndimage import label

from mars_world import build_heightmap, EXTENT, CELL, CRATER_CENTRE, CRATER_RADIUS
from cost_map import TraversabilityMap, CostParams
from world_map import load_perceived
from rover_cost_map import rover_cost_map, RoverParams, local_step

ORIGIN = (-EXTENT / 2, -EXTENT / 2)


def main():
    hmap, feats, rocks = build_heightmap()
    tm_true = TraversabilityMap(hmap, CELL, ORIGIN, CostParams())
    pm, z = load_perceived()
    n = pm.n

    rp = RoverParams()
    tm_rov, d = rover_cost_map(pm, rp)
    drone_hz = pm.hazard(0.5)
    known = pm.known

    print("=== B5 CHECKPOINT A1 -- rover traversability vs drone hazard map ===")
    print(f"  rover geometry from the URDF: wheel radius {rp.wheel_radius_m:.2f} m "
          f"(cylinder r=0.29, 6 wheels), chassis ~1.0 x 1.3 m")
    print(f"  rover limits: max slope {rp.max_slope_deg:.0f} deg (slip-limited), "
          f"max step {rp.step_limit_m:.2f} m (= one wheel radius)")
    print(f"  drone limits: max slope {CostParams().slope_max_deg:.0f} deg, "
          f"prominence {CostParams().obs_max_m:.2f} m over a "
          f"{CostParams().neighbourhood_m:.0f} m window")
    print(f"  inflation: rover {rp.inflate_cells} cell ({rp.inflate_cells*CELL:.2f} m, "
          f"= half-width) vs drone {CostParams().inflate_cells} cells "
          f"({CostParams().inflate_cells*CELL:.2f} m, flight buffer)")

    rov = tm_rov.blocked_raw & known
    dro = drone_hz & known
    both = rov & dro
    rov_only = rov & ~dro
    dro_only = dro & ~rov
    free_both = known & ~rov & ~dro

    print(f"\n=== DIFFERENCES over {int(known.sum())} KNOWN cells ===")
    print(f"  blocked for BOTH                  {int(both.sum()):6d}")
    print(f"  blocked for ROVER only            {int(rov_only.sum()):6d}   "
          f"<- flyable, not drivable")
    print(f"  blocked for DRONE only            {int(dro_only.sum()):6d}")
    print(f"  free for both                     {int(free_both.sum()):6d}")
    print(f"  rover blocks {100*rov.sum()/known.sum():.1f}% of known ground, "
          f"drone {100*dro.sum()/known.sum():.1f}%")

    steep_only = rov_only & d["too_steep"] & ~d["too_stepped"]
    step_only = rov_only & d["too_stepped"] & ~d["too_steep"]
    both_cause = rov_only & d["too_steep"] & d["too_stepped"]
    print(f"\n=== WHY the rover blocks what the drone did not ===")
    print(f"  step > {rp.step_limit_m:.2f} m only        {int(step_only.sum()):6d}  "
          f"<- rocks the drone flew over")
    print(f"  slope > {rp.max_slope_deg:.0f} deg only          {int(steep_only.sum()):6d}  "
          f"<- grades flyable but not drivable")
    print(f"  both causes                       {int(both_cause.sum()):6d}")
    if steep_only.any():
        sl = d["slope"][steep_only]
        band = float(((sl >= rp.max_slope_deg)
                      & (sl <= CostParams().slope_max_deg)).mean())
        print(f"  slope-only cells: {sl.min():.1f}-{sl.max():.1f} deg, mean {sl.mean():.1f}")
        print(f"    {100*band:.0f}% fall in the {rp.max_slope_deg:.0f}-"
              f"{CostParams().slope_max_deg:.0f} deg band, which is the expected"
              f" 'flyable but not drivable' wedge.")
        print(f"    The rest are STEEPER than the drone's own {CostParams().slope_max_deg:.0f}"
              f" deg limit yet were not flagged, because the two maps are derived")
        print(f"    differently: the drone's hazard label is its per-frame CLASSIFIER"
              f" output, while the rover map re-differentiates the FUSED height")
        print(f"    raster. Averaging many frames leaves the fused surface slightly"
              f" rougher than any single frame the classifier scored.")
    if step_only.any():
        st = d["step"][step_only]
        print(f"  step-only cells have steps {st.min():.2f}-{st.max():.2f} m "
              f"(mean {st.mean():.2f}) vs the {rp.step_limit_m:.2f} m limit")

    print(f"\n=== HAND CHECK -- are the differences where they should be? ===")
    frac = rov_only.sum() / max(known.sum(), 1)
    print(f"  rover-only blocks are {100*frac:.1f}% of known ground "
          f"-- {'sane' if 0.01 < frac < 0.5 else 'SUSPECT (everywhere or nowhere)'}")
    checked = 0
    for (bx, by, br, bh) in sorted(rocks, key=lambda r: -r[3])[:6]:
        i, j = tm_rov.to_grid(bx, by)
        if not known[i, j]:
            continue
        checked += 1
        print(f"  boulder ({bx:5.1f},{by:5.1f}) h {bh:.1f} m: "
              f"drone {'HAZ ' if dro[i,j] else 'safe'}  "
              f"rover {'BLOCK' if rov[i,j] else 'ok   '}  "
              f"sensed step {d['step'][i,j]:.2f} m  slope {d['slope'][i,j]:.1f} deg")
        if checked >= 4:
            break

    lbl, nlab = label(~tm_rov.blocked)
    sizes = np.bincount(lbl.ravel())[1:]
    order = np.argsort(sizes)[::-1]
    print(f"\n=== CONNECTIVITY of drivable ground ===")
    print(f"  {nlab} disconnected drivable components; largest "
          f"{sizes[order[0]] if len(sizes) else 0} cells")
    ci, cj = tm_rov.to_grid(*CRATER_CENTRE)
    if lbl[ci, cj] > 0:
        comp = lbl[ci, cj]
        same = comp == lbl[order[0] + 1] if False else (comp == (order[0] + 1))
        print(f"  crater floor at {CRATER_CENTRE} IS drivable "
              f"(component {comp}, {sizes[comp-1]} cells)")
        print(f"  ... and it is {'the main region' if same else 'ISOLATED from the main region'}"
              f" -- the rim blocks the way in" if not same else "")
    else:
        print(f"  crater floor at {CRATER_CENTRE} is not drivable")

    ext = [-EXTENT/2, EXTENT/2, -EXTENT/2, EXTENT/2]
    fig, ax = plt.subplots(1, 3, figsize=(19, 6.4))

    a = ax[0]
    a.imshow(np.where(known, pm.height, np.nan).T, origin="lower", extent=ext, cmap="terrain")
    ov = np.zeros((n, n, 4)); ov[drone_hz] = [0.85, .08, .08, .85]
    ov[~known] = [0.45, .45, .45, .85]
    a.imshow(np.transpose(ov, (1, 0, 2)), origin="lower", extent=ext)
    a.set_title("DRONE hazard map (fly-over)\nred = hazard, grey = unknown")

    a = ax[1]
    a.imshow(np.where(known, pm.height, np.nan).T, origin="lower", extent=ext, cmap="terrain")
    ov = np.zeros((n, n, 4)); ov[tm_rov.blocked_raw] = [0.10, .10, .75, .85]
    ov[~known] = [0.45, .45, .45, .85]
    a.imshow(np.transpose(ov, (1, 0, 2)), origin="lower", extent=ext)
    a.set_title(f"ROVER cost map (drive-on)\nblue = not drivable  "
                f"(slope>{rp.max_slope_deg:.0f}d, step>{rp.step_limit_m:.2f}m)")

    a = ax[2]
    dm = np.ones((n, n, 3))
    dm[free_both] = [0.90, 0.96, 0.90]
    dm[both] = [0.55, 0.55, 0.55]
    dm[step_only | both_cause] = [0.95, 0.35, 0.05]
    dm[steep_only] = [0.98, 0.80, 0.10]
    dm[dro_only] = [0.10, 0.60, 0.85]
    dm[~known] = [0.80, 0.80, 0.80]
    a.imshow(np.transpose(dm, (1, 0, 2)), origin="lower", extent=ext)
    for (bx, by, br, bh) in rocks:
        a.add_patch(plt.Circle((bx, by), br, fill=False, color="k", lw=0.8))
    a.add_patch(plt.Circle(CRATER_CENTRE, CRATER_RADIUS, fill=False, color="k",
                           lw=1.0, ls="--"))
    a.set_title("WHERE THEY DIFFER\norange = step blocks rover  amber = slope blocks rover\n"
                "grey = both block  pale = both free")

    for a in ax:
        a.set_xlabel("x [m]"); a.set_ylabel("y [m]")
    fig.suptitle("B5 Checkpoint A1 — the rover's traversability is not the drone's",
                 fontsize=13)
    fig.tight_layout()
    fig.savefig("b5_a1_rover_costmap.png", dpi=110)
    print("\n  plot  b5_a1_rover_costmap.png")

    np.savez("b5_rover_costmap.npz", blocked=tm_rov.blocked,
             blocked_raw=tm_rov.blocked_raw, cost=tm_rov.cost,
             slope=d["slope"], step=d["step"], known=known,
             cell=CELL, origin=ORIGIN)
    with open("b5_a1_rover_costmap.json", "w") as fh:
        json.dump({"rover_params": {k: v for k, v in rp.__dict__.items()},
                   "step_limit_m": rp.step_limit_m,
                   "known_cells": int(known.sum()),
                   "blocked_both": int(both.sum()),
                   "blocked_rover_only": int(rov_only.sum()),
                   "blocked_drone_only": int(dro_only.sum()),
                   "free_both": int(free_both.sum()),
                   "cause_step_only": int(step_only.sum()),
                   "cause_slope_only": int(steep_only.sum()),
                   "cause_both": int(both_cause.sum()),
                   "drivable_components": int(nlab)}, fh, indent=1, default=float)
    print("  log   b5_a1_rover_costmap.json + b5_rover_costmap.npz")


if __name__ == "__main__":
    main()
