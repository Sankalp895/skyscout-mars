"""B3 CHECKPOINT 3 -- fly pair B on GROUND TRUTH."""
import json

import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.colors import LinearSegmentedColormap

from mars_world import EXTENT, CRATER_CENTRE, CRATER_RADIUS
from b3_flight import build_plan, fly, CRUISE_CLEARANCE, CRUISE_SPEED


def main():
    tm, r, path, ground, route_max, cruise_z = build_plan()
    print("=== PLAN (pair B) ===")
    print(f"  waypoints        {len(path)}")
    print(f"  path length      {r['length_m']:.2f} m  (straight {r['straight_m']:.2f} m,"
          f" detour {r['detour_ratio']:.3f}x)")
    print(f"  route terrain    min {ground.min():.2f}  max {route_max:.2f} m")
    print(f"  cruise altitude  {cruise_z:.2f} m  (= route max + {CRUISE_CLEARANCE})")
    print(f"  route clearance to nearest hazard: {r['min_clearance_m']:.2f} m")

    h = fly("truth", path, tm)

    print(f"\n=== FLIGHT ON GROUND TRUTH ===")
    print(f"  reached goal          {h['reached']}   "
          f"final error {h['goal_err']:.3f} m (xy {h['goal_err_xy']:.3f} m)")
    print(f"  duration              {h['t'][-1]:.1f} s at {CRUISE_SPEED} m/s")
    print(f"  MAX cross-track       {h['xtrack_true'].max():.3f} m")
    print(f"  mean cross-track      {h['xtrack_true'].mean():.3f} m")
    print(f"  min AGL               {h['agl'].min():.3f} m   "
          f"{'CLEARS TERRAIN' if h['agl'].min() > 0 else 'GROUND CONTACT'}")
    print(f"  min hazard clearance  {h['clearance'].min():.3f} m "
          f"(route plan had {r['min_clearance_m']:.2f} m)")
    clean = h["xtrack_true"].max() < 1.5 and h["agl"].min() > 0 and h["reached"]
    print(f"  VERDICT: {'NEAR-CLEAN -- safe to proceed to step 4' if clean else 'NOT CLEAN'}")

    ext = [-EXTENT / 2, EXTENT / 2, -EXTENT / 2, EXTENT / 2]
    cmap = LinearSegmentedColormap.from_list(
        "trav", ["#12341f", "#2e7d32", "#c9b037", "#e65100"])
    cost_show = np.where(np.isfinite(tm.cost), tm.cost, np.nan)

    fig = plt.figure(figsize=(15, 7))
    gs = fig.add_gridspec(2, 2, width_ratios=[1.2, 1])

    a = fig.add_subplot(gs[:, 0])
    a.imshow(cost_show.T, origin="lower", extent=ext, cmap=cmap,
             vmin=1.0, vmax=np.nanpercentile(cost_show, 99))
    blk = np.zeros((*tm.blocked.shape, 4)); blk[tm.blocked] = [0.85, .06, .06, 1]
    a.imshow(np.transpose(blk, (1, 0, 2)), origin="lower", extent=ext)
    a.add_patch(plt.Circle(CRATER_CENTRE, CRATER_RADIUS, fill=False,
                           color="cyan", lw=1.2, alpha=0.7))
    a.plot(path[:, 0], path[:, 1], color="#00e5ff", lw=3, label="planned (A*)")
    a.plot(h["true"][:, 0], h["true"][:, 1], color="white", lw=1.6, ls="--",
           label="flown (ground truth)")
    a.plot(*path[0, :2], "o", color="lime", ms=11, mec="k", label="start")
    a.plot(*path[-1, :2], "*", color="magenta", ms=18, mec="k", label="goal")
    a.set_xlabel("x [m]"); a.set_ylabel("y [m]")
    a.set_title(f"B3 step 3 — flown on GROUND TRUTH\n"
                f"max cross-track {h['xtrack_true'].max():.3f} m")
    a.legend(loc="upper right", fontsize=8)

    a = fig.add_subplot(gs[0, 1])
    a.plot(h["t"], h["xtrack_true"], color="navy")
    a.set_ylabel("cross-track [m]"); a.grid(alpha=0.3)
    a.set_title("cross-track error vs plan")

    a = fig.add_subplot(gs[1, 1])
    a.plot(h["t"], h["agl"], color="seagreen", label="AGL")
    a.axhline(0, color="crimson", ls="--", lw=1, label="ground")
    a.set_xlabel("time [s]"); a.set_ylabel("altitude above ground [m]")
    a.set_title(f"altitude profile — min AGL {h['agl'].min():.2f} m")
    a.legend(fontsize=8); a.grid(alpha=0.3)

    fig.tight_layout()
    fig.savefig("b3_step3_truth.png", dpi=110)
    print("\n  plot  b3_step3_truth.png")

    with open("b3_step3_truth.json", "w") as f:
        json.dump({"plan": {"length_m": r["length_m"], "detour": r["detour_ratio"],
                            "cruise_z": cruise_z, "route_max_terrain": route_max,
                            "route_clearance_m": r["min_clearance_m"]},
                   "reached": h["reached"], "goal_err_m": h["goal_err"],
                   "max_cross_track_m": float(h["xtrack_true"].max()),
                   "mean_cross_track_m": float(h["xtrack_true"].mean()),
                   "min_agl_m": float(h["agl"].min()),
                   "min_hazard_clearance_m": float(h["clearance"].min()),
                   "duration_s": float(h["t"][-1])}, f, indent=1)
    print("  log   b3_step3_truth.json")


if __name__ == "__main__":
    main()
