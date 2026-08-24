"""B3 CHECKPOINT 4 -- fly pair B on the ESKF ESTIMATE."""
import json

import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.colors import LinearSegmentedColormap

from mars_world import EXTENT, CRATER_CENTRE, CRATER_RADIUS
from cost_map import CostParams
from b3_flight import build_plan, fly, CRUISE_CLEARANCE

INFLATION_M = CostParams().inflate_cells * (EXTENT / 127)


def main():
    tm, r, path, ground, route_max, cruise_z = build_plan()
    print("=== PLAN (pair B) ===")
    print(f"  length {r['length_m']:.2f} m  detour {r['detour_ratio']:.3f}x  "
          f"cruise z {cruise_z:.2f} m")
    print(f"  route clearance to nearest hazard: {r['min_clearance_m']:.2f} m")
    print(f"  hazard inflation margin:           {INFLATION_M:.2f} m")

    print("\n  flying on GROUND TRUTH (step-3 baseline) ...")
    T = fly("truth", path, tm)
    print("  flying on ESKF ESTIMATE ...")
    E = fly("estimate", path, tm)

    n = min(len(T["true"]), len(E["true"]))
    traj_diff = np.linalg.norm(T["true"][:n] - E["true"][:n], axis=1)
    same = int((traj_diff < 1e-9).sum())
    print(f"\n=== GUARD: is the estimate actually steering? ===")
    print(f"  |truth_traj - est_traj|  mean {traj_diff.mean():.4f}  "
          f"max {traj_diff.max():.4f} m")
    print(f"  bit-identical samples: {same}/{n}")
    if same > 0.5 * n:
        raise SystemExit(
            "ABORT: the two runs are the same flight -- truth is leaking into "
            "the control path. Do not report these numbers.")
    print("  the two flights genuinely differ -- the swap is live")

    print("\n=== CHECKPOINT 4 -- ALL NUMBERS IN TRUTH SPACE ===")
    print(f"  {'':34s}{'truth-guided':>15}{'estimate-guided':>18}")
    print(f"  {'reached goal':34s}{str(T['reached']):>15}{str(E['reached']):>18}")
    print(f"  {'final goal error (true) [m]':34s}{T['goal_err']:15.3f}{E['goal_err']:18.3f}")
    print(f"  {'MAX cross-track, TRUE [m]':34s}{T['xtrack_true'].max():15.3f}"
          f"{E['xtrack_true'].max():18.3f}")
    print(f"  {'mean cross-track, TRUE [m]':34s}{T['xtrack_true'].mean():15.3f}"
          f"{E['xtrack_true'].mean():18.3f}")
    print(f"  {'min AGL [m]':34s}{T['agl'].min():15.3f}{E['agl'].min():18.3f}")
    print(f"  {'CLOSEST hazard approach, TRUE [m]':34s}"
          f"{T['clearance'].min():15.3f}{E['clearance'].min():18.3f}")
    print(f"  {'duration [s]':34s}{T['t'][-1]:15.1f}{E['t'][-1]:18.1f}")

    print(f"\n  cross-track measured in ESTIMATE space: "
          f"{E['xtrack_guid'].max():.3f} m  <-- what the drone BELIEVED")
    print(f"  cross-track measured in TRUTH space:    "
          f"{E['xtrack_true'].max():.3f} m  <-- what actually happened")
    print("  (judging on the first number is the trap; only the second counts)")

    eskf_err = np.linalg.norm(E["true"] - E["est"], axis=1)
    print(f"\n=== ESKF ERROR DURING THE FLIGHT ===")
    print(f"  mean {eskf_err.mean():.3f} m   max {eskf_err.max():.3f} m   "
          f"final {eskf_err[-1]:.3f} m")
    print(f"  reported 1-sigma mean {E['sigma'].mean():.3f} m")

    print(f"\n=== SAFETY MARGIN VERDICT ===")
    print(f"  route was planned {r['min_clearance_m']:.2f} m from the nearest hazard")
    print(f"  hazards already inflated by {INFLATION_M:.2f} m beyond the real obstacle")
    print(f"  controller's own contribution (step 3): {T['xtrack_true'].max():.3f} m")
    print(f"  ESKF max position error:                {eskf_err.max():.3f} m")
    print(f"  TRUE trajectory got within {E['clearance'].min():.3f} m of a hazard cell")
    margin = E["clearance"].min()
    budget = r["min_clearance_m"]
    if margin <= 0:
        verdict = "FAILED -- localization error pushed the real drone into a hazard"
    elif margin < 0.5:
        verdict = "PASSED BUT MARGINAL -- got lucky, not comfortably clear"
    else:
        verdict = (f"PASSED WITH MARGIN -- {margin:.2f} m of the {budget:.2f} m "
                   f"budget survived, plus {INFLATION_M:.2f} m inflation still in hand")
    print(f"  VERDICT: {verdict}")

    print(f"\n  HAND CHECK vs B2: steady-state ESKF error was ~0.5-0.6 m mean "
          f"(1.80 m max).")
    print(f"    Here: {eskf_err.mean():.3f} m mean, {eskf_err.max():.3f} m max "
          f"-- {'consistent' if eskf_err.mean() < 1.5 else 'INCONSISTENT'} with B2.")
    print(f"    Clearance lost vs the truth-guided run: "
          f"{T['clearance'].min() - E['clearance'].min():+.3f} m, which should be "
          f"the same order as the ESKF error. It is "
          f"{'consistent' if abs(T['clearance'].min()-E['clearance'].min()) < 3*max(eskf_err.mean(),0.1) else 'NOT consistent'}.")

    ext = [-EXTENT / 2, EXTENT / 2, -EXTENT / 2, EXTENT / 2]
    cmap = LinearSegmentedColormap.from_list(
        "trav", ["#12341f", "#2e7d32", "#c9b037", "#e65100"])
    cost_show = np.where(np.isfinite(tm.cost), tm.cost, np.nan)

    fig = plt.figure(figsize=(16, 8))
    gs = fig.add_gridspec(3, 2, width_ratios=[1.25, 1])

    a = fig.add_subplot(gs[:, 0])
    a.imshow(cost_show.T, origin="lower", extent=ext, cmap=cmap,
             vmin=1.0, vmax=np.nanpercentile(cost_show, 99))
    blk = np.zeros((*tm.blocked.shape, 4)); blk[tm.blocked] = [0.85, .06, .06, 1]
    a.imshow(np.transpose(blk, (1, 0, 2)), origin="lower", extent=ext)
    a.add_patch(plt.Circle(CRATER_CENTRE, CRATER_RADIUS, fill=False,
                           color="cyan", lw=1.2, alpha=0.6))
    a.plot(path[:, 0], path[:, 1], color="#00e5ff", lw=3.2, label="planned (A*)")
    a.plot(E["true"][:, 0], E["true"][:, 1], color="white", lw=2.0,
           label="TRUE trajectory")
    a.plot(E["est"][:, 0], E["est"][:, 1], color="orange", lw=1.4, ls="--",
           label="ESKF estimate (what it steered on)")
    a.plot(*path[0, :2], "o", color="lime", ms=11, mec="k", label="start")
    a.plot(*path[-1, :2], "*", color="magenta", ms=18, mec="k", label="goal")
    a.set_xlabel("x [m]"); a.set_ylabel("y [m]")
    a.set_title("B3 step 4 — flown on the ESKF ESTIMATE\n"
                f"TRUE max cross-track {E['xtrack_true'].max():.3f} m,  "
                f"closest hazard {E['clearance'].min():.2f} m")
    a.legend(loc="upper right", fontsize=8)

    a = fig.add_subplot(gs[0, 1])
    a.plot(E["t"], E["xtrack_true"], color="navy", label="TRUE vs plan")
    a.plot(E["t"], E["xtrack_guid"], color="orange", ls="--", alpha=0.8,
           label="ESTIMATE vs plan (what it believed)")
    a.plot(T["t"], T["xtrack_true"], color="gray", ls=":", label="truth-guided baseline")
    a.set_ylabel("cross-track [m]"); a.legend(fontsize=7); a.grid(alpha=0.3)
    a.set_title("cross-track — truth space vs estimate space")

    a = fig.add_subplot(gs[1, 1])
    a.plot(E["t"], eskf_err, color="crimson", label="|true - estimate|")
    a.plot(E["t"], E["sigma"], color="darkorange", ls=":", label="ESKF 1σ")
    a.set_ylabel("ESKF error [m]"); a.legend(fontsize=7); a.grid(alpha=0.3)

    a = fig.add_subplot(gs[2, 1])
    a.plot(E["t"], E["clearance"], color="seagreen", label="TRUE dist to hazard")
    a.axhline(0, color="crimson", ls="--", lw=1.2, label="hazard boundary")
    a.axhline(INFLATION_M, color="orange", ls=":", lw=1.2,
              label=f"real obstacle ({INFLATION_M:.2f} m inflation)")
    a.set_xlabel("time [s]"); a.set_ylabel("clearance [m]")
    a.legend(fontsize=7); a.grid(alpha=0.3)

    fig.tight_layout()
    fig.savefig("b3_step4_estimate.png", dpi=110)
    print("\n  plot  b3_step4_estimate.png")

    with open("b3_step4_estimate.json", "w") as f:
        json.dump({
            "route_clearance_m": r["min_clearance_m"],
            "inflation_m": INFLATION_M,
            "truth_guided": {
                "reached": T["reached"], "goal_err_m": T["goal_err"],
                "max_xtrack_true_m": float(T["xtrack_true"].max()),
                "min_hazard_clearance_m": float(T["clearance"].min())},
            "estimate_guided": {
                "reached": E["reached"], "goal_err_m": E["goal_err"],
                "max_xtrack_true_m": float(E["xtrack_true"].max()),
                "max_xtrack_estimate_m": float(E["xtrack_guid"].max()),
                "min_hazard_clearance_m": float(E["clearance"].min()),
                "min_agl_m": float(E["agl"].min())},
            "eskf_err_mean_m": float(eskf_err.mean()),
            "eskf_err_max_m": float(eskf_err.max()),
            "verdict": verdict}, f, indent=1)
    print("  log   b3_step4_estimate.json")


if __name__ == "__main__":
    main()
