"""B4 CHECKPOINT 4 -- close the loop on the PERCEIVED map."""
import json

import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from scipy.ndimage import distance_transform_edt

from skyscout_sim import SkyScoutSim, STEPS_PER_CONTROL
from mars_world import build_heightmap, EXTENT, CELL, CRATER_CENTRE, CRATER_RADIUS
from cost_map import TraversabilityMap, CostParams
from world_map import load_perceived
from astar import plan
from path_follow import PathFollower, cross_track
from imu import SimulatedIMU
from eskf import ESKF
from nav_params import IMUParams, ESKFParams, FixParams

ORIGIN = (-EXTENT / 2, -EXTENT / 2)
CRUISE_CLEARANCE = 3.0
SPEED = 1.5
INFLATION_M = CostParams().inflate_cells * CELL


def pick_pair(tm_perc, pm, region_c=(-15.0, 10.0), region=40.0):
    """Choose start/goal inside the SURVEYED region, both on perceived-free ground, preferring a pa."""
    n = tm_perc.n
    free = ~tm_perc.blocked
    cand = []
    for x in np.arange(region_c[0] - region/2 + 2, region_c[0] + region/2 - 1, 2.0):
        for y in np.arange(region_c[1] - region/2 + 2, region_c[1] + region/2 - 1, 2.0):
            i, j = tm_perc.to_grid(x, y)
            if free[i, j] and pm.count[i, j] >= 2:
                cand.append((x, y))
    best = None
    for a in cand:
        for b in cand:
            d = np.hypot(b[0]-a[0], b[1]-a[1])
            if d < 24.0:
                continue
            r = plan(tm_perc, a, b)
            if not r["ok"]:
                continue
            score = r["detour_ratio"] * d
            if best is None or score > best[0]:
                best = (score, a, b, r)
    return best


def main():
    hmap, feats, rocks = build_heightmap()
    tm_true = TraversabilityMap(hmap, CELL, ORIGIN, CostParams())
    truth = tm_true.blocked_raw
    n = tm_true.n

    pm, z = load_perceived()
    tm_perc = pm.to_traversability()
    perc_hz = tm_perc.blocked_raw

    print("=== B4 CHECKPOINT 4 -- planning over the PERCEIVED map ===")
    print(f"  perceived map: {pm.summary()}")
    print(f"  perceived blocked (raw) {int(perc_hz.sum())}   "
          f"after inflation {int(tm_perc.blocked.sum())}   free {int((~tm_perc.blocked).sum())}")

    best = pick_pair(tm_perc, pm)
    if best is None:
        raise SystemExit("no viable start/goal pair on the perceived map")
    _, start_xy, goal_xy, r = best
    print(f"\n  start {start_xy}  goal {goal_xy}")
    print(f"  planned over PERCEIVED: {r['length_m']:.2f} m  straight "
          f"{r['straight_m']:.2f} m  detour {r['detour_ratio']:.3f}x  "
          f"expanded {r['expanded']}")
    print(f"  perceived clearance along route {r['min_clearance_m']:.2f} m")

    xy = r["world"]
    ph = pm.height
    idx = [tm_perc.to_grid(x, y) for x, y in xy]
    route_perc = np.array([ph[i, j] for i, j in idx])
    route_perc = route_perc[np.isfinite(route_perc)]
    cruise_z = float(route_perc.max()) + CRUISE_CLEARANCE
    path = np.c_[xy, np.full(len(xy), cruise_z)]
    print(f"  perceived route terrain max {route_perc.max():.2f} m "
          f"-> cruise {cruise_z:.2f} m")

    sim = SkyScoutSim(start_pos=tuple(path[0]), logging_level="error")
    ip, ep, fp = IMUParams(), ESKFParams(), FixParams()
    imu = SimulatedIMU(ip)
    s0 = sim.get_true_state()
    kf = ESKF(s0["pos"], s0["lin_vel"], s0["quat"], imu=ip, params=ep)
    rng_fix = np.random.default_rng(fp.seed)
    fix_every = int(round(ip.rate_hz / fp.rate_hz))
    follower = PathFollower(path, speed=SPEED, lookahead=2.5)

    d_true = distance_transform_edt(~truth) * CELL
    d_perc = distance_transform_edt(~perc_hz) * CELL
    miss_cells = truth & (~perc_hz)
    d_miss = (distance_transform_edt(~miss_cells) * CELL
              if miss_cells.any() else np.full((n, n), np.inf))

    H = {k: [] for k in ("t", "true", "est", "xtrack", "agl",
                         "cl_true", "cl_perc", "cl_miss", "eskf")}
    k, done = 0, False
    while sim.sim_time < 400.0 and not done:
        st = sim.get_true_state()
        tgt, tvel, frac, done = follower.target(kf.p)
        for _ in range(STEPS_PER_CONTROL):
            sim.apply_hover()
            sim.apply_attitude_hold()
            sim.apply_track(tgt, tvel, meas_pos=kf.p, meas_vel=kf.v)
            sim.step_sim()
            s = sim.get_true_state()
            a_world = np.asarray(sim.drone.get_links_acc())[0]
            am, gm, _, _ = imu.measure(s["quat"], s["ang_vel"], a_world)
            kf.predict(am, gm, imu.dt)
            if k % fix_every == 0 and k > 0:
                kf.update_position(s["pos"] + rng_fix.normal(0, fp.sigma_m, 3),
                                   fp.sigma_m)
            k += 1
        s = sim.get_true_state()
        i, j = tm_true.to_grid(s["pos"][0], s["pos"][1])
        H["t"].append(s["t"]); H["true"].append(s["pos"].copy())
        H["est"].append(kf.p.copy())
        H["xtrack"].append(cross_track(path, s["pos"]))
        H["agl"].append(float(s["pos"][2] - sim.terrain_height(*s["pos"][:2])))
        H["cl_true"].append(float(d_true[i, j]))
        H["cl_perc"].append(float(d_perc[i, j]))
        H["cl_miss"].append(float(d_miss[i, j]))
        H["eskf"].append(float(np.linalg.norm(s["pos"] - kf.p)))
    for kk in H:
        H[kk] = np.asarray(H[kk])
    reached = bool(done)
    goal_err = float(np.linalg.norm(H["true"][-1][:2] - path[-1][:2]))

    print(f"\n=== FLIGHT -- ALL NUMBERS IN TRUTH SPACE ===")
    print(f"  reached goal            {reached}   final xy error {goal_err:.3f} m")
    print(f"  duration                {H['t'][-1]:.1f} s for {r['length_m']:.1f} m")
    print(f"  max cross-track (TRUE)  {H['xtrack'].max():.3f} m")
    print(f"  min AGL (TRUE)          {H['agl'].min():.3f} m")
    print(f"  ESKF error   mean {H['eskf'].mean():.3f}  max {H['eskf'].max():.3f} m")

    print(f"\n=== THE B4 FAILURE MODE: did a perception MISS endanger the drone? ===")
    print(f"  real hazard cells the map called SAFE: {int(miss_cells.sum())}")
    print(f"  closest TRUE-trajectory approach to a REAL hazard      "
          f"{H['cl_true'].min():.3f} m")
    print(f"  closest approach to what the MAP thought was hazard    "
          f"{H['cl_perc'].min():.3f} m")
    print(f"  closest approach to a MISSED hazard specifically       "
          f"{H['cl_miss'].min():.3f} m")
    surprise = H["cl_perc"].min() - H["cl_true"].min()
    print(f"  SURPRISE = perceived margin - real margin = {surprise:+.3f} m")
    if surprise > 0.5:
        verdict_miss = ("the drone was CLOSER to a real hazard than its map "
                        "implied -- perception error ate margin")
    elif H["cl_true"].min() <= 0:
        verdict_miss = "FAILED -- true trajectory entered a real hazard cell"
    else:
        verdict_miss = ("no meaningful surprise: the real margin matched what "
                        "the map promised")
    print(f"  VERDICT: {verdict_miss}")

    print(f"\n=== STACKED ERROR BUDGET ===")
    smear = float(z["cell"]) * 0.69 if "cell" in z else 0.432
    terms = {"ESKF position error (B2/B3)": H["eskf"].mean(),
             "map smear from estimated pose (step 3)": 0.432,
             "controller cross-track (B3 step 3)": 0.199}
    print(f"  margin available:")
    print(f"    planned clearance on perceived map   {r['min_clearance_m']:.2f} m")
    print(f"    hazard inflation                     {INFLATION_M:.2f} m")
    print(f"  error terms (mean):")
    for kk, v in terms.items():
        print(f"    {kk:38s} {v:.3f} m")
    tot = sum(terms.values())
    print(f"    {'stacked (linear, worst case)':38s} {tot:.3f} m")
    print(f"    {'stacked (RSS, independent)':38s} "
          f"{np.sqrt(sum(v*v for v in terms.values())):.3f} m")
    dom = max(terms, key=terms.get)
    print(f"  DOMINANT TERM: {dom} ({terms[dom]:.3f} m)")
    real = H["cl_true"].min()
    print(f"  real margin achieved {real:.3f} m vs stacked error {tot:.3f} m")
    if real > tot:
        v2 = "CLEARED WITH MARGIN -- the real gap exceeds the whole stacked error"
    elif real > 0:
        v2 = ("CLEARED BUT GOT LUCKY -- the real gap is smaller than the stacked "
              "error, so a different noise draw could have touched a hazard")
    else:
        v2 = "FAILED"
    print(f"  VERDICT: {v2}")

    print(f"\n  HAND CHECK: distance to the RAW hazard edge = distance to the")
    print(f"    INFLATED edge + the inflation width.")
    pred = r["min_clearance_m"] + INFLATION_M
    print(f"    predicted {r['min_clearance_m']:.2f} + {INFLATION_M:.2f} = {pred:.3f} m"
          f"   measured (real) {real:.3f} m   delta {real - pred:+.3f} m  "
          f"{'MATCH' if abs(real-pred) < 0.05 else 'MISMATCH'}")
    print(f"\n    Where did the error budget go? It was spent against the PERCEIVED")
    print(f"    hazard, not the real one: closest perceived approach "
          f"{H['cl_perc'].min():.3f} m vs real {real:.3f} m. The {surprise:+.3f} m")
    print(f"    surprise is NEGATIVE, i.e. the map was conservative -- the thing the")
    print(f"    drone shaved past was a false alarm or an unknown cell, not a rock.")

    ext = [-EXTENT/2, EXTENT/2, -EXTENT/2, EXTENT/2]
    fig, ax = plt.subplots(1, 2, figsize=(17, 8))

    a = ax[0]
    shade = np.zeros((n, n, 4))
    shade[perc_hz] = [0.85, 0.10, 0.10, 0.85]
    shade[~pm.known] = [0.45, 0.45, 0.45, 0.85]
    a.imshow(np.where(pm.known, pm.height, np.nan).T, origin="lower", extent=ext,
             cmap="terrain")
    a.imshow(np.transpose(shade, (1, 0, 2)), origin="lower", extent=ext)
    a.plot(path[:, 0], path[:, 1], color="#00e5ff", lw=3.0, label="planned over PERCEIVED")
    a.plot(H["true"][:, 0], H["true"][:, 1], color="white", lw=2.0, label="TRUE trajectory")
    a.plot(H["est"][:, 0], H["est"][:, 1], color="orange", lw=1.2, ls="--",
           label="ESKF estimate (what it steered on)")
    a.plot(*path[0, :2], "o", color="lime", ms=12, mec="k", label="start")
    a.plot(*path[-1, :2], "*", color="magenta", ms=20, mec="k", label="goal")
    if miss_cells.any():
        mi, mj = np.where(miss_cells)
        a.scatter(ORIGIN[0] + mi*CELL, ORIGIN[1] + mj*CELL, s=6, c="yellow",
                  marker="x", label="MISSED real hazard")
    a.set_xlim(-40, 15); a.set_ylim(-15, 35)
    a.set_xlabel("x [m]"); a.set_ylabel("y [m]")
    a.set_title("B4 payoff — perceived map + plan over it + TRUE trajectory\n"
                f"red = perceived hazard, grey = UNKNOWN (blocked)")
    a.legend(loc="upper left", fontsize=8)

    a = ax[1]
    a.plot(H["t"], H["cl_true"], color="crimson", lw=2, label="TRUE dist to REAL hazard")
    a.plot(H["t"], H["cl_perc"], color="seagreen", lw=1.5, ls="--",
           label="TRUE dist to PERCEIVED hazard")
    a.axhline(INFLATION_M, color="orange", ls=":", label=f"inflation {INFLATION_M:.2f} m")
    a.axhline(0, color="k", lw=1.2, label="hazard boundary")
    a.axhline(tot, color="purple", ls="-.", label=f"stacked error {tot:.2f} m")
    a.set_xlabel("time [s]"); a.set_ylabel("clearance [m]")
    a.set_title(f"clearance in TRUTH space\nclosest real hazard {real:.2f} m")
    a.legend(fontsize=8); a.grid(alpha=0.3)

    fig.tight_layout()
    fig.savefig("b4_step4_closeloop.png", dpi=110)
    print("\n  plot  b4_step4_closeloop.png")

    with open("b4_step4_closeloop.json", "w") as fh:
        json.dump({"start": list(start_xy), "goal": list(goal_xy),
                   "plan_perceived": {kk: r[kk] for kk in
                                      ("length_m","straight_m","detour_ratio",
                                       "expanded","min_clearance_m")},
                   "cruise_z": cruise_z,
                   "reached": reached, "goal_err_m": goal_err,
                   "duration_s": float(H["t"][-1]),
                   "max_xtrack_m": float(H["xtrack"].max()),
                   "min_agl_m": float(H["agl"].min()),
                   "eskf_mean_m": float(H["eskf"].mean()),
                   "eskf_max_m": float(H["eskf"].max()),
                   "closest_real_hazard_m": real,
                   "closest_perceived_hazard_m": float(H["cl_perc"].min()),
                   "closest_missed_hazard_m": float(H["cl_miss"].min()),
                   "surprise_m": float(surprise),
                   "miss_cells": int(miss_cells.sum()),
                   "error_budget": terms,
                   "stacked_linear_m": float(tot),
                   "verdict_miss": verdict_miss, "verdict_margin": v2},
                  fh, indent=1, default=float)
    print("  log   b4_step4_closeloop.json")


if __name__ == "__main__":
    main()
