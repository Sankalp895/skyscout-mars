"""B5 CHECKPOINT A2 -- plan the rover over its own map and drive it."""
import json

import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from scipy.ndimage import label

from mars_world import build_heightmap, EXTENT, CELL, CRATER_CENTRE, CRATER_RADIUS
from world_map import load_perceived, PerceivedMap
from rover_cost_map import rover_cost_map, RoverParams, local_step
from astar import plan
from rover_sim import RoverSim, SIM_HZ
from nav_math import quat_to_R

ORIGIN = (-EXTENT / 2, -EXTENT / 2)
WHEEL_R = 0.29
SPEED = 0.6


def true_rover_map(hmap, pm, rp):
    """Rover traversability from the TRUE heights -- SCORING ONLY."""
    tp = PerceivedMap(pm.n, pm.cell, pm.origin)
    tp.count = np.ones_like(pm.count)
    tp._h_sum = hmap.copy()
    tp._h_cnt = np.ones_like(pm.count)
    tp.logodds = np.full_like(pm.logodds, -10.0)
    tm, d = rover_cost_map(tp, rp)
    return tm, d


def main():
    hmap, feats, rocks = build_heightmap()
    pm, z = load_perceived()
    rp = RoverParams()
    tm_rov, d = rover_cost_map(pm, rp)
    tm_true_rov, d_true = true_rover_map(hmap, pm, rp)

    lbl, nlab = label(~tm_rov.blocked)
    sizes = np.bincount(lbl.ravel())[1:]
    main_lbl = int(np.argmax(sizes)) + 1
    ii, jj = np.where(lbl == main_lbl)
    world = np.c_[ORIGIN[0] + ii * CELL, ORIGIN[1] + jj * CELL]
    idx = np.random.default_rng(3).choice(len(world), size=min(400, len(world)),
                                          replace=False)
    W = world[idx]
    D = np.linalg.norm(W[:, None, :] - W[None, :, :], axis=-1)
    a, b = np.unravel_index(np.argmax(D), D.shape)
    start_xy, goal_xy = tuple(W[a]), tuple(W[b])

    print("=== B5 CHECKPOINT A2 -- rover plan + drive ===")
    print(f"  drivable component: {sizes[main_lbl-1]} cells of "
          f"{int((~tm_rov.blocked).sum())} drivable total ({nlab} components)")
    print(f"  start {np.round(start_xy,1)}  goal {np.round(goal_xy,1)}")

    r = plan(tm_rov, start_xy, goal_xy)
    if not r["ok"]:
        raise SystemExit(f"no rover route: {r['reason']}")
    path = r["world"]
    print(f"  ROUTE over the ROVER map: {r['length_m']:.2f} m  straight "
          f"{r['straight_m']:.2f} m  detour {r['detour_ratio']:.3f}x  "
          f"expanded {r['expanded']}")
    print(f"  min clearance to non-drivable {r['min_clearance_m']:.2f} m")

    sim = RoverSim(start_xy=start_xy)
    ride, _ = sim.settle(2.0)
    print(f"\n  measured ride height {ride:.3f} m (chassis origin above ground)")

    log = []
    for k in range(len(path)):
        xy = path[k]
        nxt = path[min(k + 1, len(path) - 1)]
        yaw = float(np.arctan2(nxt[1] - xy[1], nxt[0] - xy[0]))
        p = sim.set_pose(xy, yaw, ride)
        log.append({"xy": [float(xy[0]), float(xy[1])], "z": float(p[2]),
                    "yaw": yaw})
    drive_xy = np.array([l["xy"] for l in log])
    print(f"  KINEMATIC drive: {len(log)} waypoints, "
          f"{r['length_m']/SPEED:.0f} s at {SPEED} m/s")

    cells = [tm_rov.to_grid(x, y) for x, y in drive_xy]
    bad_true = [(i, j) for i, j in cells if tm_true_rov.blocked_raw[i, j]]
    bad_perc = [(i, j) for i, j in cells if tm_rov.blocked_raw[i, j]]
    slope_on_route = np.array([d_true["slope"][i, j] for i, j in cells])
    step_on_route = np.array([d_true["step"][i, j] for i, j in cells])
    print(f"\n=== TRUTH SPACE -- is the driven route actually drivable? ===")
    print(f"  cells entered that PERCEIVED map called undrivable   {len(bad_perc)}")
    print(f"  cells entered that TRUE terrain says are undrivable  {len(bad_true)}"
          f"   <- the perception-miss failure mode")
    print(f"  TRUE slope on route  max {slope_on_route.max():.1f} deg "
          f"(limit {rp.max_slope_deg:.0f})")
    print(f"  TRUE step  on route  max {step_on_route.max():.2f} m "
          f"(limit {rp.step_limit_m:.2f})")
    ok_route = len(bad_true) == 0

    print(f"\n=== PHYSICS-DRIVEN SEGMENT -- the rocker-bogie differential ===")
    st = d_true["step"]
    cand = np.argwhere((st > 0.12) & (st < rp.step_limit_m)
                       & (~tm_true_rov.blocked_raw))
    res = {}
    if len(cand):
        ci, cj = cand[len(cand)//2]
        sx, sy = ORIGIN[0] + ci*CELL, ORIGIN[1] + cj*CELL
        print(f"  step feature at ({sx:.1f}, {sy:.1f}), height {st[ci,cj]:.2f} m "
              f"(under the {rp.step_limit_m:.2f} m limit, so it should be climbable)")
        for use_diff in (True, False):
            rs = RoverSim(start_xy=(sx - 3.0, sy))
            for _ in range(int(1.5*SIM_HZ)):
                if use_diff: rs.apply_differential()
                rs.drive(0.0); rs.step()
            tilts, rock, xs = [], [], []
            for _ in range(int(9.0*SIM_HZ)):
                if use_diff: rs.apply_differential()
                rs.drive(SPEED / WHEEL_R)
                rs.step()
                s = rs.state()
                R = quat_to_R(s["quat"])
                tilts.append(np.degrees(np.arccos(np.clip(R[2,2], -1, 1))))
                aa = rs.artic_angles()
                rock.append([np.degrees(aa["rocker_port"]),
                             np.degrees(aa["rocker_stbd"])])
                xs.append(s["pos"][0])
            tilts = np.array(tilts); rock = np.array(rock); xs = np.array(xs)
            tag = "WITH diff " if use_diff else "NO diff   "
            print(f"  {tag} chassis tilt mean {tilts.mean():5.2f}  max {tilts.max():6.2f} deg"
                  f"   travelled {xs[-1]-xs[0]:5.2f} m")
            print(f"             rocker sum |port+stbd| mean "
                  f"{np.abs(rock.sum(1)).mean():6.2f} deg  (differential drives this to 0)")
            res["with" if use_diff else "without"] = {
                "tilt_mean": float(tilts.mean()), "tilt_max": float(tilts.max()),
                "rocker_sum_mean": float(np.abs(rock.sum(1)).mean()),
                "travelled_m": float(xs[-1]-xs[0]),
                "tilts": tilts.tolist()[::20], "rock": rock.tolist()[::20]}
            del rs
        w, wo = res["with"], res["without"]
        print(f"\n  DIFFERENTIAL VERDICT: chassis tilt {wo['tilt_max']:.2f} deg without"
              f" -> {w['tilt_max']:.2f} deg with "
              f"({100*(1-w['tilt_max']/max(wo['tilt_max'],1e-9)):.0f}% reduction)")
        print(f"  This is the FIRST verification of the rocker-bogie differential.")
        print(f"  URDF cannot express it (Track A had to emit an SDF <plugin>);")
        print(f"  left out, the chassis pitches with the rockers instead of at their mean.")

    ext = [-EXTENT/2, EXTENT/2, -EXTENT/2, EXTENT/2]
    fig = plt.figure(figsize=(17, 7.5))
    gs = fig.add_gridspec(2, 3, width_ratios=[1.5, 1, 1])

    a = fig.add_subplot(gs[:, 0])
    a.imshow(np.where(pm.known, pm.height, np.nan).T, origin="lower", extent=ext,
             cmap="terrain")
    ov = np.zeros((pm.n, pm.n, 4)); ov[tm_rov.blocked_raw] = [0.10,.10,.75,.80]
    ov[~pm.known] = [0.45,.45,.45,.85]
    a.imshow(np.transpose(ov, (1,0,2)), origin="lower", extent=ext)
    surv = np.load("b4_step3_mapping.json") if False else None
    a.plot(drive_xy[:,0], drive_xy[:,1], color="#ffd400", lw=3, label="ROVER route (driven)")
    a.plot(*start_xy, "o", color="lime", ms=12, mec="k", label="rover start")
    a.plot(*goal_xy, "*", color="magenta", ms=20, mec="k", label="rover goal")
    a.set_xlim(-40, 15); a.set_ylim(-20, 35)
    a.set_xlabel("x [m]"); a.set_ylabel("y [m]")
    a.set_title(f"Rover route over its OWN cost map\n{r['length_m']:.1f} m, "
                f"detour {r['detour_ratio']:.2f}x  (blue = not drivable, grey = unknown)")
    a.legend(loc="upper left", fontsize=8)

    a = fig.add_subplot(gs[0, 1])
    a.plot(slope_on_route, color="darkorange", label="TRUE slope on route")
    a.axhline(rp.max_slope_deg, color="crimson", ls="--", label=f"limit {rp.max_slope_deg:.0f}d")
    a.set_ylabel("slope [deg]"); a.legend(fontsize=7); a.grid(alpha=.3)
    a.set_title("truth-space slope along the driven route", fontsize=9)

    a = fig.add_subplot(gs[1, 1])
    a.plot(step_on_route, color="seagreen", label="TRUE step on route")
    a.axhline(rp.step_limit_m, color="crimson", ls="--",
              label=f"wheel radius {rp.step_limit_m:.2f} m")
    a.set_xlabel("waypoint"); a.set_ylabel("step [m]"); a.legend(fontsize=7); a.grid(alpha=.3)

    if res:
        a = fig.add_subplot(gs[0, 2])
        a.plot(res["without"]["tilts"], color="crimson", label="NO differential")
        a.plot(res["with"]["tilts"], color="navy", label="WITH differential")
        a.set_ylabel("chassis tilt [deg]"); a.legend(fontsize=7); a.grid(alpha=.3)
        a.set_title("rocker-bogie over a step", fontsize=9)
        a = fig.add_subplot(gs[1, 2])
        rk = np.array(res["with"]["rock"])
        a.plot(rk[:,0], color="teal", label="rocker port")
        a.plot(rk[:,1], color="purple", label="rocker stbd")
        a.plot(rk.sum(1), color="k", ls=":", label="sum (diff -> 0)")
        a.set_xlabel("sample"); a.set_ylabel("[deg]"); a.legend(fontsize=7); a.grid(alpha=.3)

    fig.suptitle("B5 Checkpoint A2 — rover plans and drives over the drone's map",
                 fontsize=13)
    fig.tight_layout()
    fig.savefig("b5_a2_rover_drive.png", dpi=110)
    print("\n  plot  b5_a2_rover_drive.png")

    np.savez("b5_rover_route.npz", path=path, drive_xy=drive_xy,
             start=start_xy, goal=goal_xy)
    with open("b5_a2_rover_drive.json", "w") as fh:
        json.dump({"start": list(map(float, start_xy)), "goal": list(map(float, goal_xy)),
                   "route": {k: r[k] for k in ("length_m","straight_m","detour_ratio",
                                               "expanded","min_clearance_m")},
                   "ride_height_m": ride,
                   "cells_undrivable_true": len(bad_true),
                   "cells_undrivable_perceived": len(bad_perc),
                   "true_slope_max_deg": float(slope_on_route.max()),
                   "true_step_max_m": float(step_on_route.max()),
                   "route_truly_drivable": bool(ok_route),
                   "differential": {k: {kk: vv for kk, vv in v.items()
                                        if kk not in ("tilts","rock")}
                                    for k, v in res.items()}}, fh, indent=1, default=float)
    print("  log   b5_a2_rover_drive.json + b5_rover_route.npz")


if __name__ == "__main__":
    main()
