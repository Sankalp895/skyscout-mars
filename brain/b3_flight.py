"""B3 steps 3 & 4 -- fly the planned path."""
import numpy as np

from skyscout_sim import SkyScoutSim, STEPS_PER_CONTROL
from mars_world import build_heightmap, EXTENT, CELL
from cost_map import TraversabilityMap, CostParams
from astar import plan
from path_follow import PathFollower, cross_track
from imu import SimulatedIMU
from eskf import ESKF
from nav_params import IMUParams, ESKFParams, FixParams

ORIGIN = (-EXTENT / 2, -EXTENT / 2)

PAIR_B = ((-35.0, -5.0), (10.0, -35.0))

CRUISE_CLEARANCE = 3.0
CRUISE_SPEED = 1.5
LOOKAHEAD = 2.5


def build_plan():
    h, _, _ = build_heightmap()
    tm = TraversabilityMap(h, CELL, ORIGIN, CostParams())
    r = plan(tm, *PAIR_B)
    assert r["ok"], r.get("reason")

    xy = r["world"]
    ground = np.array([tm_height(h, x, y) for x, y in xy])
    route_max = float(ground.max())
    cruise_z = route_max + CRUISE_CLEARANCE
    path = np.c_[xy, np.full(len(xy), cruise_z)]
    return tm, r, path, ground, route_max, cruise_z


def tm_height(h, x, y):
    """terrain height straight from the heightmap array (no sim needed)."""
    n = h.shape[0]
    fx = np.clip((x - ORIGIN[0]) / CELL, 0, n - 1)
    fy = np.clip((y - ORIGIN[1]) / CELL, 0, n - 1)
    i0, j0 = int(np.floor(fx)), int(np.floor(fy))
    i1, j1 = min(i0 + 1, n - 1), min(j0 + 1, n - 1)
    tx, ty = fx - i0, fy - j0
    return float(h[i0, j0]*(1-tx)*(1-ty) + h[i1, j0]*tx*(1-ty)
                 + h[i0, j1]*(1-tx)*ty + h[i1, j1]*tx*ty)


def fly(guidance, path, tm, max_time=120.0, seed_offset=0):
    """Fly the path. Returns the history dict."""
    start = path[0]
    sim = SkyScoutSim(start_pos=tuple(start), logging_level="error")
    ip, ep, fp = IMUParams(), ESKFParams(), FixParams()
    imu = SimulatedIMU(ip)
    s0 = sim.get_true_state()
    kf = ESKF(s0["pos"], s0["lin_vel"], s0["quat"], imu=ip, params=ep)
    rng_fix = np.random.default_rng(fp.seed + seed_offset)
    fix_every = int(round(ip.rate_hz / fp.rate_hz))

    follower = PathFollower(path, CRUISE_SPEED, LOOKAHEAD)
    h = {k: [] for k in ("t", "true", "est", "xtrack_true", "xtrack_guid",
                         "agl", "clearance", "sigma")}
    k, done = 0, False
    while sim.sim_time < max_time and not done:
        st = sim.get_true_state()
        guide_pos = st["pos"] if guidance == "truth" else kf.p
        tgt, tvel, frac, done = follower.target(guide_pos)

        for _ in range(STEPS_PER_CONTROL):
            sim.apply_hover()
            if guidance == "truth":
                sim.apply_track(tgt, tvel)
            else:
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
        h["t"].append(s["t"])
        h["true"].append(s["pos"].copy())
        h["est"].append(kf.p.copy())
        h["xtrack_true"].append(cross_track(path, s["pos"]))
        h["xtrack_guid"].append(cross_track(path, guide_pos))
        gnd = sim.terrain_height(s["pos"][0], s["pos"][1])
        h["agl"].append(float(s["pos"][2] - gnd))
        h["clearance"].append(tm.clearance_m(s["pos"][0], s["pos"][1]))
        h["sigma"].append(kf.sigma_pos)

    for key in h:
        h[key] = np.asarray(h[key])
    h["reached"] = bool(done)
    h["goal_err"] = float(np.linalg.norm(h["true"][-1] - path[-1]))
    h["goal_err_xy"] = float(np.linalg.norm(h["true"][-1][:2] - path[-1][:2]))
    del sim
    return h
