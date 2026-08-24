"""B5 part B -- collect the timeline the HUD narrates."""
import numpy as np

import camera_rig
from camera_rig import DownwardCamera
from skyscout_sim import SkyScoutSim, STEPS_PER_CONTROL
from nav_math import quat_to_R
from mars_world import build_heightmap, EXTENT, CELL
from hazard_classify import PerceptParams, score_frame
from world_map import PerceivedMap
from path_follow import PathFollower
from imu import SimulatedIMU
from eskf import ESKF
from nav_params import IMUParams, ESKFParams, FixParams
from b5_a2_rover_drive import SPEED as ROVER_SPEED

RES, FOV, ALT = (320, 240), 60.0, 12.0
ORIGIN = (-EXTENT/2, -EXTENT/2)
REGION_C, REGION, N_LEGS = (-15.0, 10.0), 40.0, 5
CAPTURE_EVERY_M = 3.0


def survey_path(alt):
    x0, x1 = REGION_C[0]-REGION/2, REGION_C[0]+REGION/2
    ys = np.linspace(REGION_C[1]-REGION/2, REGION_C[1]+REGION/2, N_LEGS)
    pts = []
    for k, y in enumerate(ys):
        xs = (x0, x1) if k % 2 == 0 else (x1, x0)
        pts += [[xs[0], y, alt], [xs[1], y, alt]]
    return np.array(pts, float)


def main():
    hmap, _, _ = build_heightmap()
    n = hmap.shape[0]
    pp = PerceptParams()
    path = survey_path(ALT)

    sim = SkyScoutSim(start_pos=tuple(path[0]), logging_level="error",
                      camera_res=RES, camera_fov=FOV)
    rig = DownwardCamera(sim, RES, FOV); rig.warm_up()
    pmap = PerceivedMap(n, CELL, ORIGIN)

    ip, ep, fp = IMUParams(), ESKFParams(), FixParams()
    imu = SimulatedIMU(ip)
    s0 = sim.get_true_state()
    kf = ESKF(s0["pos"], s0["lin_vel"], s0["quat"], imu=ip, params=ep)
    rng = np.random.default_rng(fp.seed)
    fix_every = int(round(ip.rate_hz/fp.rate_hz))
    fol = PathFollower(path, speed=1.5, lookahead=2.5)

    ii, jj = np.meshgrid(np.arange(n), np.arange(n), indexing="ij")
    region = ((np.abs(ORIGIN[0]+ii*CELL - REGION_C[0]) <= REGION/2)
              & (np.abs(ORIGIN[1]+jj*CELL - REGION_C[1]) <= REGION/2))

    snaps, k, done, last = [], 0, False, None
    while sim.sim_time < 400 and not done:
        st = sim.get_true_state()
        tgt, tvel, frac, done = fol.target(kf.p)
        if last is None or np.linalg.norm(st["pos"][:2]-last) >= CAPTURE_EVERY_M:
            depth, _, cp, la = rig.capture(st["pos"])
            if not rig.self_view_pixels(depth):
                r = score_frame(rig.project(depth, kf.p), CELL, ORIGIN, n, pp)
                pmap.fuse(r["score"], r["observed"], height=r["height"])
                snaps.append({
                    "t": float(sim.sim_time),
                    "true": st["pos"].copy(), "est": kf.p.copy(),
                    "err": float(np.linalg.norm(st["pos"]-kf.p)),
                    "known": pmap.known.copy(),
                    "haz": pmap.hazard(0.5).copy(),
                    "cov": float((pmap.known & region).sum()/region.sum()),
                })
            last = st["pos"][:2].copy()
        for _ in range(STEPS_PER_CONTROL):
            sim.apply_hover(); sim.apply_attitude_hold()
            sim.apply_track(tgt, tvel, meas_pos=kf.p, meas_vel=kf.v)
            sim.step_sim()
            s = sim.get_true_state()
            aw = np.asarray(sim.drone.get_links_acc())[0]
            am, gm, _, _ = imu.measure(s["quat"], s["ang_vel"], aw)
            kf.predict(am, gm, imu.dt)
            if k % fix_every == 0 and k > 0:
                kf.update_position(s["pos"]+rng.normal(0, fp.sigma_m, 3), fp.sigma_m)
            k += 1

    print(f"survey: {len(snaps)} snapshots, {sim.sim_time:.0f} s, "
          f"final coverage {snaps[-1]['cov']*100:.1f}%")

    rr = np.load("b5_rover_route.npz")
    np.savez_compressed(
        "b5_hud_timeline.npz",
        t=np.array([s["t"] for s in snaps]),
        drone_true=np.array([s["true"] for s in snaps]),
        drone_est=np.array([s["est"] for s in snaps]),
        err=np.array([s["err"] for s in snaps]),
        cov=np.array([s["cov"] for s in snaps]),
        known=np.array([s["known"] for s in snaps]),
        haz=np.array([s["haz"] for s in snaps]),
        rover_path=rr["drive_xy"], rover_start=rr["start"], rover_goal=rr["goal"],
        rover_speed=ROVER_SPEED, hmap=hmap, cell=CELL, origin=ORIGIN,
        region=region)
    print("wrote b5_hud_timeline.npz")


if __name__ == "__main__":
    main()
