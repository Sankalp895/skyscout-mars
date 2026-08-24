"""Reproduce the step-1 flight loop and dump everything about each capture."""
import numpy as np
import camera_rig
from camera_rig import DownwardCamera
from skyscout_sim import SkyScoutSim, STEPS_PER_CONTROL
from mars_world import build_heightmap, EXTENT
from b3_flight import tm_height
from imu import SimulatedIMU
from eskf import ESKF
from nav_params import IMUParams, ESKFParams, FixParams
from path_follow import PathFollower

camera_rig.DEPTH_IS_EUCLIDEAN = False
camera_rig.FOV_IS_VERTICAL = True

RES, FOV, ALT = (320, 240), 60.0, 12.0
hmap, _, _ = build_heightmap()
path = np.array([[-24.0, 6.0, ALT], [-4.0, 6.0, ALT]])

sim = SkyScoutSim(start_pos=tuple(path[0]), logging_level="error",
                  camera_res=RES, camera_fov=FOV)
rig = DownwardCamera(sim, RES, FOV)
rig.warm_up()

ip, ep, fp = IMUParams(), ESKFParams(), FixParams()
imu = SimulatedIMU(ip)
s0 = sim.get_true_state()
kf = ESKF(s0["pos"], s0["lin_vel"], s0["quat"], imu=ip, params=ep)
rng_fix = np.random.default_rng(fp.seed)
fix_every = int(round(ip.rate_hz / fp.rate_hz))
follower = PathFollower(path, speed=1.5, lookahead=2.5)

leg_time = np.linalg.norm(path[1] - path[0]) / 1.5
cap_dt = leg_time / 7
nxt, k, i = 0.0, 0, 0

print(f"{'i':>2} {'t':>6} | {'true xyz':>24} | {'est xyz':>24} | "
      f"{'dz':>7} {'dmin':>7} {'dmax':>7} {'RMS':>9}")
while sim.sim_time < 30.0 and i < 8:
    st = sim.get_true_state()
    tgt, tvel, frac, done = follower.target(kf.p)
    if sim.sim_time >= nxt:
        depth, _, cam_pos, lookat = rig.capture(kf.p)
        d = np.asarray(depth, float)
        pts = rig.project(d, cam_pos, lookat)
        p = pts.reshape(-1, 3)
        m = ((np.abs(p[:, 0]) < EXTENT/2 - 1) & (np.abs(p[:, 1]) < EXTENT/2 - 1)
             & np.isfinite(p).all(1))
        pp = p[m]
        truth = np.array([tm_height(hmap, x, y) for x, y in pp[:, :2]])
        resid = pp[:, 2] - truth
        rms = float(np.sqrt(np.mean(resid ** 2)))
        t_, e_ = st["pos"], kf.p
        print(f"{i:2d} {sim.sim_time:6.2f} | "
              f"{t_[0]:7.3f}{t_[1]:8.3f}{t_[2]:8.3f} | "
              f"{e_[0]:7.3f}{e_[1]:8.3f}{e_[2]:8.3f} | "
              f"{e_[2]-t_[2]:7.3f} {d.min():7.3f} {d.max():7.3f} {rms:8.4f}m")
        if rms > 0.1:
            bad = np.abs(resid) > 0.5
            print(f"      ^ {bad.sum()}/{len(resid)} pixels off by >0.5 m; "
                  f"worst residual {resid[np.argmax(np.abs(resid))]:+.2f} m")
            full = np.zeros(p.shape[0], bool); full[np.where(m)[0][bad]] = True
            img = full.reshape(d.shape)
            rows, cols = np.where(img)
            print(f"        bad-pixel rows {rows.min()}-{rows.max()}, "
                  f"cols {cols.min()}-{cols.max()}  (frame is {d.shape})")
            print(f"        depth at those pixels: "
                  f"[{d[img].min():.3f}, {d[img].max():.3f}] m")
            print(f"        quat (true) {st['quat'].round(4)}  "
                  f"-> tilt {np.degrees(2*np.arccos(np.clip(abs(st['quat'][0]),0,1))):.2f} deg")
        nxt += cap_dt
        i += 1

    for _ in range(STEPS_PER_CONTROL):
        sim.apply_hover()
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
