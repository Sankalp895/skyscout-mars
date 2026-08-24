"""Same path, same everything, ONE variable changed: truth vs estimate feedback."""
import numpy as np
from skyscout_sim import SkyScoutSim, STEPS_PER_CONTROL
from nav_math import quat_to_R
from path_follow import PathFollower
from imu import SimulatedIMU
from eskf import ESKF
from nav_params import IMUParams, ESKFParams, FixParams

ALT = 12.0
path = np.array([[-5.,-10.,ALT],[5.,-10.,ALT],[5.,2.,ALT],[-5.,2.,ALT]])

for mode in ("truth", "estimate"):
    sim = SkyScoutSim(start_pos=tuple(path[0]), logging_level="error")
    ip, ep, fp = IMUParams(), ESKFParams(), FixParams()
    imu = SimulatedIMU(ip)
    s0 = sim.get_true_state()
    kf = ESKF(s0["pos"], s0["lin_vel"], s0["quat"], imu=ip, params=ep)
    rng = np.random.default_rng(fp.seed)
    fix_every = int(round(ip.rate_hz/fp.rate_hz))
    fol = PathFollower(path, speed=1.5, lookahead=2.5)
    k, done = 0, False
    tilts, ws, verr = [], [], []
    while sim.sim_time < 45.0 and not done:
        st = sim.get_true_state()
        gp = st["pos"] if mode=="truth" else kf.p
        tgt, tvel, frac, done = fol.target(gp)
        for _ in range(STEPS_PER_CONTROL):
            sim.apply_hover()
            if mode=="truth": sim.apply_track(tgt, tvel)
            else: sim.apply_track(tgt, tvel, meas_pos=kf.p, meas_vel=kf.v)
            sim.step_sim()
            s = sim.get_true_state()
            a = np.asarray(sim.drone.get_links_acc())[0]
            am, gm, _, _ = imu.measure(s["quat"], s["ang_vel"], a)
            kf.predict(am, gm, imu.dt)
            if k % fix_every == 0 and k > 0:
                kf.update_position(s["pos"] + rng.normal(0, fp.sigma_m, 3), fp.sigma_m)
            k += 1
        s = sim.get_true_state()
        R = quat_to_R(s["quat"])
        tilts.append(np.degrees(np.arccos(np.clip(R[2,2],-1,1))))
        ws.append(np.linalg.norm(s["ang_vel"]))
        verr.append(np.linalg.norm(s["lin_vel"] - kf.v))
    t = np.array(tilts); w = np.array(ws); ve = np.array(verr)
    print(f"{mode:9s} done={done}  tilt mean {t.mean():6.2f} max {t.max():7.2f} deg   "
          f"|w| max {w.max():6.3f} rad/s   |v_true-v_est| mean {ve.mean():.3f} max {ve.max():.3f}")
    del sim
