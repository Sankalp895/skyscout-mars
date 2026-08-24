"""B2 CHECKPOINT 1 -- is the sensor model even right? No filter here."""
import json

import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

from skyscout_sim import SkyScoutSim, STEPS_PER_CONTROL, MARS_G, SIM_DT
from mars_world import build_heightmap
from imu import SimulatedIMU, MARS_G_VEC
from nav_params import IMUParams
from b1_flight import reference, CLIMB, CRUISE_X, HOLD_S, SPEED, CLEARANCE


def hand_checks(sim, imu):
    """The 'sphere-drop' equivalents for the IMU (B2.2)."""
    out = {}
    for _ in range(120):
        sim.apply_hover()
        sim.step_sim()
    s = sim.get_true_state()
    a_world = np.asarray(sim.drone.get_links_acc())[0]
    f, w = imu.true_signals(s["quat"], s["ang_vel"], a_world)
    out["hover_specific_force"] = f.round(4).tolist()
    out["hover_ok"] = bool(abs(f[2] - (-MARS_G)) < 0.05 and np.linalg.norm(f[:2]) < 0.05)

    # specific force: f_body = R.T @ (a_world - g_world). Gravity is re-added on integration.
    for _ in range(30):
        sim.step_sim()
    s = sim.get_true_state()
    a_world = np.asarray(sim.drone.get_links_acc())[0]
    f_ff, _ = imu.true_signals(s["quat"], s["ang_vel"], a_world)
    out["freefall_specific_force"] = f_ff.round(4).tolist()
    out["freefall_ok"] = bool(np.linalg.norm(f_ff) < 0.15)
    return out


def main():
    p = IMUParams()
    max_h = float(build_heightmap()[0].max())
    z0 = round(max_h + CLEARANCE, 2)

    sim = SkyScoutSim(start_pos=(0, 0, z0), logging_level="error")
    imu = SimulatedIMU(p)

    print("=== HAND CHECKS (before trusting anything) ===")
    hc = hand_checks(sim, imu)
    print(f"  hover  specific force {hc['hover_specific_force']}  "
          f"expect [0, 0, {-MARS_G:.2f}]   {'PASS' if hc['hover_ok'] else 'FAIL'}")
    print(f"  free-fall specific force {hc['freefall_specific_force']}  "
          f"expect ~[0,0,0]              {'PASS' if hc['freefall_ok'] else 'FAIL'}")

    del sim
    sim = SkyScoutSim(start_pos=(0, 0, z0), logging_level="error")
    imu = SimulatedIMU(p)

    total = CLIMB / SPEED + HOLD_S + CRUISE_X / SPEED + 3.0
    rec = {k: [] for k in ("t", "acc_m", "acc_t", "gyr_m", "gyr_t",
                           "abias", "gbias")}
    while sim.sim_time < total:
        target, tvel, phase = reference(sim.sim_time, z0)
        for _ in range(STEPS_PER_CONTROL):
            sim.apply_hover()
            sim.apply_track(target, tvel)
            sim.step_sim()
            s = sim.get_true_state()
            a_world = np.asarray(sim.drone.get_links_acc())[0]
            am, gm, ft, wt = imu.measure(s["quat"], s["ang_vel"], a_world)
            rec["t"].append(s["t"])
            rec["acc_m"].append(am); rec["acc_t"].append(ft)
            rec["gyr_m"].append(gm); rec["gyr_t"].append(wt)
            rec["abias"].append(imu.accel_bias.copy())
            rec["gbias"].append(imu.gyro_bias.copy())

    for k in rec:
        rec[k] = np.asarray(rec[k])
    n = len(rec["t"])

    ares = rec["acc_m"] - rec["acc_t"]
    gres = rec["gyr_m"] - rec["gyr_t"]
    print(f"\n=== IMU over {n} samples @ {p.rate_hz:.0f} Hz ({rec['t'][-1]:.1f} s) ===")
    print(f"  accel residual (meas-true)  mean {ares.mean(0).round(4).tolist()}  "
          f"std {ares.std(0).round(4).tolist()} m/s^2")
    print(f"  gyro  residual (meas-true)  mean {gres.mean(0).round(5).tolist()}  "
          f"std {gres.std(0).round(5).tolist()} rad/s")
    print(f"  predicted white-noise sigma  accel {imu.sig_a:.4f}   gyro {imu.sig_g:.5f}")
    print(f"  accel bias  start {np.asarray(p.accel_bias0).round(4).tolist()}  "
          f"end {rec['abias'][-1].round(4).tolist()}")
    print(f"  gyro  bias  start {np.asarray(p.gyro_bias0).round(5).tolist()}  "
          f"end {rec['gbias'][-1].round(5).tolist()}")

    t = rec["t"]
    fig, ax = plt.subplots(4, 1, figsize=(11, 12), sharex=True)
    lbl = "xyz"
    for i in range(3):
        ax[0].plot(t, rec["acc_m"][:, i], lw=0.5, alpha=0.55)
        ax[0].plot(t, rec["acc_t"][:, i], lw=1.6, label=f"true {lbl[i]}")
    ax[0].axhline(-MARS_G, ls=":", c="k", lw=1)
    ax[0].annotate(f"hover reads +{-MARS_G:.2f} on body z\n(feels thrust, not gravity)",
                   xy=(t[len(t)//6], -MARS_G), xytext=(t[len(t)//6], -MARS_G + 1.1),
                   fontsize=8, arrowprops=dict(arrowstyle="->", lw=0.8))
    ax[0].set_ylabel("specific force [m/s²]")
    ax[0].set_title("B2 step 1 — simulated IMU vs ground truth (faint = measured, bold = true)")
    ax[0].legend(fontsize=8, ncol=3)

    for i in range(3):
        ax[1].plot(t, rec["gyr_m"][:, i], lw=0.5, alpha=0.55)
        ax[1].plot(t, rec["gyr_t"][:, i], lw=1.6, label=f"true {lbl[i]}")
    ax[1].set_ylabel("angular rate [rad/s]")
    ax[1].legend(fontsize=8, ncol=3)

    for i in range(3):
        ax[2].plot(t, rec["abias"][:, i], label=f"accel bias {lbl[i]}")
    ax[2].set_ylabel("accel bias [m/s²]")
    ax[2].legend(fontsize=8, ncol=3)

    for i in range(3):
        ax[3].plot(t, rec["gbias"][:, i], label=f"gyro bias {lbl[i]}")
    ax[3].set_ylabel("gyro bias [rad/s]")
    ax[3].set_xlabel("time [s]")
    ax[3].legend(fontsize=8, ncol=3)
    for a in ax:
        a.grid(alpha=0.3)
    fig.tight_layout()
    fig.savefig("b2_step1_imu.png", dpi=110)
    print("\n  plot  b2_step1_imu.png")

    with open("b2_step1_imu.json", "w") as f:
        json.dump({"params": p.__dict__, "hand_checks": hc,
                   "accel_residual_mean": ares.mean(0).tolist(),
                   "accel_residual_std": ares.std(0).tolist(),
                   "gyro_residual_mean": gres.mean(0).tolist(),
                   "gyro_residual_std": gres.std(0).tolist()}, f, indent=1)
    print("  log   b2_step1_imu.json")


if __name__ == "__main__":
    main()
