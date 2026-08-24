"""B2 CHECKPOINT 2 -- dead-reckoning."""
import json
from dataclasses import replace

import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

from skyscout_sim import SkyScoutSim, STEPS_PER_CONTROL
from mars_world import build_heightmap
from imu import SimulatedIMU
from dead_reckon import DeadReckoner
from nav_math import quat_angle_between
from nav_params import IMUParams
from b1_flight import reference, CLIMB, CRUISE_X, HOLD_S, SPEED, CLEARANCE

TOTAL = CLIMB / SPEED + HOLD_S + CRUISE_X / SPEED + 3.0


def run(imu_params, z0, label):
    """Fly the B1 path, dead-reckon alongside it, return the history."""
    sim = SkyScoutSim(start_pos=(0, 0, z0), logging_level="error")
    imu = SimulatedIMU(imu_params)
    s0 = sim.get_true_state()
    dr = DeadReckoner(s0["pos"], s0["lin_vel"], s0["quat"])

    h = {k: [] for k in ("t", "true_p", "dr_p", "true_v", "dr_v", "att_err")}
    while sim.sim_time < TOTAL:
        target, tvel, phase = reference(sim.sim_time, z0)
        for _ in range(STEPS_PER_CONTROL):
            sim.apply_hover()
            sim.apply_track(target, tvel)
            sim.step_sim()
            s = sim.get_true_state()
            a_world = np.asarray(sim.drone.get_links_acc())[0]
            am, gm, _, _ = imu.measure(s["quat"], s["ang_vel"], a_world)
            dr.propagate(am, gm, imu.dt)
            p, v, q = dr.state
            h["t"].append(s["t"])
            h["true_p"].append(s["pos"]); h["dr_p"].append(p)
            h["true_v"].append(s["lin_vel"]); h["dr_v"].append(v)
            h["att_err"].append(np.degrees(quat_angle_between(q, s["quat"])))
    for k in h:
        h[k] = np.asarray(h[k])
    h["perr"] = np.linalg.norm(h["dr_p"] - h["true_p"], axis=1)
    h["verr"] = np.linalg.norm(h["dr_v"] - h["true_v"], axis=1)
    h["label"] = label
    del sim
    return h


def main():
    z0 = round(float(build_heightmap()[0].max()) + CLEARANCE, 2)

    perfect = IMUParams(gyro_noise_density=0.0, gyro_bias_rw=0.0,
                        accel_noise_density=0.0, accel_bias_rw=0.0,
                        gyro_bias0=(0, 0, 0), accel_bias0=(0, 0, 0))
    A = run(perfect, z0, "perfect IMU (integrator check)")
    print("=== A. HAND CHECK: perfect IMU, no noise, no bias ===")
    print(f"  position error after {A['t'][-1]:.1f} s : {A['perr'][-1]:.4f} m")
    print(f"  max position error         : {A['perr'].max():.4f} m")
    print(f"  max velocity error         : {A['verr'].max():.4f} m/s")
    print(f"  max attitude error         : {A['att_err'].max():.4f} deg")
    ok = A["perr"][-1] < 0.05
    print(f"  integrator correct: {'PASS' if ok else 'FAIL'} "
          f"(residual is pure discretization, not sensor error)")

    B = run(IMUParams(), z0, "real IMU (noise + drifting bias)")
    print(f"\n=== B. DEAD-RECKONING with the real IMU ===")
    for frac in (0.25, 0.5, 0.75, 1.0):
        i = min(int(frac * (len(B['t']) - 1)), len(B['t']) - 1)
        print(f"  t = {B['t'][i]:5.1f} s   position error {B['perr'][i]:8.3f} m"
              f"   velocity error {B['verr'][i]:7.3f} m/s"
              f"   attitude error {B['att_err'][i]:6.3f} deg")
    print(f"\n  FINAL DRIFT after {B['t'][-1]:.1f} s : {B['perr'][-1]:.3f} m")
    print(f"  per-axis final offset : {(B['dr_p'][-1] - B['true_p'][-1]).round(3).tolist()} m")
    t_end = B["t"][-1]
    ab = np.array(IMUParams().accel_bias0)
    print(f"  predicted from accel bias alone, 0.5*|b|*t^2 = "
          f"{0.5 * np.linalg.norm(ab) * t_end**2:.3f} m "
          f"(|b| = {np.linalg.norm(ab):.4f} m/s^2)")

    fig = plt.figure(figsize=(13, 10))
    gs = fig.add_gridspec(3, 2, height_ratios=[1.25, 1, 1])

    ax = fig.add_subplot(gs[0, :])
    ax.plot(B["t"], B["perr"], lw=2.2, color="crimson",
            label="real IMU — dead-reckoned position error")
    ax.plot(A["t"], A["perr"], lw=1.6, color="seagreen",
            label="perfect IMU — integrator only (hand check)")
    ax.set_ylabel("position error [m]")
    ax.set_title("B2 step 2 — dead-reckoning drift (NO corrections). "
                 "The error the ESKF must crush in step 4.")
    ax.annotate(f"{B['perr'][-1]:.1f} m off after {B['t'][-1]:.1f} s",
                xy=(B["t"][-1], B["perr"][-1]),
                xytext=(B["t"][-1] * 0.55, B["perr"][-1] * 0.75),
                arrowprops=dict(arrowstyle="->", lw=1.2), fontsize=11)
    ax.legend(); ax.grid(alpha=0.3)

    ax = fig.add_subplot(gs[1, 0])
    ax.plot(B["true_p"][:, 0], B["true_p"][:, 2], lw=2, label="true")
    ax.plot(B["dr_p"][:, 0], B["dr_p"][:, 2], lw=2, ls="--", label="dead-reckoned")
    ax.set_xlabel("x [m]"); ax.set_ylabel("z [m]")
    ax.set_title("trajectory, side view"); ax.legend(fontsize=8); ax.grid(alpha=0.3)

    ax = fig.add_subplot(gs[1, 1])
    for i, c in enumerate("xyz"):
        ax.plot(B["t"], B["dr_p"][:, i] - B["true_p"][:, i], label=f"{c} error")
    ax.set_ylabel("per-axis error [m]"); ax.set_xlabel("time [s]")
    ax.set_title("error by axis (bias double-integrates)")
    ax.legend(fontsize=8); ax.grid(alpha=0.3)

    ax = fig.add_subplot(gs[2, 0])
    ax.plot(B["t"], B["verr"], color="darkorange")
    ax.set_ylabel("velocity error [m/s]"); ax.set_xlabel("time [s]")
    ax.set_title("velocity error (bias integrated once)"); ax.grid(alpha=0.3)

    ax = fig.add_subplot(gs[2, 1])
    ax.plot(B["t"], B["att_err"], color="purple")
    ax.set_ylabel("attitude error [deg]"); ax.set_xlabel("time [s]")
    ax.set_title("attitude error (gyro bias integrated once)"); ax.grid(alpha=0.3)

    fig.tight_layout()
    fig.savefig("b2_step2_deadreckon.png", dpi=110)
    print("\n  plot  b2_step2_deadreckon.png")

    with open("b2_step2_deadreckon.json", "w") as f:
        json.dump({
            "perfect_imu_final_pos_err_m": float(A["perr"][-1]),
            "perfect_imu_max_pos_err_m": float(A["perr"].max()),
            "real_imu_final_pos_err_m": float(B["perr"][-1]),
            "real_imu_final_vel_err_mps": float(B["verr"][-1]),
            "real_imu_final_att_err_deg": float(B["att_err"][-1]),
            "real_imu_final_offset_m": (B["dr_p"][-1] - B["true_p"][-1]).tolist(),
            "duration_s": float(B["t"][-1]),
        }, f, indent=1)
    print("  log   b2_step2_deadreckon.json")


if __name__ == "__main__":
    main()
