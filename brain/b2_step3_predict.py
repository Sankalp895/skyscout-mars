"""B2 CHECKPOINT 3 -- ESKF prediction only."""
import json

import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

from skyscout_sim import SkyScoutSim, STEPS_PER_CONTROL
from mars_world import build_heightmap
from imu import SimulatedIMU
from dead_reckon import DeadReckoner
from eskf import ESKF
from nav_params import IMUParams, ESKFParams
from b1_flight import reference, CLIMB, CRUISE_X, HOLD_S, SPEED, CLEARANCE

TOTAL = CLIMB / SPEED + HOLD_S + CRUISE_X / SPEED + 3.0


def main():
    z0 = round(float(build_heightmap()[0].max()) + CLEARANCE, 2)
    ip, ep = IMUParams(), ESKFParams()

    sim = SkyScoutSim(start_pos=(0, 0, z0), logging_level="error")
    imu = SimulatedIMU(ip)
    s0 = sim.get_true_state()
    dr = DeadReckoner(s0["pos"], s0["lin_vel"], s0["quat"])
    kf = ESKF(s0["pos"], s0["lin_vel"], s0["quat"], imu=ip, params=ep)

    h = {k: [] for k in ("t", "true_p", "dr_p", "kf_p", "sig_p",
                         "sig_ax", "trP")}
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
            kf.predict(am, gm, imu.dt)
            h["t"].append(s["t"])
            h["true_p"].append(s["pos"])
            h["dr_p"].append(dr.p.copy())
            h["kf_p"].append(kf.p.copy())
            h["sig_p"].append(kf.sigma_pos)
            h["sig_ax"].append(kf.sigma_pos_axes)
            h["trP"].append(float(np.trace(kf.P)))
    for k in h:
        h[k] = np.asarray(h[k])

    kf_err = np.linalg.norm(h["kf_p"] - h["true_p"], axis=1)
    dr_err = np.linalg.norm(h["dr_p"] - h["true_p"], axis=1)
    overlay = np.abs(h["kf_p"] - h["dr_p"]).max()

    print("=== PASS CONDITION 1: ESKF prediction == dead-reckoning ===")
    print(f"  max |ESKF nominal - dead-reckoned| over all axes/time: {overlay:.3e} m")
    print(f"  ESKF final error {kf_err[-1]:.3f} m   DR final error {dr_err[-1]:.3f} m")
    c1 = overlay < 1e-9
    print(f"  {'PASS' if c1 else 'FAIL'} -- they are the same trajectory "
          f"(shared mechanization, no measurement leaked in)")
    print(f"  sanity: the filter did NOT track truth "
          f"({kf_err[-1]:.1f} m off) -- correct for prediction-only")

    print("\n=== PASS CONDITION 2: covariance grows monotonically ===")
    d_sig = np.diff(h["sig_p"])
    d_tr = np.diff(h["trP"])
    c2 = bool((d_sig >= -1e-12).all() and (d_tr >= -1e-12).all())
    print(f"  sigma_pos    min step delta {d_sig.min():+.3e}  "
          f"({'monotone' if (d_sig >= -1e-12).all() else 'NOT monotone'})")
    print(f"  trace(P)     min step delta {d_tr.min():+.3e}  "
          f"({'monotone' if (d_tr >= -1e-12).all() else 'NOT monotone'})")
    print(f"  sigma_pos {h['sig_p'][0]:.4f} m -> {h['sig_p'][-1]:.3f} m")
    print(f"  {'PASS' if c2 else 'FAIL'}")

    print("\n=== PASS CONDITION 3: 1-sigma brackets the true error ===")
    print(f"  HAND CHECK at t = {h['t'][-1]:.1f} s")
    print(f"    true position error        {kf_err[-1]:8.3f} m")
    print(f"    filter 1-sigma (3D radius) {h['sig_p'][-1]:8.3f} m")
    print(f"    per-axis 1-sigma           {h['sig_ax'][-1].round(3).tolist()} m")
    ratio = h["sig_p"][-1] / kf_err[-1]
    c3 = ratio >= 1.0
    print(f"    sigma / error = {ratio:.2f}  -> "
          f"{'brackets (honest/conservative)' if c3 else 'UNDER-REPORTS (confidently wrong)'}")
    pred = 0.5 * ep.sigma_ab0 * h["t"][-1] ** 2
    print(f"    cross-check: 0.5 * sigma_ab0 * t^2 = 0.5*{ep.sigma_ab0}*{h['t'][-1]:.1f}^2"
          f" = {pred:.2f} m per axis")
    print(f"    (the dominant term IS the unknown bias, as expected)")
    print(f"  {'PASS' if c3 else 'FAIL'}")

    print(f"\n=== VERDICT: {'ALL THREE PASS' if (c1 and c2 and c3) else 'FAILED'} ===")

    fig, ax = plt.subplots(3, 1, figsize=(11, 12))

    a = ax[0]
    a.plot(h["t"], h["dr_p"][:, 0], lw=5, alpha=0.35, color="crimson",
           label="step-2 dead-reckoning (thick)")
    a.plot(h["t"], h["kf_p"][:, 0], lw=1.3, color="navy", ls="--",
           label="ESKF nominal, prediction only")
    a.plot(h["t"], h["true_p"][:, 0], lw=1.6, color="seagreen", label="true")
    a.set_ylabel("x [m]"); a.set_title(
        f"B2 step 3 — ESKF prediction overlays dead-reckoning exactly "
        f"(max diff {overlay:.1e} m)")
    a.legend(fontsize=9); a.grid(alpha=0.3)

    a = ax[1]
    a.plot(h["t"], dr_err, lw=5, alpha=0.35, color="crimson",
           label="dead-reckoning error")
    a.plot(h["t"], kf_err, lw=1.3, ls="--", color="navy", label="ESKF error")
    a.plot(h["t"], h["sig_p"], lw=2, color="darkorange",
           label="ESKF reported 1σ")
    a.plot(h["t"], 3 * h["sig_p"], lw=1, ls=":", color="darkorange", label="3σ")
    a.fill_between(h["t"], 0, h["sig_p"], color="orange", alpha=0.15)
    a.set_ylabel("position error / uncertainty [m]")
    a.set_title("covariance must BRACKET the true error, never under-report it")
    a.annotate(f"1σ = {h['sig_p'][-1]:.1f} m  vs  true error {kf_err[-1]:.1f} m",
               xy=(h["t"][-1], h["sig_p"][-1]),
               xytext=(h["t"][-1] * 0.42, h["sig_p"][-1] * 1.05),
               arrowprops=dict(arrowstyle="->", lw=1.1), fontsize=10)
    a.legend(fontsize=9); a.grid(alpha=0.3)

    a = ax[2]
    for i, c in enumerate("xyz"):
        a.plot(h["t"], h["sig_ax"][:, i], label=f"1σ {c}")
    a.plot(h["t"], h["trP"], color="gray", ls=":", label="trace(P)")
    a.set_yscale("log"); a.set_xlabel("time [s]")
    a.set_ylabel("uncertainty [m] / trace(P)")
    a.set_title("covariance growth — monotone, no measurements to shrink it")
    a.legend(fontsize=9); a.grid(alpha=0.3)

    fig.tight_layout()
    fig.savefig("b2_step3_predict.png", dpi=110)
    print("\n  plot  b2_step3_predict.png")

    with open("b2_step3_predict.json", "w") as f:
        json.dump({"overlay_max_diff_m": float(overlay),
                   "eskf_final_err_m": float(kf_err[-1]),
                   "dr_final_err_m": float(dr_err[-1]),
                   "sigma_pos_final_m": float(h["sig_p"][-1]),
                   "sigma_over_error": float(ratio),
                   "monotone": bool(c2),
                   "pass": bool(c1 and c2 and c3)}, f, indent=1)
    print("  log   b2_step3_predict.json")


if __name__ == "__main__":
    main()
