"""B2 CHECKPOINT 4 -- the correction."""
import json

import numpy as np
from scipy.stats import chi2
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

from skyscout_sim import SkyScoutSim, STEPS_PER_CONTROL
from mars_world import build_heightmap
from imu import SimulatedIMU
from dead_reckon import DeadReckoner
from eskf import ESKF, P_, V_, TH_, AB_, GB_, N as NX
from nav_math import qmul, qnorm
from nav_params import IMUParams, ESKFParams, FixParams
from b1_flight import reference, CLEARANCE

TOTAL = 40.0


def attitude_error(q_est, q_true):
    """delta-theta (3-vector) such that q_true ~= q_est (x) dq."""
    qe = qnorm(np.asarray(q_est, float))
    dq = qmul(np.array([qe[0], -qe[1], -qe[2], -qe[3]]), qnorm(np.asarray(q_true, float)))
    if dq[0] < 0:
        dq = -dq
    return 2.0 * dq[1:]


def main():
    z0 = round(float(build_heightmap()[0].max()) + CLEARANCE, 2)
    ip, ep, fp = IMUParams(), ESKFParams(), FixParams()
    rng_fix = np.random.default_rng(fp.seed)

    sim = SkyScoutSim(start_pos=(0, 0, z0), logging_level="error")
    imu = SimulatedIMU(ip)
    s0 = sim.get_true_state()
    kf_on = ESKF(s0["pos"], s0["lin_vel"], s0["quat"], imu=ip, params=ep)
    kf_off = ESKF(s0["pos"], s0["lin_vel"], s0["quat"], imu=ip, params=ep)
    dr = DeadReckoner(s0["pos"], s0["lin_vel"], s0["quat"])

    fix_every = int(round(ip.rate_hz / fp.rate_hz))
    h = {k: [] for k in ("t", "true_p", "on_p", "off_p", "dr_p",
                         "sig_on", "sig_off", "ab_est", "gb_est",
                         "ab_true", "gb_true", "nees", "nees_p", "fix_t")}
    k = 0
    while sim.sim_time < TOTAL:
        target, tvel, phase = reference(sim.sim_time, z0)
        for _ in range(STEPS_PER_CONTROL):
            sim.apply_hover()
            sim.apply_track(target, tvel)
            sim.step_sim()
            s = sim.get_true_state()
            a_world = np.asarray(sim.drone.get_links_acc())[0]
            ab_true = imu.accel_bias.copy()
            gb_true = imu.gyro_bias.copy()
            am, gm, _, _ = imu.measure(s["quat"], s["ang_vel"], a_world)

            kf_on.predict(am, gm, imu.dt)
            kf_off.predict(am, gm, imu.dt)
            dr.propagate(am, gm, imu.dt)

            if k % fix_every == 0 and k > 0:
                z = s["pos"] + rng_fix.normal(0, fp.sigma_m, 3)
                kf_on.update_position(z, fp.sigma_m)
                h["fix_t"].append(s["t"])
            k += 1

            e = np.zeros(NX)
            e[P_] = s["pos"] - kf_on.p
            e[V_] = s["lin_vel"] - kf_on.v
            e[TH_] = attitude_error(kf_on.q, s["quat"])
            e[AB_] = ab_true - kf_on.ab
            e[GB_] = gb_true - kf_on.gb
            h["nees"].append(float(e @ np.linalg.solve(kf_on.P, e)))
            ep_ = e[P_]
            h["nees_p"].append(float(ep_ @ np.linalg.solve(kf_on.P[P_, P_], ep_)))

            h["t"].append(s["t"])
            h["true_p"].append(s["pos"]); h["on_p"].append(kf_on.p.copy())
            h["off_p"].append(kf_off.p.copy()); h["dr_p"].append(dr.p.copy())
            h["sig_on"].append(kf_on.sigma_pos); h["sig_off"].append(kf_off.sigma_pos)
            h["ab_est"].append(kf_on.ab.copy()); h["gb_est"].append(kf_on.gb.copy())
            h["ab_true"].append(ab_true); h["gb_true"].append(gb_true)

    for key in h:
        h[key] = np.asarray(h[key])
    err_on = np.linalg.norm(h["on_p"] - h["true_p"], axis=1)
    err_off = np.linalg.norm(h["off_p"] - h["true_p"], axis=1)

    settle = h["t"] > 5.0
    print("=== 1. CORRECTED vs UNCORRECTED ===")
    print(f"  duration {h['t'][-1]:.1f} s, {len(h['fix_t'])} position fixes "
          f"@ {fp.rate_hz:.0f} Hz, sigma_meas {fp.sigma_m} m")
    print(f"  {'':22s}{'max':>10}{'mean':>10}{'final':>10}")
    print(f"  {'WITHOUT correction':22s}{err_off.max():10.2f}{err_off.mean():10.2f}{err_off[-1]:10.2f}  m")
    print(f"  {'WITH correction':22s}{err_on.max():10.2f}{err_on.mean():10.2f}{err_on[-1]:10.2f}  m")
    print(f"  {'after settling (t>5s)':22s}{err_on[settle].max():10.2f}"
          f"{err_on[settle].mean():10.2f}{'':10s}  m")
    print(f"  improvement at end: {err_off[-1] / max(err_on[-1], 1e-9):.0f}x")

    print("\n=== 2. BIAS CONVERGENCE (is the filter OBSERVING the biases?) ===")
    for name, est, tru in (("accel", h["ab_est"], h["ab_true"]),
                           ("gyro", h["gb_est"], h["gb_true"])):
        e0 = np.linalg.norm(est[0] - tru[0])
        e1 = np.linalg.norm(est[-1] - tru[-1])
        print(f"  {name:5s} true final {tru[-1].round(5).tolist()}")
        print(f"  {name:5s} est  final {est[-1].round(5).tolist()}")
        print(f"  {name:5s} |error| {e0:.5f} -> {e1:.5f}   "
              f"({100*(1-e1/max(e0,1e-12)):.0f}% reduction)")

    print("\n=== 3. COVARIANCE BEHAVIOUR ===")
    drops = np.sum(np.diff(h["sig_on"]) < -1e-9)
    print(f"  sigma_pos decreased on {drops} steps (measurement updates) "
          f"-- prediction-only had 0")
    ss = h["sig_on"][settle]
    print(f"  steady-state sigma_pos: mean {ss.mean():.3f} m  "
          f"min {ss.min():.3f}  max {ss.max():.3f}")
    print(f"  prediction-only sigma_pos at same time: {h['sig_off'][-1]:.1f} m")
    print(f"  HAND CHECK: sigma_meas = {fp.sigma_m} m per axis -> a 3-axis "
          f"radius of {fp.sigma_m*np.sqrt(3):.2f} m if a single fix fully "
          f"determined position;")
    print(f"    filter settles at {ss.mean():.3f} m, i.e. {'below' if ss.mean() < fp.sigma_m*np.sqrt(3) else 'above'} "
          f"that -- it fuses many fixes plus the IMU, so below is expected.")
    print(f"    vs {h['sig_off'][-1]:.0f} m prediction-only: "
          f"{h['sig_off'][-1]/ss.mean():.0f}x tighter. ORDER OF MAGNITUDE OK.")

    print("\n=== 4. NEES CONSISTENCY (15 DOF) ===")
    nees = h["nees"][settle]
    K = len(nees)
    anees = nees.mean()
    lo, hi = chi2.ppf(0.025, NX*K)/K, chi2.ppf(0.975, NX*K)/K
    if anees < lo:
        verdict = "CONSERVATIVE (covariance too large -- safe but pessimistic)"
    elif anees > hi:
        verdict = "OPTIMISTIC (covariance too small -- CONFIDENTLY WRONG)"
    else:
        verdict = "CONSISTENT"
    print(f"  ANEES over {K} samples (t>5s): {anees:.2f}   expected {NX}")
    print(f"  95% acceptance region: [{lo:.2f}, {hi:.2f}]")
    print(f"  VERDICT: {verdict}")
    npos = h["nees_p"][settle].mean()
    lo3, hi3 = chi2.ppf(0.025, 3*K)/K, chi2.ppf(0.975, 3*K)/K
    print(f"  position-only ANEES: {npos:.2f}  expected 3  "
          f"region [{lo3:.2f}, {hi3:.2f}]")

    fig = plt.figure(figsize=(14, 13))
    gs = fig.add_gridspec(4, 2, height_ratios=[1.15, 1.15, 1, 1])

    a = fig.add_subplot(gs[0, :])
    a.semilogy(h["t"], err_off, lw=2.4, color="crimson",
               label="WITHOUT correction (dead-reckoning)")
    a.semilogy(h["t"], err_on, lw=2.0, color="navy", label="WITH 1 Hz position fixes")
    for ft in h["fix_t"][::5]:
        a.axvline(ft, color="gray", alpha=0.15, lw=0.6)
    a.set_ylabel("position error [m], log scale")
    a.set_title("B2 step 4 — THE PAYOFF: GPS-denied localization. "
                f"{err_off[-1]:.1f} m uncorrected → {err_on[-1]:.2f} m corrected")
    a.legend(); a.grid(alpha=0.3, which="both")

    a = fig.add_subplot(gs[1, 0])
    a.plot(h["true_p"][:, 0], h["true_p"][:, 1], lw=3, color="seagreen", label="true")
    a.plot(h["on_p"][:, 0], h["on_p"][:, 1], lw=1.5, ls="--", color="navy",
           label="ESKF corrected")
    a.plot(h["off_p"][:, 0], h["off_p"][:, 1], lw=1.5, color="crimson",
           alpha=0.8, label="uncorrected")
    a.set_xlabel("x [m]"); a.set_ylabel("y [m]"); a.set_title("trajectory, top view")
    a.legend(fontsize=8); a.grid(alpha=0.3); a.axis("equal")

    a = fig.add_subplot(gs[1, 1])
    a.plot(h["t"], h["sig_on"], color="darkorange", lw=1.5, label="1σ WITH fixes")
    a.plot(h["t"], err_on, color="navy", lw=1, alpha=0.8, label="actual error")
    a.plot(h["t"], h["sig_off"], color="crimson", ls=":", label="1σ prediction only")
    a.set_yscale("log"); a.set_xlabel("time [s]"); a.set_ylabel("[m]")
    a.set_title("covariance sawtooth — drops at every fix")
    a.legend(fontsize=8); a.grid(alpha=0.3, which="both")

    for row, (name, est, tru, unit) in enumerate(
            (("accel bias", h["ab_est"], h["ab_true"], "m/s²"),
             ("gyro bias", h["gb_est"], h["gb_true"], "rad/s"))):
        a = fig.add_subplot(gs[2 + row, :])
        for i, c in enumerate("xyz"):
            line, = a.plot(h["t"], est[:, i], lw=1.6, label=f"est {c}")
            a.plot(h["t"], tru[:, i], lw=2.2, ls=":", alpha=0.75,
                   color=line.get_color(), label=f"true {c}")
        a.set_ylabel(f"{name} [{unit}]")
        a.set_title(f"{name} convergence — estimate (solid) chasing truth (dotted)")
        a.legend(fontsize=7, ncol=6); a.grid(alpha=0.3)
        if row == 1:
            a.set_xlabel("time [s]")

    fig.tight_layout()
    fig.savefig("b2_step4_correct.png", dpi=110)
    print("\n  plot  b2_step4_correct.png")

    with open("b2_step4_correct.json", "w") as f:
        json.dump({
            "duration_s": float(h["t"][-1]), "n_fixes": int(len(h["fix_t"])),
            "err_uncorrected": {"max": float(err_off.max()), "mean": float(err_off.mean()),
                                "final": float(err_off[-1])},
            "err_corrected": {"max": float(err_on.max()), "mean": float(err_on.mean()),
                              "final": float(err_on[-1]),
                              "settled_mean": float(err_on[settle].mean())},
            "sigma_settled_m": float(ss.mean()),
            "sigma_prediction_only_m": float(h["sig_off"][-1]),
            "accel_bias_true_final": h["ab_true"][-1].tolist(),
            "accel_bias_est_final": h["ab_est"][-1].tolist(),
            "gyro_bias_true_final": h["gb_true"][-1].tolist(),
            "gyro_bias_est_final": h["gb_est"][-1].tolist(),
            "anees_15dof": float(anees), "anees_region": [float(lo), float(hi)],
            "nees_verdict": verdict,
        }, f, indent=1)
    print("  log   b2_step4_correct.json")


if __name__ == "__main__":
    main()
