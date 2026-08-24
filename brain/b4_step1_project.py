"""B4 CHECKPOINT 1 -- mount the camera, capture, project rays to the ground."""
import json

import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.colors import LinearSegmentedColormap

import camera_rig
from camera_rig import DownwardCamera, backproject, footprint_corners
from skyscout_sim import SkyScoutSim, STEPS_PER_CONTROL
from mars_world import build_heightmap, EXTENT, CELL, CRATER_CENTRE, CRATER_RADIUS
from b3_flight import tm_height
from imu import SimulatedIMU
from eskf import ESKF
from nav_params import IMUParams, ESKFParams, FixParams
from path_follow import PathFollower

RES = (320, 240)
FOV = 60.0
SURVEY_ALT = 12.0
ORIGIN = (-EXTENT / 2, -EXTENT / 2)

LEG_START = (-24.0, 6.0)
LEG_END = (-4.0, 6.0)
N_FRAMES = 8


def score_convention(depth, cam_pos, lookat, hmap, euclid, vert):
    """Back-project under one convention, compare implied terrain height to the known heightmap."""
    pts = backproject(depth, cam_pos, lookat, RES, FOV,
                      depth_is_euclidean=euclid, fov_is_vertical=vert)
    p = pts.reshape(-1, 3)
    m = ((np.abs(p[:, 0]) < EXTENT / 2 - 1) & (np.abs(p[:, 1]) < EXTENT / 2 - 1)
         & np.isfinite(p).all(1))
    if m.sum() < 100:
        return np.inf, 0
    p = p[m]
    truth = np.array([tm_height(hmap, x, y) for x, y in p[:, :2]])
    return float(np.sqrt(np.mean((p[:, 2] - truth) ** 2))), int(m.sum())


def main():
    hmap, feats, rocks = build_heightmap()
    report = {}

    sim = SkyScoutSim(start_pos=(LEG_START[0], LEG_START[1], SURVEY_ALT),
                      logging_level="error", camera_res=RES, camera_fov=FOV)
    rig = DownwardCamera(sim, RES, FOV)
    rig.warm_up()

    print("=== PART A -- determine depth / fov conventions by measurement ===")
    print(f"  (centre-pixel checks cannot separate these; scoring the WHOLE frame)")
    probe_xy = (-15.0, 10.0)
    sim.drone.set_pos(np.array([probe_xy[0], probe_xy[1], SURVEY_ALT]))
    depth, _, cam_pos, lookat = rig.capture((probe_xy[0], probe_xy[1], SURVEY_ALT))

    print(f"\n  {'depth convention':>20} {'fov axis':>10} {'RMS height err':>16} {'pts':>8}")
    best, results = None, {}
    for euclid in (True, False):
        for vert in (True, False):
            rms, n = score_convention(depth, cam_pos, lookat, hmap, euclid, vert)
            tag = f"{'euclidean' if euclid else 'planar-z'}/{'vert' if vert else 'horiz'}"
            results[tag] = {"rms_m": rms, "points": n}
            print(f"  {'euclidean' if euclid else 'planar-z':>20} "
                  f"{'vertical' if vert else 'horizontal':>10} {rms:15.4f} m {n:8d}")
            if best is None or rms < best[0]:
                best = (rms, euclid, vert, tag)
    rms, EUCLID, VERT, tag = best
    print(f"\n  WINNER: {tag}   RMS {rms:.4f} m")
    others = sorted(v["rms_m"] for k, v in results.items() if v["rms_m"] != rms)
    print(f"  next best is {others[0]:.4f} m -- a {others[0]/max(rms,1e-6):.1f}x "
          f"margin, so this is not a coin flip")
    camera_rig.DEPTH_IS_EUCLIDEAN = EUCLID
    camera_rig.FOV_IS_VERTICAL = VERT
    report["conventions"] = {"depth_is_euclidean": bool(EUCLID),
                             "fov_is_vertical": bool(VERT),
                             "rms_m": rms, "all": results}

    print("\n=== HAND CHECK -- footprint vs 2*h*tan(fov/2) ===")
    gnd = sim.terrain_height(*probe_xy)
    agl = float(cam_pos[2] - gnd)
    corners = footprint_corners(cam_pos, lookat, RES, FOV, ground_z=gnd)
    meas_w = float(np.linalg.norm(corners[1] - corners[0]))
    meas_h = float(np.linalg.norm(corners[3] - corners[0]))
    short = min(meas_w, meas_h)
    analytic = 2 * agl * np.tan(np.radians(FOV) / 2)
    n_short = min(RES)
    analytic_px = analytic * (n_short - 1) / n_short
    print(f"  camera AGL              {agl:.3f} m  "
          f"(body {SURVEY_ALT:.2f} + mount {camera_rig.MOUNT_OFFSET_BODY[2]:+.2f})")
    print(f"  measured footprint      {meas_w:.3f} x {meas_h:.3f} m")
    print(f"  analytic 2*h*tan(θ/2)   {analytic:.3f} m  "
          f"({'vertical' if VERT else 'horizontal'} axis, sensor edge-to-edge)")
    print(f"  same, pixel-centre span {analytic_px:.3f} m  (x {n_short-1}/{n_short})")
    print(f"  short axis {short:.3f} vs {analytic_px:.3f} -> "
          f"error {abs(short-analytic_px):.4f} m  "
          f"{'MATCH' if abs(short-analytic_px) < 0.02 else 'MISMATCH'}")
    print(f"  ground sampling         {100*short/n_short:.2f} cm/px")
    print(f"  cost-map cells covered  {meas_w*meas_h/CELL**2:.0f}")
    report["footprint"] = {"agl_m": agl, "measured_w_m": meas_w,
                           "measured_h_m": meas_h, "analytic_m": analytic,
                           "analytic_pixel_centre_m": analytic_px,
                           "cm_per_px": 100 * short / n_short}

    print(f"\n=== PART B -- fly a {np.hypot(*(np.subtract(LEG_END, LEG_START))):.0f} m "
          f"leg at {SURVEY_ALT} m, capture {N_FRAMES} frames ===")
    del sim, rig
    path = np.array([[LEG_START[0], LEG_START[1], SURVEY_ALT],
                     [LEG_END[0], LEG_END[1], SURVEY_ALT]])
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

    frames, k, done = [], 0, False
    next_capture, capture_dt = 0.0, None
    leg_time = np.linalg.norm(path[1] - path[0]) / 1.5
    capture_dt = leg_time / (N_FRAMES - 1)

    while sim.sim_time < 120.0 and len(frames) < N_FRAMES:
        st = sim.get_true_state()
        tgt, tvel, frac, done = follower.target(kf.p)
        if sim.sim_time >= next_capture:
            depth, rgb, cam_pos, lookat = rig.capture(st["pos"], want_rgb=False)
            n_self = rig.self_view_pixels(depth)
            if n_self:
                raise SystemExit(
                    f"ABORT: {n_self} pixels closer than 1 m at t={sim.sim_time:.2f}"
                    " -- the camera is imaging the airframe, not the ground.")
            pts = rig.project(depth, kf.p)
            frames.append({
                "t": float(sim.sim_time),
                "true_pos": st["pos"].copy(), "est_pos": kf.p.copy(),
                "cam_pos": cam_pos.copy(), "lookat": lookat.copy(),
                "pts": pts, "depth": depth,
                "fp_true": footprint_corners(
                    cam_pos, lookat, RES, FOV,
                    ground_z=sim.terrain_height(*cam_pos[:2])),
                "fp_est": footprint_corners(
                    *rig.pose_from(kf.p), RES, FOV,
                    ground_z=sim.terrain_height(*kf.p[:2])),
            })
            next_capture += capture_dt

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

    print(f"  captured {len(frames)} frames over {frames[-1]['t']:.1f} s")

    print("\n=== PROJECTION ACCURACY (vs known heightmap) ===")
    print(f"  {'frame':>6} {'t[s]':>6} {'ESKF err':>9} {'height RMS':>11} "
           f"{'footprint shift':>16}")
    rmss, shifts, eskf_errs = [], [], []
    for i, f in enumerate(frames):
        p = f["pts"].reshape(-1, 3)
        m = ((np.abs(p[:, 0]) < EXTENT/2 - 1) & (np.abs(p[:, 1]) < EXTENT/2 - 1)
             & np.isfinite(p).all(1))
        pp = p[m]
        truth = np.array([tm_height(hmap, x, y) for x, y in pp[:, :2]])
        rms = float(np.sqrt(np.mean((pp[:, 2] - truth) ** 2)))
        err = float(np.linalg.norm(f["true_pos"] - f["est_pos"]))
        shift = float(np.linalg.norm(f["fp_est"].mean(0) - f["fp_true"].mean(0)))
        rmss.append(rms); shifts.append(shift); eskf_errs.append(err)
        print(f"  {i:6d} {f['t']:6.2f} {err:8.3f}m {rms:10.4f}m {shift:15.3f}m")

    rmss, shifts, eskf_errs = map(np.asarray, (rmss, shifts, eskf_errs))
    print(f"\n  mean height RMS      {rmss.mean():.4f} m")
    print(f"  mean ESKF pos error  {eskf_errs.mean():.3f} m")
    print(f"  mean footprint shift {shifts.mean():.3f} m  "
          f"({shifts.mean()/CELL:.2f} cost-map cells)")
    print(f"\n  HAND CHECK: the footprint shift is the map SMEAR the ESKF injects.")
    print(f"    It should track the ESKF horizontal error almost exactly, since a")
    print(f"    downward camera translates its footprint one-for-one with position.")
    hxy = np.array([np.linalg.norm((f['true_pos'] - f['est_pos'])[:2]) for f in frames])
    print(f"    ESKF horizontal error mean {hxy.mean():.3f} m vs shift "
          f"{shifts.mean():.3f} m -> ratio {shifts.mean()/max(hxy.mean(),1e-9):.3f}  "
          f"{'CONSISTENT' if abs(shifts.mean()/max(hxy.mean(),1e-9) - 1) < 0.15 else 'INCONSISTENT'}")
    report["projection"] = {
        "mean_height_rms_m": float(rmss.mean()),
        "mean_eskf_err_m": float(eskf_errs.mean()),
        "mean_eskf_horiz_err_m": float(hxy.mean()),
        "mean_footprint_shift_m": float(shifts.mean()),
        "shift_in_cells": float(shifts.mean() / CELL),
        "per_frame": [{"t": f["t"], "rms_m": float(r), "shift_m": float(s),
                       "eskf_err_m": float(e)}
                      for f, r, s, e in zip(frames, rmss, shifts, eskf_errs)]}

    ext = [-EXTENT/2, EXTENT/2, -EXTENT/2, EXTENT/2]
    fig = plt.figure(figsize=(16, 7))
    gs = fig.add_gridspec(2, 3, width_ratios=[1.3, 1, 1])

    a = fig.add_subplot(gs[:, 0])
    a.imshow(hmap.T, origin="lower", extent=ext, cmap="terrain")
    a.add_patch(plt.Circle(CRATER_CENTRE, CRATER_RADIUS, fill=False,
                           color="cyan", lw=1.4, alpha=0.8, label="crater"))
    for x, y, r, hh in rocks:
        a.add_patch(plt.Circle((x, y), r, fill=False, color="red", lw=1.0, alpha=0.7))
    for i, f in enumerate(frames):
        c = f["fp_true"]
        a.add_patch(plt.Polygon(c, fill=False, color="white", lw=1.6,
                                label="true footprint" if i == 0 else None))
        c = f["fp_est"]
        a.add_patch(plt.Polygon(c, fill=False, color="orange", lw=1.1, ls="--",
                                label="believed footprint" if i == 0 else None))
    tp = np.array([f["true_pos"] for f in frames])
    a.plot(tp[:, 0], tp[:, 1], "o-", color="black", ms=4, lw=1.2,
           label="drone (true)")
    a.set_xlim(-32, 6); a.set_ylim(-8, 20)
    a.set_xlabel("x [m]"); a.set_ylabel("y [m]")
    a.set_title(f"B4 step 1 — {len(frames)} projected footprints at {SURVEY_ALT} m\n"
                f"footprint {meas_w:.1f} x {meas_h:.1f} m")
    a.legend(loc="upper left", fontsize=8)

    a = fig.add_subplot(gs[0, 1])
    p = frames[len(frames)//2]["pts"].reshape(-1, 3)
    sc = a.scatter(p[::7, 0], p[::7, 1], c=p[::7, 2], s=1, cmap="terrain")
    a.set_title("one frame's projected points\n(coloured by sensed height)", fontsize=9)
    a.set_aspect("equal"); plt.colorbar(sc, ax=a, label="z [m]")

    a = fig.add_subplot(gs[1, 1])
    a.plot([f["t"] for f in frames], rmss, "o-", color="crimson")
    a.set_xlabel("t [s]"); a.set_ylabel("height RMS [m]")
    a.set_title("projected height vs known terrain", fontsize=9); a.grid(alpha=0.3)

    a = fig.add_subplot(gs[0, 2])
    a.plot([f["t"] for f in frames], shifts, "o-", color="darkorange",
           label="footprint shift")
    a.plot([f["t"] for f in frames], hxy, "s--", color="navy",
           label="ESKF horiz error")
    a.set_xlabel("t [s]"); a.set_ylabel("[m]"); a.legend(fontsize=7)
    a.set_title("map smear = localization error", fontsize=9); a.grid(alpha=0.3)

    a = fig.add_subplot(gs[1, 2])
    d = frames[len(frames)//2]["depth"]
    im = a.imshow(d, cmap="viridis"); a.set_title("depth frame (mid-leg)", fontsize=9)
    plt.colorbar(im, ax=a, label="range [m]"); a.axis("off")

    fig.tight_layout()
    fig.savefig("b4_step1_project.png", dpi=110)
    print("\n  plot  b4_step1_project.png")

    with open("b4_step1_project.json", "w") as f:
        json.dump(report, f, indent=1, default=float)
    print("  log   b4_step1_project.json")


if __name__ == "__main__":
    main()
