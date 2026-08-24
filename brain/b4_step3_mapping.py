"""B4 CHECKPOINT 3 -- fly a survey, accumulate one persistent world hazard map."""
import json

import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

import camera_rig
from camera_rig import DownwardCamera
from skyscout_sim import SkyScoutSim, STEPS_PER_CONTROL
from nav_math import quat_to_R
from mars_world import build_heightmap, EXTENT, CELL, CRATER_CENTRE, CRATER_RADIUS
from cost_map import TraversabilityMap, CostParams
from hazard_classify import PerceptParams, score_frame, confusion
from world_map import PerceivedMap, boundary_displacement
from path_follow import PathFollower
from imu import SimulatedIMU
from eskf import ESKF
from nav_params import IMUParams, ESKFParams, FixParams

RES, FOV, ALT = (320, 240), 60.0, 12.0
ORIGIN = (-EXTENT / 2, -EXTENT / 2)

REGION_C = (-15.0, 10.0)
REGION = 40.0
N_LEGS = 5
CAPTURE_EVERY_M = 3.0
SPEED = 1.5


def survey_path(alt):
    """Lawnmower: legs along x, stepping in y."""
    x0, x1 = REGION_C[0] - REGION / 2, REGION_C[0] + REGION / 2
    ys = np.linspace(REGION_C[1] - REGION / 2, REGION_C[1] + REGION / 2, N_LEGS)
    pts = []
    for k, y in enumerate(ys):
        xs = (x0, x1) if k % 2 == 0 else (x1, x0)
        pts.append([xs[0], y, alt])
        pts.append([xs[1], y, alt])
    return np.array(pts, float)


def main():
    hmap, feats, rocks = build_heightmap()
    tm_true = TraversabilityMap(hmap, CELL, ORIGIN, CostParams())
    truth = tm_true.blocked_raw
    n = tm_true.n
    pp = PerceptParams()

    path = survey_path(ALT)
    leg_len = REGION
    total_len = float(np.sum(np.linalg.norm(np.diff(path, axis=0), axis=1)))

    print("=== B4 CHECKPOINT 3 -- survey mapping ===")
    print(f"  region {REGION:.0f}x{REGION:.0f} m centred {REGION_C}   "
          f"{N_LEGS} legs, {total_len:.0f} m of track")
    in_region = [r for r in rocks
                 if abs(r[0]-REGION_C[0]) < REGION/2 and abs(r[1]-REGION_C[1]) < REGION/2]
    print(f"  contains the crater (r {CRATER_RADIUS} m) and {len(in_region)} boulders")

    sim = SkyScoutSim(start_pos=tuple(path[0]), logging_level="error",
                      camera_res=RES, camera_fov=FOV)
    rig = DownwardCamera(sim, RES, FOV)
    rig.warm_up()

    pmap = PerceivedMap(n, CELL, ORIGIN)

    ip, ep, fp = IMUParams(), ESKFParams(), FixParams()
    imu = SimulatedIMU(ip)
    s0 = sim.get_true_state()
    kf = ESKF(s0["pos"], s0["lin_vel"], s0["quat"], imu=ip, params=ep)
    rng_fix = np.random.default_rng(fp.seed)
    fix_every = int(round(ip.rate_hz / fp.rate_hz))
    follower = PathFollower(path, speed=SPEED, lookahead=2.5)

    k, done = 0, False
    skipped = []
    last_cap = None
    eskf_err, smear, tilts = [], [], []
    n_frames = 0
    max_t = total_len / SPEED * 1.8 + 30

    while sim.sim_time < max_t and not done:
        st = sim.get_true_state()
        tgt, tvel, frac, done = follower.target(kf.p)

        if last_cap is None or np.linalg.norm(st["pos"][:2] - last_cap) >= CAPTURE_EVERY_M:
            depth, _, cam_pos, lookat = rig.capture(st["pos"])
            R = quat_to_R(st["quat"])
            tilt = float(np.degrees(np.arccos(np.clip(R[2, 2], -1, 1))))
            yaw = float(np.degrees(np.arctan2(R[1, 0], R[0, 0])))
            nself = rig.self_view_pixels(depth)
            if nself:
                skipped.append({"t": float(sim.sim_time), "tilt_deg": tilt,
                                "yaw_deg": yaw, "n_self_px": nself,
                                "min_depth": float(np.asarray(depth).min())})
                if len(skipped) == 1:
                    np.save("b4_rejected_depth.npy", np.asarray(depth))
                    np.save("b4_rejected_state.npy",
                            np.array([tilt, yaw, st["pos"][0], st["pos"][1],
                                      st["pos"][2], sim.terrain_height(*st["pos"][:2])]))
            else:
                pts = rig.project(depth, kf.p)
                r = score_frame(pts, CELL, ORIGIN, n, pp)
                pmap.fuse(r["score"], r["observed"], height=r["height"])
                n_frames += 1
                eskf_err.append(float(np.linalg.norm(st["pos"] - kf.p)))
                smear.append(float(np.linalg.norm((st["pos"] - kf.p)[:2])))
                tilts.append(tilt)
            last_cap = st["pos"][:2].copy()

        for _ in range(STEPS_PER_CONTROL):
            sim.apply_hover()
            sim.apply_attitude_hold()
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

    print(f"\n  flew {sim.sim_time:.1f} s, {n_frames} frames fused, "
          f"survey completed: {done}")

    print(f"\n=== FRAMES REJECTED (airframe in frame) ===")
    print(f"  {len(skipped)} of {n_frames + len(skipped)} captures rejected "
          f"({100*len(skipped)/max(n_frames+len(skipped),1):.1f}%)")
    if skipped:
        tl = np.array([s["tilt_deg"] for s in skipped])
        kept_tilt = np.asarray(tilts) if len(tilts) else np.array([0.0])
        yw = np.array([s["yaw_deg"] for s in skipped])
        md = np.array([s["min_depth"] for s in skipped])
        print(f"  rejected TILT from vertical  mean {tl.mean():.2f}  max {tl.max():.2f} deg")
        print(f"  rejected YAW                 range [{yw.min():.1f}, {yw.max():.1f}] deg")
        print(f"  rejected min depth in frame  mean {md.mean():.3f}  min {md.min():.3f} m")
        print(f"  accepted TILT from vertical  mean {kept_tilt.mean():.2f}  max {kept_tilt.max():.2f} deg")
        print(f"  -> the camera hangs {abs(camera_rig.MOUNT_OFFSET_BODY[2]):.2f} m below "
              f"the body origin; a rotor arm at ~0.6 m radius drops below that")
        print(f"     once the body tilts past ~{np.degrees(np.arcsin(min(1.0, abs(camera_rig.MOUNT_OFFSET_BODY[2])/0.6))):.1f} deg, "
              f"which is what happens on the lawnmower turns.")
        ts = np.array([s["t"] for s in skipped])
        print(f"  rejection times: {np.round(ts,1).tolist()[:12]}"
              f"{' ...' if len(ts) > 12 else ''}")

    eskf_err = np.asarray(eskf_err); smear = np.asarray(smear); tilts = np.asarray(tilts)

    ii, jj = np.meshgrid(np.arange(n), np.arange(n), indexing="ij")
    wx = ORIGIN[0] + ii * CELL
    wy = ORIGIN[1] + jj * CELL
    region = ((np.abs(wx - REGION_C[0]) <= REGION/2)
              & (np.abs(wy - REGION_C[1]) <= REGION/2))

    known = pmap.known & region
    unknown = region & ~pmap.known
    perceived = pmap.hazard(0.5) & region

    print(f"\n=== COVERAGE ===")
    print(f"  region cells            {int(region.sum())}")
    print(f"  observed (known)        {int(known.sum())}  "
          f"({100*known.sum()/region.sum():.1f}%)")
    print(f"  UNKNOWN                 {int(unknown.sum())}  "
          f"({100*unknown.sum()/region.sum():.1f}%)  <- not safe, blocked for planning")
    print(f"  mean observations/cell  {pmap.count[known].mean():.1f}   "
          f"max {int(pmap.count.max())}")

    fp_x = 2 * (ALT - float(np.nanmedian(hmap))) * np.tan(np.radians(FOV)/2)
    leg_spacing = REGION / (N_LEGS - 1)
    print(f"\n  HAND CHECK -- coverage arithmetic")
    print(f"    cross-leg footprint  ~{fp_x:.1f} m   leg spacing {leg_spacing:.1f} m")
    print(f"    -> {N_LEGS} legs sweep {(N_LEGS-1)*leg_spacing + fp_x:.1f} m of a "
          f"{REGION:.0f} m region")
    print(f"    predicted coverage   {min(100.0, 100*((N_LEGS-1)*leg_spacing + fp_x)/REGION):.0f}%"
          f"   measured {100*known.sum()/region.sum():.0f}%")
    print(f"    (measured is lower: each frame loses a {pp.neighbourhood_m/2:.1f} m "
          f"border to the top-hat erosion from checkpoint 2)")

    C = confusion(pmap.prob, truth, known, 0.5)
    print(f"\n=== AGREEMENT over {C['n_observed']} OBSERVED cells (truth space) ===")
    print(f"  {'':14}{'pred SAFE':>12}{'pred HAZARD':>13}")
    print(f"  {'true SAFE':14}{C['tn']:12d}{C['fp']:13d}")
    print(f"  {'true HAZARD':14}{C['fn']:12d}{C['tp']:13d}")
    print(f"\n  precision {C['precision']:.3f}  recall {C['recall']:.3f}  "
          f"IoU {C['iou']:.3f}")
    print(f"  MISS rate        {C['miss_rate']:.3f}  ({C['fn']} real hazard cells "
          f"mapped safe)   <-- dangerous")
    print(f"  FALSE-ALARM rate {C['false_alarm_rate']:.3f}  ({C['fp']} safe cells "
          f"mapped hazard)   <-- costs range")

    disp = boundary_displacement(perceived, truth & region, known, CELL)
    print(f"\n=== SMEAR FROM ESTIMATED-POSE PROJECTION ===")
    print(f"  ESKF 3D error over the survey  mean {eskf_err.mean():.3f}  "
          f"max {eskf_err.max():.3f} m")
    print(f"  horizontal (= footprint shift) mean {smear.mean():.3f}  "
          f"max {smear.max():.3f} m  ({smear.mean()/CELL:.2f} cells)")
    print(f"  body tilt during captures      mean {tilts.mean():.2f}  "
          f"max {tilts.max():.2f} deg")
    print(f"    -> unmodelled attitude term  {ALT*np.tan(np.radians(tilts.mean())):.3f} m "
          f"mean ({ALT*np.tan(np.radians(tilts.max())):.3f} m worst)")
    print(f"  hazard EDGE displacement:")
    for kk, vv in disp.items():
        print(f"    {kk:32s} {vv:.3f} m")
    print(f"  hand-check: edge displacement should sit near the horizontal smear")
    ratio = disp.get("true_to_perceived_mean_m", 0) / max(smear.mean(), 1e-9)
    print(f"    true->perceived {disp.get('true_to_perceived_mean_m',0):.3f} m vs "
          f"smear {smear.mean():.3f} m  -> ratio {ratio:.2f}")

    np.savez("b4_perceived_map.npz", logodds=pmap.logodds, count=pmap.count,
             prob=np.where(pmap.known, pmap.prob, np.nan),
             height=pmap.height, known=pmap.known, region=region,
             cell=CELL, origin=ORIGIN)
    print(f"\n  saved b4_perceived_map.npz  (prob + height + known mask)")

    ext = [-EXTENT/2, EXTENT/2, -EXTENT/2, EXTENT/2]
    fig = plt.figure(figsize=(17, 9))
    gs = fig.add_gridspec(2, 3)

    a = fig.add_subplot(gs[0, 0])
    show = np.where(pmap.known, pmap.prob, np.nan)
    im = a.imshow(show.T, origin="lower", extent=ext, cmap="inferno", vmin=0, vmax=1)
    a.imshow(np.transpose(np.stack([np.zeros((n,n))]*3 + [(unknown*0.55)], -1),
                          (1,0,2)), origin="lower", extent=ext)
    a.add_patch(plt.Rectangle((REGION_C[0]-REGION/2, REGION_C[1]-REGION/2),
                              REGION, REGION, fill=False, color="cyan", lw=1.6))
    a.set_title("PERCEIVED hazard map (built by the drone)\ngrey = UNKNOWN")
    plt.colorbar(im, ax=a, label="P(hazard)")

    a = fig.add_subplot(gs[0, 1])
    a.imshow(np.where(np.isfinite(tm_true.cost), tm_true.cost, np.nan).T,
             origin="lower", extent=ext, cmap="viridis")
    blk = np.zeros((n, n, 4)); blk[truth] = [0.85, .06, .06, 1]
    a.imshow(np.transpose(blk, (1,0,2)), origin="lower", extent=ext)
    a.add_patch(plt.Rectangle((REGION_C[0]-REGION/2, REGION_C[1]-REGION/2),
                              REGION, REGION, fill=False, color="cyan", lw=1.6))
    a.set_title("B1 GROUND-TRUTH cost map (red = hazard)")

    a = fig.add_subplot(gs[0, 2])
    err = np.ones((n, n, 3))
    err[known & truth & perceived] = [0.1, 0.6, 0.1]
    err[known & truth & ~perceived] = [0.9, 0.0, 0.0]
    err[known & ~truth & perceived] = [1.0, 0.75, 0.0]
    err[unknown] = [0.55, 0.55, 0.55]
    err[~region] = [1, 1, 1]
    a.imshow(np.transpose(err, (1,0,2)), origin="lower", extent=ext)
    a.set_title(f"green=hit RED=MISS amber=false alarm grey=UNKNOWN\n"
                f"miss {C['miss_rate']:.3f}  false alarm {C['false_alarm_rate']:.3f}")

    a = fig.add_subplot(gs[1, 0])
    im = a.imshow(np.where(pmap.known, pmap.count, np.nan).T, origin="lower",
                  extent=ext, cmap="magma")
    a.plot(path[:, 0], path[:, 1], "-", color="cyan", lw=1.4, label="survey track")
    a.legend(fontsize=8)
    a.set_title("observations per cell (overlap)")
    plt.colorbar(im, ax=a)

    a = fig.add_subplot(gs[1, 1])
    a.imshow(np.where(pmap.known, pmap.height, np.nan).T, origin="lower",
             extent=ext, cmap="terrain")
    a.set_title("PERCEIVED heightfield (feeds TraversabilityMap in step 4)")

    a = fig.add_subplot(gs[1, 2])
    a.plot(eskf_err, color="crimson", label="ESKF 3D error")
    a.plot(smear, color="darkorange", label="horizontal smear")
    a.axhline(CELL, color="k", ls=":", label=f"1 cell = {CELL:.2f} m")
    a.set_xlabel("frame"); a.set_ylabel("[m]"); a.legend(fontsize=8); a.grid(alpha=.3)
    a.set_title("localization error during the survey")

    fig.suptitle("B4 Checkpoint 3 — perceived world hazard map from a survey flight",
                 fontsize=13)
    fig.tight_layout()
    fig.savefig("b4_step3_mapping.png", dpi=110)
    print("  plot  b4_step3_mapping.png")

    with open("b4_step3_mapping.json", "w") as fh:
        json.dump({"frames": n_frames, "survey_s": float(sim.sim_time),
                   "coverage": {"region_cells": int(region.sum()),
                                "known": int(known.sum()),
                                "unknown": int(unknown.sum()),
                                "pct": float(100*known.sum()/region.sum())},
                   "agreement": C, "smear": {
                       "eskf_mean_m": float(eskf_err.mean()),
                       "eskf_max_m": float(eskf_err.max()),
                       "horiz_mean_m": float(smear.mean()),
                       "horiz_max_m": float(smear.max()),
                       "tilt_mean_deg": float(tilts.mean()),
                       **disp},
                   "map": pmap.summary(), "rejected_frames": skipped}, fh, indent=1, default=float)
    print("  log   b4_step3_mapping.json")


if __name__ == "__main__":
    main()
