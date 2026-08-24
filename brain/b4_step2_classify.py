"""B4 CHECKPOINT 2 -- per-patch hazard classification, scored against truth."""
import json

import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

import camera_rig
from camera_rig import DownwardCamera
from skyscout_sim import SkyScoutSim
from mars_world import build_heightmap, EXTENT, CELL, CRATER_CENTRE, CRATER_RADIUS
from cost_map import TraversabilityMap, CostParams
from hazard_classify import PerceptParams, score_frame, confusion

RES, FOV, ALT = (320, 240), 60.0, 12.0
ORIGIN = (-EXTENT / 2, -EXTENT / 2)

SITES = [
    ("crater bowl",   (-15.0, 10.0)),
    ("crater rim W",  (-26.0, 10.0)),
    ("crater rim S",  (-15.0, -1.0)),
    ("boulder A",     (12.7, 14.2)),
    ("ridge toe",     (2.0, 6.0)),
    ("open ground SW", (-30.0, -26.0)),
    ("open ground SE", (18.0, -28.0)),
    ("open ground S", (-6.0, -20.0)),
]


def main():
    hmap, feats, rocks = build_heightmap()
    tm = TraversabilityMap(hmap, CELL, ORIGIN, CostParams())
    truth = tm.blocked_raw
    n = tm.n
    pp = PerceptParams()

    print("=== B4 CHECKPOINT 2 -- hazard classification (depth + GLCM-on-depth) ===")
    print(f"  grid {n}x{n} @ {CELL:.3f} m   fine raster {CELL/pp.fine_div:.3f} m")
    print(f"  true hazard cells (un-inflated): {truth.sum()} "
          f"({100*truth.mean():.2f}% of map)")
    print(f"  fusion: noisy-OR  slope^{pp.w_slope} prom^{pp.w_prom} "
          f"rough^{pp.w_rough}   threshold {pp.threshold}")

    sim = SkyScoutSim(start_pos=(SITES[0][1][0], SITES[0][1][1], ALT),
                      logging_level="error", camera_res=RES, camera_fov=FOV)
    rig = DownwardCamera(sim, RES, FOV)
    rig.warm_up()

    acc_score = np.full((n, n), np.nan)
    acc_obs = np.zeros((n, n), bool)
    per_site, frames = [], []

    print(f"\n  {'site':16} {'obs cells':>10} {'true haz':>9} {'recall':>8} "
          f"{'precis':>8} {'miss':>7} {'falarm':>8}")
    for name, (x, y) in SITES:
        pos = np.array([x, y, ALT])
        sim.drone.set_pos(pos)
        depth, _, cam_pos, lookat = rig.capture(pos)
        if rig.self_view_pixels(depth):
            raise SystemExit(f"ABORT: airframe in frame at {name}")
        pts = rig.project(depth, pos)
        r = score_frame(pts, CELL, ORIGIN, n, pp)

        c = confusion(r["score"], truth, r["observed"], pp.threshold)
        per_site.append({"site": name, "xy": [x, y], **c})
        print(f"  {name:16} {c['n_observed']:10d} {c['n_true_hazard']:9d} "
              f"{c['recall']:8.3f} {c['precision']:8.3f} "
              f"{c['miss_rate']:7.3f} {c['false_alarm_rate']:8.3f}")

        m = r["observed"] & np.isfinite(r["score"])
        upd = m & (~acc_obs | (r["score"] > np.nan_to_num(acc_score, nan=-1)))
        acc_score[upd] = r["score"][upd]
        acc_obs |= m
        frames.append({"name": name, "xy": (x, y), "r": r})

    C = confusion(acc_score, truth, acc_obs, pp.threshold)
    print(f"\n=== POOLED over {C['n_observed']} observed cells "
          f"({100*acc_obs.mean():.1f}% of the map) ===")
    print(f"  {'':14}{'pred SAFE':>12}{'pred HAZARD':>13}")
    print(f"  {'true SAFE':14}{C['tn']:12d}{C['fp']:13d}")
    print(f"  {'true HAZARD':14}{C['fn']:12d}{C['tp']:13d}")
    print(f"\n  precision {C['precision']:.3f}   recall {C['recall']:.3f}   "
          f"F1 {C['f1']:.3f}   IoU {C['iou']:.3f}")
    print(f"  MISS rate        {C['miss_rate']:.3f}  "
          f"({C['fn']} real hazard cells called safe)   <-- the dangerous one")
    print(f"  FALSE-ALARM rate {C['false_alarm_rate']:.3f}  "
          f"({C['fp']} safe cells called hazard)        <-- costs range only")

    print(f"\n=== THRESHOLD SWEEP (miss vs false alarm) ===")
    print(f"  {'thr':>6} {'recall':>8} {'precis':>8} {'miss':>8} {'falarm':>8} {'IoU':>7}")
    sweep = []
    for t in [0.2, 0.3, 0.4, 0.5, 0.6, 0.7, 0.8, 0.9, 0.95, 0.99, 0.999, 1.0]:
        c = confusion(acc_score, truth, acc_obs, t)
        sweep.append({"threshold": float(t), **c})
        print(f"  {t:6.2f} {c['recall']:8.3f} {c['precision']:8.3f} "
              f"{c['miss_rate']:8.3f} {c['false_alarm_rate']:8.3f} {c['iou']:7.3f}")

    print(f"\n=== ABLATION -- does GLCM-on-depth add anything over geometry? ===")
    abl = {}
    for tag, kw in (("geometry only (slope+prom)", dict(w_rough=0.0)),
                    ("roughness only (GLCM)", dict(w_slope=0.0, w_prom=0.0)),
                    ("fused (slope+prom+GLCM)", dict())):
        p2 = PerceptParams(**{**pp.__dict__, **kw})
        s = np.full((n, n), np.nan); o = np.zeros((n, n), bool)
        for f in frames:
            pos = np.array([f["xy"][0], f["xy"][1], ALT])
            sim.drone.set_pos(pos)
            depth, _, cp, la = rig.capture(pos)
            rr = score_frame(rig.project(depth, pos), CELL, ORIGIN, n, p2)
            m = rr["observed"] & np.isfinite(rr["score"])
            upd = m & (~o | (rr["score"] > np.nan_to_num(s, nan=-1)))
            s[upd] = rr["score"][upd]; o |= m
        c = confusion(s, truth, o, p2.threshold)
        abl[tag] = c
        print(f"  {tag:28} recall {c['recall']:.3f}  precision {c['precision']:.3f}  "
              f"miss {c['miss_rate']:.3f}  IoU {c['iou']:.3f}")

    print(f"\n=== HAND CHECK -- does a known boulder score high? ===")
    bx, by, br, bh = max(rocks, key=lambda r: r[3])
    i, j = tm.to_grid(bx, by)
    print(f"  tallest boulder ({bx}, {by})  r {br} m  height {bh} m")
    print(f"    cell ({i},{j})  truth hazard = {bool(truth[i,j])}")
    print(f"    perceived score = {acc_score[i,j]:.3f}  "
          f"-> {'HAZARD' if acc_score[i,j] >= pp.threshold else 'SAFE'}")
    print(f"    sensed slope {frames[3]['r']['slope_deg'][i,j]:.1f} deg   "
          f"prominence {frames[3]['r']['prominence_m'][i,j]:.2f} m   "
          f"GLCM contrast {frames[3]['r']['roughness'][i,j]:.2f}")
    print(f"    truth: slope {tm.slope_deg[i,j]:.1f} deg  "
          f"prominence {tm.prominence[i,j]:.2f} m")
    ok_b = acc_score[i, j] >= pp.threshold
    ox, oy = SITES[-3][1]
    oi, oj = tm.to_grid(ox, oy)
    print(f"  open ground ({ox}, {oy}) cell ({oi},{oj})  truth hazard "
          f"{bool(truth[oi,oj])}  perceived {acc_score[oi,oj]:.3f} "
          f"-> {'HAZARD' if acc_score[oi,oj] >= pp.threshold else 'SAFE'}")
    ok_o = acc_score[oi, oj] < pp.threshold
    print(f"  VERDICT: boulder flagged = {ok_b}, open ground clear = {ok_o} "
          f"-> {'PASS' if ok_b and ok_o else 'FAIL'}")

    ext = [-EXTENT/2, EXTENT/2, -EXTENT/2, EXTENT/2]
    fig = plt.figure(figsize=(17, 9))
    gs = fig.add_gridspec(2, 3)

    a = fig.add_subplot(gs[0, 0])
    show = np.where(acc_obs, acc_score, np.nan)
    im = a.imshow(show.T, origin="lower", extent=ext, cmap="inferno", vmin=0, vmax=1)
    a.set_title("PERCEIVED hazard score (max over captures)")
    plt.colorbar(im, ax=a, label="score")
    for x, y, r, hh in rocks:
        a.add_patch(plt.Circle((x, y), r, fill=False, color="cyan", lw=0.9))
    a.add_patch(plt.Circle(CRATER_CENTRE, CRATER_RADIUS, fill=False,
                           color="cyan", lw=1.2, ls="--"))

    a = fig.add_subplot(gs[0, 1])
    a.imshow(truth.T, origin="lower", extent=ext, cmap="Reds")
    a.set_title("TRUE hazard (un-inflated)")
    for x, y, r, hh in rocks:
        a.add_patch(plt.Circle((x, y), r, fill=False, color="navy", lw=0.9))

    a = fig.add_subplot(gs[0, 2])
    err = np.full((n, n, 3), 1.0)
    pred = acc_obs & np.isfinite(acc_score) & (acc_score >= pp.threshold)
    err[acc_obs & truth & pred] = [0.1, 0.6, 0.1]
    err[acc_obs & truth & ~pred] = [0.9, 0.0, 0.0]
    err[acc_obs & ~truth & pred] = [1.0, 0.75, 0.0]
    err[~acc_obs] = [0.85, 0.85, 0.85]
    a.imshow(np.transpose(err, (1, 0, 2)), origin="lower", extent=ext)
    a.set_title(f"green=hit  RED=MISS  amber=false alarm  grey=unseen\n"
                f"miss {C['miss_rate']:.3f}   false alarm {C['false_alarm_rate']:.3f}")

    a = fig.add_subplot(gs[1, 0])
    s = np.array([x["threshold"] for x in sweep])
    a.plot(s, [x["miss_rate"] for x in sweep], "o-", color="crimson",
           label="miss rate (dangerous)")
    a.plot(s, [x["false_alarm_rate"] for x in sweep], "s-", color="darkorange",
           label="false-alarm rate")
    a.plot(s, [x["iou"] for x in sweep], "^--", color="navy", label="IoU")
    a.axvline(pp.threshold, color="k", ls=":", label=f"chosen {pp.threshold}")
    a.set_xlabel("decision threshold"); a.legend(fontsize=8); a.grid(alpha=0.3)
    a.set_title("the two errors trade off")

    a = fig.add_subplot(gs[1, 1])
    names = list(abl.keys())
    xs = np.arange(len(names))
    a.bar(xs - 0.2, [abl[k]["recall"] for k in names], 0.4, label="recall")
    a.bar(xs + 0.2, [abl[k]["precision"] for k in names], 0.4, label="precision")
    a.set_xticks(xs); a.set_xticklabels(["geom", "GLCM", "fused"], fontsize=9)
    a.legend(fontsize=8); a.grid(alpha=0.3, axis="y")
    a.set_title("ablation: does GLCM earn its place?")

    a = fig.add_subplot(gs[1, 2])
    f = frames[3]["r"]
    m = f["observed"]
    a.scatter(np.where(m, f["prominence_m"], np.nan).ravel(),
              np.where(m, f["roughness"], np.nan).ravel(),
              c=np.where(m, truth, np.nan).ravel(), cmap="coolwarm", s=3, alpha=.6)
    a.set_xlabel("sensed prominence [m]"); a.set_ylabel("GLCM contrast")
    a.set_title("feature space over the boulder frame\n(red = true hazard)")
    a.grid(alpha=0.3)

    fig.suptitle("B4 Checkpoint 2 — depth + GLCM-on-depth hazard classifier "
                 "(static captures, TRUE pose)", fontsize=13)
    fig.tight_layout()
    fig.savefig("b4_step2_classify.png", dpi=110)
    print("\n  plot  b4_step2_classify.png")

    with open("b4_step2_classify.json", "w") as fh:
        json.dump({"params": pp.__dict__, "pooled": C, "per_site": per_site,
                   "sweep": sweep,
                   "ablation": {k: v for k, v in abl.items()},
                   "handcheck": {"boulder_flagged": bool(ok_b),
                                 "open_ground_clear": bool(ok_o)}},
                  fh, indent=1, default=float)
    print("  log   b4_step2_classify.json")


if __name__ == "__main__":
    main()
