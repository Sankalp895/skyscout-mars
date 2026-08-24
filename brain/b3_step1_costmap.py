"""B3 CHECKPOINT 1 -- traversability cost map."""
import json

import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.colors import LinearSegmentedColormap

from mars_world import (build_heightmap, EXTENT, CELL, N,
                        CRATER_CENTRE, CRATER_RADIUS)
from cost_map import TraversabilityMap, CostParams

ORIGIN = (-EXTENT / 2, -EXTENT / 2)


def main():
    h, feats, rocks = build_heightmap()
    tm = TraversabilityMap(h, CELL, ORIGIN, CostParams())

    print("=== COST MAP ===")
    for k, v in tm.summary().items():
        print(f"  {k:18s} {v}")

    print("\n=== HAND CHECK: cost at known terrain features ===")
    def probe(x, y):
        i, j = tm.to_grid(x, y)
        return (tm.slope_deg[i, j], tm.prominence[i, j],
                tm.cost[i, j], tm.blocked[i, j])

    cx, cy = CRATER_CENTRE
    samples = [("crater centre (bowl floor)", cx, cy),
               ("crater RIM", cx + CRATER_RADIUS, cy),
               ("ridge crest", 5.0, 15.0)]
    for rx, ry, rr, rh in sorted(rocks, key=lambda r: -r[3])[:3]:
        samples.append((f"boulder h={rh} m", rx, ry))
    open_pts = []
    for x, y in [(25, -25), (-30, -20), (30, 25), (0, -30)]:
        open_pts.append((f"open ground ({x},{y})", x, y))
    samples += open_pts

    print(f"  {'location':28s}{'slope°':>8}{'prom m':>8}{'cost':>9}  blocked")
    rows = []
    for name, x, y in samples:
        s, pr, c, b = probe(x, y)
        cs = "INF" if not np.isfinite(c) else f"{c:.2f}"
        print(f"  {name:28s}{s:8.1f}{pr:8.2f}{cs:>9}  {'YES' if b else 'no'}")
        rows.append({"name": name, "x": x, "y": y, "slope_deg": float(s),
                     "prominence_m": float(pr), "blocked": bool(b),
                     "cost": None if not np.isfinite(c) else float(c)})

    open_costs = [r["cost"] for r in rows if r["name"].startswith("open")]
    feat_blocked = [r["blocked"] for r in rows if not r["name"].startswith("open")]
    open_blocked = [r["blocked"] for r in rows if r["name"].startswith("open")]
    print(f"\n  open ground cost      mean {np.mean(open_costs):.2f} "
          f"(1.0 = perfectly flat)   any blocked: {any(open_blocked)}")
    print(f"  hazard features blocked: {sum(feat_blocked)}/{len(feat_blocked)}")
    print(f"  {'PASS' if (not any(open_blocked)) else 'FAIL'} -- open ground is traversable")

    fig, ax = plt.subplots(2, 2, figsize=(14, 12))

    ext = [-EXTENT / 2, EXTENT / 2, -EXTENT / 2, EXTENT / 2]

    a = ax[0, 0]
    im = a.imshow(h.T, origin="lower", extent=ext, cmap="terrain")
    a.set_title("terrain elevation [m]"); plt.colorbar(im, ax=a, shrink=0.8)

    a = ax[0, 1]
    im = a.imshow(tm.slope_deg.T, origin="lower", extent=ext, cmap="magma")
    a.set_title(f"slope [deg]  (blocked above {tm.p.slope_max_deg})")
    plt.colorbar(im, ax=a, shrink=0.8)

    a = ax[1, 0]
    im = a.imshow(tm.prominence.T, origin="lower", extent=ext, cmap="inferno")
    a.set_title(f"local prominence [m]  (blocked above {tm.p.obs_max_m})")
    plt.colorbar(im, ax=a, shrink=0.8)

    a = ax[1, 1]
    cost_show = np.where(np.isfinite(tm.cost), tm.cost, np.nan)
    cmap = LinearSegmentedColormap.from_list(
        "trav", ["#12341f", "#2e7d32", "#c9b037", "#e65100"])
    im = a.imshow(cost_show.T, origin="lower", extent=ext, cmap=cmap,
                  vmin=1.0, vmax=np.nanpercentile(cost_show, 99))
    blk = np.zeros((*tm.blocked.shape, 4))
    blk[tm.blocked.T if False else tm.blocked] = [0.9, 0.05, 0.05, 1.0]
    a.imshow(np.transpose(blk, (1, 0, 2)), origin="lower", extent=ext)
    a.set_title(f"TRAVERSABILITY COST  (red = BLOCKED, "
                f"{tm.summary()['blocked_pct']}% of map)")
    plt.colorbar(im, ax=a, shrink=0.8)

    circ = plt.Circle(CRATER_CENTRE, CRATER_RADIUS, fill=False,
                      color="cyan", lw=2)
    a.add_patch(circ)
    a.annotate("crater rim", CRATER_CENTRE, color="cyan", fontsize=9,
               ha="center")
    for rx, ry, rr, rh in rocks:
        a.plot(rx, ry, "o", mfc="none", mec="white", ms=6, mew=1.2)
    a.plot([], [], "o", mfc="none", mec="white", label="boulders")
    a.annotate("ridge", (2, 12), color="white", fontsize=9, rotation=-37)
    a.legend(loc="lower left", fontsize=8)

    for row in ax:
        for a_ in row:
            a_.set_xlabel("x [m]"); a_.set_ylabel("y [m]")
    fig.suptitle("B3 step 1 — traversability cost map from the B1 heightfield "
                 "(seed 20260808)", fontsize=13)
    fig.tight_layout()
    fig.savefig("b3_step1_costmap.png", dpi=110)
    print("\n  plot  b3_step1_costmap.png")

    with open("b3_step1_costmap.json", "w") as f:
        json.dump({"params": tm.p.__dict__, "summary": tm.summary(),
                   "probes": rows}, f, indent=1)
    print("  log   b3_step1_costmap.json")


if __name__ == "__main__":
    main()
