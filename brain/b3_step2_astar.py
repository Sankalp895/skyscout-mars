"""B3 CHECKPOINT 2 -- A* planning."""
import json

import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.colors import LinearSegmentedColormap

from mars_world import build_heightmap, EXTENT, CELL, CRATER_CENTRE, CRATER_RADIUS
from cost_map import TraversabilityMap, CostParams
from astar import plan, straight_line_blocked

ORIGIN = (-EXTENT / 2, -EXTENT / 2)

PAIRS = [
    ("A: open ground, clear shot",  (-30.0, -28.0), (22.0, -30.0)),
    ("B: around the RIDGE toe",     (-35.0, -5.0),  (10.0, -35.0)),
    ("C: past the CRATER",          (-34.0, 24.0),  (-2.0, -6.0)),
    ("D: SW -> NE (severed)",       (-32.0, -22.0), (30.0, 26.0)),
]


def main():
    h, feats, rocks = build_heightmap()
    tm = TraversabilityMap(h, CELL, ORIGIN, CostParams())

    results = []
    print("=== A* PLANNING ===")
    for name, s, gxy in PAIRS:
        nhit, frac = straight_line_blocked(tm, s, gxy)
        r = plan(tm, s, gxy)
        if not r["ok"]:
            print(f"\n  {name}: NO PATH -- {r['reason']}")
            results.append({"name": name, "ok": False, "reason": r["reason"]})
            continue

        touched = sum(1 for i, j in r["cells"] if tm.blocked[i, j])
        print(f"\n  {name}")
        print(f"    start {s} -> goal {gxy}")
        print(f"    straight line crosses blocked cells: {nhit}/400 samples "
              f"({100*frac:.0f}%)  -> {'DETOUR REQUIRED' if nhit else 'clear shot'}")
        print(f"    path length {r['length_m']:6.2f} m   straight "
              f"{r['straight_m']:6.2f} m   DETOUR RATIO {r['detour_ratio']:.3f}x")
        print(f"    cells expanded {r['expanded']:6d}   total cost {r['total_cost']:8.2f}")
        print(f"    blocked cells entered: {touched}  "
              f"{'PASS' if touched == 0 else 'FAIL'}")
        print(f"    min clearance to any hazard along path: "
              f"{r['min_clearance_m']:.2f} m")
        results.append({
            "name": name, "ok": True, "start": s, "goal": gxy,
            "straight_blocked_frac": float(frac),
            "length_m": r["length_m"], "straight_m": r["straight_m"],
            "detour_ratio": r["detour_ratio"], "expanded": r["expanded"],
            "blocked_entered": touched,
            "min_clearance_m": r["min_clearance_m"],
            "world": r["world"].tolist(),
        })

    print("\n=== DETOUR SANITY ===")
    for res in results:
        if not res.get("ok"):
            continue
        dr, blk = res["detour_ratio"], res["straight_blocked_frac"]
        if blk == 0:
            verdict = "clear shot -- ratio should be ~1.0" if dr < 1.15 else \
                      "UNEXPECTED detour on a clear line"
        elif dr < 1.05:
            verdict = "SUSPECT: straight line was blocked but path barely deviates"
        elif dr > 2.0:
            verdict = "SUSPECT: very long detour -- map may be over-blocked"
        else:
            verdict = "sane detour"
        print(f"  {res['name']:30s} {dr:.3f}x  -> {verdict}")

    ext = [-EXTENT / 2, EXTENT / 2, -EXTENT / 2, EXTENT / 2]
    cmap = LinearSegmentedColormap.from_list(
        "trav", ["#12341f", "#2e7d32", "#c9b037", "#e65100"])
    cost_show = np.where(np.isfinite(tm.cost), tm.cost, np.nan)

    fig, axes = plt.subplots(1, 4, figsize=(24, 6.6))
    for ax, res in zip(axes, results):
        ax.imshow(cost_show.T, origin="lower", extent=ext, cmap=cmap,
                  vmin=1.0, vmax=np.nanpercentile(cost_show, 99))
        blk = np.zeros((*tm.blocked.shape, 4))
        blk[tm.blocked] = [0.85, 0.06, 0.06, 1.0]
        ax.imshow(np.transpose(blk, (1, 0, 2)), origin="lower", extent=ext)
        ax.add_patch(plt.Circle(CRATER_CENTRE, CRATER_RADIUS, fill=False,
                                color="cyan", lw=1.4, alpha=0.8))
        if not res.get("ok"):
            ax.set_title(f"{res['name']}\nNO PATH")
            continue
        s, gxy = res["start"], res["goal"]
        w = np.array(res["world"])
        ax.plot([s[0], gxy[0]], [s[1], gxy[1]], ls="--", color="white", lw=1.4,
                alpha=0.9, label="straight line")
        ax.plot(w[:, 0], w[:, 1], color="#00e5ff", lw=2.6, label="A* path")
        ax.plot(*s, "o", color="lime", ms=11, mec="k", label="start")
        ax.plot(*gxy, "*", color="magenta", ms=18, mec="k", label="goal")
        ax.set_title(f"{res['name']}\ndetour {res['detour_ratio']:.2f}x  "
                     f"({res['length_m']:.1f} m vs {res['straight_m']:.1f} m)")
        ax.legend(loc="lower left", fontsize=8)
        ax.set_xlabel("x [m]"); ax.set_ylabel("y [m]")
    fig.suptitle("B3 step 2 — A* over the traversability map "
                 "(red = blocked + 2-cell inflation)", fontsize=13)
    fig.tight_layout()
    fig.savefig("b3_step2_astar.png", dpi=110)
    print("\n  plot  b3_step2_astar.png")

    with open("b3_step2_astar.json", "w") as f:
        json.dump({"cost_params": tm.p.__dict__, "results": results}, f, indent=1)
    print("  log   b3_step2_astar.json")


if __name__ == "__main__":
    main()
