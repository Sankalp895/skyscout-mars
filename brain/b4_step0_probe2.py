"""B4 Checkpoint 0, part 2 -- does DEPTH resolve a real hazard, and what does surveying cost at."""
import json

import numpy as np
from PIL import Image

from skyscout_sim import SkyScoutSim
from mars_world import build_heightmap, EXTENT, CELL

FOV = 60.0
RES = (320, 240)
ORIGIN = (-EXTENT / 2, -EXTENT / 2)


def main():
    h, feats, rocks = build_heightmap()
    rocks = sorted(rocks, key=lambda r: -r[3])
    bx, by, br, bh = (float(v) for v in rocks[0])
    print("=== B4 CHECKPOINT 0 (part 2) ===")
    print(f"  test hazard: boulder at ({bx:.1f}, {by:.1f})  r {br:.2f} m  "
          f"height {bh:.2f} m")

    alt = 12.0
    sim = SkyScoutSim(start_pos=(bx, by, alt), logging_level="error",
                      camera_res=RES, camera_fov=FOV)
    cam = sim.camera
    cam.set_pose(pos=(bx, by, alt), lookat=(bx, by, 0.0), up=(1, 0, 0))
    cam.render(rgb=True)

    d = np.asarray(cam.render(rgb=False, depth=True)[1], float)
    if d.ndim == 3:
        d = d[..., 0]
    z = alt - d

    fp = 2 * alt * np.tan(np.radians(FOV) / 2)
    print(f"\n  altitude {alt} m, fov {FOV} deg -> footprint {fp:.2f} m")
    print(f"  depth image {d.shape}, range [{d.min():.3f}, {d.max():.3f}] m")
    print(f"  implied terrain height range [{z.min():.3f}, {z.max():.3f}] m")

    cy, cx = d.shape[0] // 2, d.shape[1] // 2
    sensed = float(np.median(z[cy-2:cy+3, cx-2:cx+3]))
    true_h = sim.terrain_height(bx, by)
    print(f"\n  --- HAND CHECK: sensed vs true terrain height at boulder peak ---")
    print(f"    sensed {sensed:.3f} m   true {true_h:.3f} m   "
          f"error {abs(sensed-true_h):.4f} m")

    yy, xx = np.mgrid[0:z.shape[0], 0:z.shape[1]]
    rpx = np.hypot(yy - cy, xx - cx)
    m_per_px = fp / max(z.shape)
    core = rpx * m_per_px < br * 0.6
    ring = (rpx * m_per_px > br * 1.8) & (rpx * m_per_px < br * 3.0)
    prom = float(np.median(z[core]) - np.median(z[ring]))
    print(f"    SENSED prominence (peak - surround) {prom:.3f} m  "
          f"vs true boulder height {bh:.3f} m")
    print(f"    -> depth {'RESOLVES' if prom > 0.4 * bh else 'FAILS TO RESOLVE'} "
          f"this hazard")

    v = (z - z.min()) / max(np.ptp(z), 1e-9)
    Image.fromarray((v * 255).astype(np.uint8)).save("b4_probe_boulder_depth.png")
    rgb = np.asarray(cam.render(rgb=True)[0], np.uint8)[..., :3]
    Image.fromarray(rgb).save("b4_probe_boulder_rgb.png")
    print("    wrote b4_probe_boulder_depth.png / _rgb.png")

    print(f"\n  --- SURVEY COST vs ALTITUDE (60 deg fov, 30% sidelap) ---")
    print(f"  {'alt':>6} {'footprint':>10} {'cm/px':>7} {'cells/frame':>12} "
          f"{'frames for 40x40 m':>20} {'lawnmower legs':>15}")
    region = 40.0
    survey = {}
    for a in (3.38, 8.0, 12.0, 20.0, 30.0):
        f = 2 * a * np.tan(np.radians(FOV) / 2)
        step = f * 0.7
        legs = int(np.ceil(region / step))
        frames = legs * int(np.ceil(region / step))
        print(f"  {a:6.2f} {f:9.2f}m {f/RES[0]*100:6.1f} "
              f"{(f/CELL)**2:11.0f} {frames:20d} {legs:15d}")
        survey[a] = {"footprint_m": f, "frames": frames, "legs": legs}

    with open("b4_step0_probe2.json", "w") as f_:
        json.dump({"boulder": {"x": bx, "y": by, "r": br, "h": bh},
                   "sensed_height_m": sensed, "true_height_m": true_h,
                   "sensed_prominence_m": prom,
                   "survey": {str(k): v for k, v in survey.items()}}, f_, indent=1)
    print("\n  wrote b4_step0_probe2.json")


if __name__ == "__main__":
    main()
