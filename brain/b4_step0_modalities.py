"""B4 CHECKPOINT 0 -- probe what a Genesis camera actually gives us headless."""
import json
import time

import numpy as np
from PIL import Image

from skyscout_sim import SkyScoutSim
from mars_world import EXTENT, CELL

RES = (320, 240)
FOV = 60.0
ALT = 12.0
PROBE_XY = (-15.0, 10.0)


def try_channel(cam, name, **kw):
    """Attempt one render channel; report shape/dtype/range or the exact error."""
    try:
        t0 = time.time()
        out = cam.render(**kw)
        dt = time.time() - t0
    except Exception as e:
        return {"available": False, "error": f"{type(e).__name__}: {e}"}
    arrs = [a for a in (out if isinstance(out, (tuple, list)) else [out])
            if a is not None]
    if not arrs:
        return {"available": False, "error": "channel returned None"}
    a = np.asarray(arrs[-1] if len(arrs) > 1 else arrs[0])
    for cand in arrs:
        c = np.asarray(cand)
        if not (c.ndim == 3 and c.shape[-1] in (3, 4)) or name == "rgb":
            a = c
            break
    return {"available": True, "seconds": round(dt, 3), "shape": list(a.shape),
            "dtype": str(a.dtype),
            "min": float(np.nanmin(a)), "max": float(np.nanmax(a)),
            "n_unique": int(np.unique(a).size) if a.dtype.kind in "iu" else None,
            "_array": a}


def main():
    report = {}
    print("=== B4 CHECKPOINT 0 -- Genesis camera modality probe ===")
    print(f"  res {RES}  fov {FOV} deg  altitude {ALT} m over {PROBE_XY}")

    sim = SkyScoutSim(start_pos=(PROBE_XY[0], PROBE_XY[1], ALT),
                      logging_level="error", camera_res=RES, camera_fov=FOV)
    cam = sim.camera
    cam.set_pose(pos=(PROBE_XY[0], PROBE_XY[1], ALT),
                 lookat=(PROBE_XY[0], PROBE_XY[1], 0.0), up=(1, 0, 0))

    print("\n  warming shader cache (the ~86 s cold cost) ...")
    t0 = time.time()
    cam.render(rgb=True)
    warm = time.time() - t0
    print(f"  warm-up: {warm:.1f} s")
    report["warmup_s"] = round(warm, 1)

    print("\n  --- channel probe (all timings post-warm-up) ---")
    channels = {
        "rgb":          dict(rgb=True),
        "depth":        dict(rgb=False, depth=True),
        "segmentation": dict(rgb=False, segmentation=True),
        "normal":       dict(rgb=False, normal=True),
    }
    for name, kw in channels.items():
        r = try_channel(cam, name, **kw)
        arr = r.pop("_array", None)
        report[name] = r
        if not r["available"]:
            print(f"  {name:14s} UNAVAILABLE -- {r['error']}")
            continue
        extra = f"  unique={r['n_unique']}" if r["n_unique"] is not None else ""
        print(f"  {name:14s} {r['seconds']:6.3f} s  shape {r['shape']}  "
              f"{r['dtype']}  range [{r['min']:.3f}, {r['max']:.3f}]{extra}")
        if arr is not None:
            np.save(f"b4_probe_{name}.npy", arr)

    print("\n  --- HAND CHECK: depth vs known terrain ---")
    d = report.get("depth", {})
    if d.get("available"):
        arr = np.load("b4_probe_depth.npy")
        a = np.asarray(arr, float)
        if a.ndim == 3:
            a = a[..., 0]
        centre = float(np.nanmedian(a[a.shape[0]//2-3:a.shape[0]//2+3,
                                      a.shape[1]//2-3:a.shape[1]//2+3]))
        gnd = sim.terrain_height(*PROBE_XY)
        expect = ALT - gnd
        print(f"    centre-pixel depth      {centre:.3f}")
        print(f"    expected range to ground {expect:.3f} m "
              f"(alt {ALT} - terrain {gnd:.3f})")
        err = abs(centre - expect)
        print(f"    error {err:.3f} m -> "
              f"{'METRIC DEPTH CONFIRMED' if err < 0.5 else 'NOT metric / different units or convention'}")
        report["depth_handcheck"] = {"centre": centre, "expected_m": expect,
                                     "error_m": err, "terrain_m": gnd}

    print("\n  --- HAND CHECK: is segmentation useful for hazard detection? ---")
    s = report.get("segmentation", {})
    if s.get("available"):
        seg = np.load("b4_probe_segmentation.npy")
        ids, counts = np.unique(seg, return_counts=True)
        print(f"    distinct IDs in a downward view of crater+boulders: {len(ids)}")
        print(f"    IDs {ids.tolist()[:10]}  pixel share "
              f"{(counts / counts.sum()).round(3).tolist()[:10]}")
        print("    NOTE: mars_world bakes crater, ridge and all 12 boulders into")
        print("    the HEIGHTFIELD. They are geometry, not separate bodies, so a")
        print("    per-object mask cannot separate a boulder from flat regolith.")
        report["segmentation_useful_for_hazards"] = bool(len(ids) > 2)
    else:
        print("    segmentation unavailable -- moot.")

    print("\n  --- HAND CHECK: ground footprint at altitude ---")
    for h in (3.38, ALT, 20.0):
        fp = 2 * h * np.tan(np.radians(FOV) / 2)
        print(f"    alt {h:5.2f} m -> footprint {fp:6.2f} m  "
              f"({fp / CELL:5.1f} cost-map cells, {fp/RES[0]*100:.1f} cm/px)")
    report["footprint"] = {"fov_deg": FOV,
                           "cruise_3.38m_m": 2*3.38*np.tan(np.radians(FOV)/2),
                           "probe_12m_m": 2*ALT*np.tan(np.radians(FOV)/2),
                           "cell_m": CELL}

    if report.get("rgb", {}).get("available"):
        rgb = np.load("b4_probe_rgb.npy")
        Image.fromarray(np.asarray(rgb, np.uint8)[..., :3]).save("b4_probe_rgb.png")
        print("\n  wrote b4_probe_rgb.png")
    if d.get("available"):
        a = np.load("b4_probe_depth.npy").astype(float)
        if a.ndim == 3:
            a = a[..., 0]
        fin = np.isfinite(a) & (a < 1e6)
        v = (a - a[fin].min()) / max(np.ptp(a[fin]), 1e-9)
        Image.fromarray((np.clip(v, 0, 1) * 255).astype(np.uint8)).save("b4_probe_depth.png")
        print("  wrote b4_probe_depth.png")

    report.pop("_array", None)
    with open("b4_step0_modalities.json", "w") as f:
        json.dump({k: v for k, v in report.items()}, f, indent=1, default=str)
    print("  wrote b4_step0_modalities.json")


if __name__ == "__main__":
    main()
