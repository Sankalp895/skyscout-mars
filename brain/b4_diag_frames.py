"""Why do frames 1-2 project with metres of height error when 0 and 3-7 are at millimetres? Two."""
import numpy as np
import camera_rig
from camera_rig import DownwardCamera
from skyscout_sim import SkyScoutSim, STEPS_PER_CONTROL
from mars_world import build_heightmap, EXTENT
from b3_flight import tm_height

camera_rig.DEPTH_IS_EUCLIDEAN = False
camera_rig.FOV_IS_VERTICAL = True

RES, FOV, ALT = (320, 240), 60.0, 12.0
hmap, _, _ = build_heightmap()

sim = SkyScoutSim(start_pos=(-24.0, 6.0, ALT), logging_level="error",
                  camera_res=RES, camera_fov=FOV)
rig = DownwardCamera(sim, RES, FOV)
rig.warm_up()

print(f"{'i':>3} {'cmd z':>8} {'true z':>8} {'depth min':>10} {'depth max':>10} "
      f"{'n>1e3':>7} {'n nan':>6} {'RMS':>10}")
for i in range(8):
    x = -24.0 + i * 2.5
    pos = np.array([x, 6.0, ALT])
    depth, _, cam_pos, lookat = rig.capture(pos)
    d = np.asarray(depth, float)
    bad_far = int((d > 1e3).sum())
    bad_nan = int((~np.isfinite(d)).sum())
    pts = rig.project(d, cam_pos, lookat).reshape(-1, 3)
    m = ((np.abs(pts[:, 0]) < EXTENT/2 - 1) & (np.abs(pts[:, 1]) < EXTENT/2 - 1)
         & np.isfinite(pts).all(1))
    p = pts[m]
    truth = np.array([tm_height(hmap, xx, yy) for xx, yy in p[:, :2]])
    rms = float(np.sqrt(np.mean((p[:, 2] - truth) ** 2)))
    st = sim.get_true_state()
    print(f"{i:3d} {pos[2]:8.3f} {st['pos'][2]:8.3f} {np.nanmin(d):10.3f} "
          f"{np.nanmax(d):10.3f} {bad_far:7d} {bad_nan:6d} {rms:9.4f}m")

print("\n--- does a repeated capture at the SAME pose agree with itself? ---")
pos = np.array([-19.0, 6.0, ALT])
prev = None
for rep in range(4):
    d, _, cp, la = rig.capture(pos)
    d = np.asarray(d, float)
    if prev is not None:
        print(f"  repeat {rep}: max|d - d_prev| = {np.abs(d - prev).max():.6f} m")
    prev = d

print("\n--- is the pose applied on the SAME render, or one late? ---")
poses = [np.array([-24.0, 6.0, ALT]), np.array([-4.0, 6.0, ALT])]
ds = []
for p_ in poses + poses[::-1]:
    d, _, cp, la = rig.capture(p_)
    ds.append((p_[0], np.asarray(d, float).copy()))
print(f"  capture at x=-24 then x=-4 then x=-4 then x=-24")
print(f"  |d(x=-24, 1st) - d(x=-24, last)| max = "
      f"{np.abs(ds[0][1] - ds[3][1]).max():.6f} m   (0 => pose applied immediately)")
print(f"  |d(x=-24,1st) - d(x=-4,2nd)|   max = {np.abs(ds[0][1] - ds[1][1]).max():.6f} m")
