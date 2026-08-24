"""B1 -- headless Genesis flight over Mars terrain, with offline frames."""
import argparse
import os
import time

import numpy as np

from skyscout_sim import (SkyScoutSim, STEPS_PER_CONTROL, CONTROL_HZ,
                          SIM_HZ, MARS_G)
from render import FollowRenderer

CLIMB = 2.0
CRUISE_X = 12.0
SPEED = 1.5
HOLD_S = 2.0
CLEARANCE = 3.0


def reference(t, z0):
    """The hardcoded path. Returns (target_pos, target_vel, phase)."""
    top = z0 + CLIMB
    t_climb, t_cruise = CLIMB / SPEED, CRUISE_X / SPEED
    if t < t_climb:
        return (np.array([0.0, 0.0, z0 + SPEED * t]),
                np.array([0.0, 0.0, SPEED]), "climb")
    t -= t_climb
    if t < HOLD_S:
        return np.array([0.0, 0.0, top]), np.zeros(3), "hold"
    t -= HOLD_S
    if t < t_cruise:
        return (np.array([SPEED * t, 0.0, top]),
                np.array([SPEED, 0.0, 0.0]), "cruise")
    return np.array([CRUISE_X, 0.0, top]), np.zeros(3), "settle"


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--frames", type=int, default=40, help="0 = skip rendering")
    ap.add_argument("--outdir", default="b1_frames")
    args = ap.parse_args()

    # Genesis allows one gs.init per process
    from mars_world import build_heightmap
    max_h = float(build_heightmap()[0].max())
    z0 = round(max_h + CLEARANCE, 2)

    sim = SkyScoutSim(start_pos=(0, 0, z0), logging_level="error",
                      camera_res=(960, 600) if args.frames else None)
    world_summary = sim.world.summary()
    print("=== WORLD ===")
    for k, v in world_summary.items():
        if k != "features":
            print(f"  {k:16s} {v}")
    for f in world_summary["features"]:
        print(f"  feature          {f}")
    print(f"\n=== DRONE ===\n  links {sim.n_links} (URDF 21; 2 fixed-joint children merged)"
          f"  dofs {sim.n_dofs}  mass {sim.mass:.3f} kg")
    print(f"  gravity {MARS_G} m/s^2   weight {sim.mass * -MARS_G:.2f} N")
    print(f"  sim {SIM_HZ:.0f} Hz / control {CONTROL_HZ:.0f} Hz   start z {z0} m "
          f"(terrain peak {max_h:.2f} + {CLEARANCE} m)")

    rnd = None
    if args.frames:
        os.makedirs(args.outdir, exist_ok=True)
        rnd = FollowRenderer(sim, res=(960, 600))
        dt = rnd.warm_up()
        print(f"\n  WARM-UP RENDER (throwaway): {dt:.1f} s  "
              f"-- absorbs the cold shader-cache cost so timed frames are honest")

    total = CLIMB / SPEED + HOLD_S + CRUISE_X / SPEED + 3.0
    n_ticks = int(total * CONTROL_HZ)
    every = max(1, n_ticks // args.frames) if args.frames else 0

    clearances, errs, shots = [], [], 0
    t_render = 0.0
    t0 = time.time()
    tick = 0
    while sim.sim_time < total:
        target, tvel, phase = reference(sim.sim_time, z0)
        for _ in range(STEPS_PER_CONTROL):
            sim.apply_hover()
            sim.apply_track(target, tvel)
            sim.step_sim()
        s = sim.get_true_state()
        sim.record(phase)
        errs.append(float(np.linalg.norm(target - s["pos"])))
        clearances.append(s["pos"][2] - sim.terrain_height(s["pos"][0], s["pos"][1]))
        tick += 1
        if rnd and tick % every == 0:
            tr = time.time()
            rnd.capture(os.path.join(args.outdir, f"f{shots:03d}.png"), phase)
            t_render += time.time() - tr
            shots += 1

    wall = time.time() - t0
    s = sim.get_true_state()
    print(f"\n=== FLIGHT ===")
    print(f"  final pos      {s['pos'].round(3).tolist()}  goal "
          f"[{CRUISE_X}, 0.0, {z0 + CLIMB}]")
    print(f"  tracking err   mean {np.mean(errs):.3f} m   max {np.max(errs):.3f} m")
    print(f"  final |v|      {np.linalg.norm(s['lin_vel']):.4f} m/s   "
          f"|w| {np.linalg.norm(s['ang_vel']):.4f} rad/s")
    print(f"  MIN GROUND CLEARANCE  {np.min(clearances):.3f} m  "
          f"(terrain peak {max_h:.2f} m)")
    print(f"  clears all terrain features: {'YES' if np.min(clearances) > 0 else 'NO'}")
    print(f"  sim wall time  {wall - t_render:.1f} s for {sim.sim_time:.1f} s sim "
          f"-> {sim.sim_time / max(wall - t_render, 1e-9):.1f}x realtime (physics only)")
    if rnd:
        print(f"  frames         {shots} -> {args.outdir}/  "
              f"({t_render / max(shots,1) * 1000:.0f} ms/frame after warm-up)")
    print(f"  log            {sim.save_log('b1_trajectory.json')} "
          f"({len(sim.log)} samples, Blender-ready)")


if __name__ == "__main__":
    main()
