"""B1.5 flight with the visual clarity layer on top."""
import argparse
import os
import time

import numpy as np

from skyscout_sim import SkyScoutSim, STEPS_PER_CONTROL, CONTROL_HZ, SIM_DT, MARS_G
from visual import VisualLayer, capture_frame
from b1_5_terrain import reference, CLIMB, CRUISE_X, HOLD_S


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--width", type=int, default=640)
    ap.add_argument("--height", type=int, default=420)
    ap.add_argument("--capture", default="", help="directory for V.6 stills")
    ap.add_argument("--every", type=int, default=10, help="capture every N ticks")
    ap.add_argument("--realtime", action="store_true")
    ap.add_argument("--no-follow", action="store_true")
    args = ap.parse_args()

    import pybullet as p
    p.connect(p.GUI, options=f"--width={args.width} --height={args.height}")
    for flag in (p.COV_ENABLE_RGB_BUFFER_PREVIEW,
                 p.COV_ENABLE_DEPTH_BUFFER_PREVIEW,
                 p.COV_ENABLE_SEGMENTATION_MARK_PREVIEW,
                 p.COV_ENABLE_SHADOWS,
                 p.COV_ENABLE_TINY_RENDERER,
                 p.COV_ENABLE_GUI):
        p.configureDebugVisualizer(flag, 0)

    probe_start = 7.27
    sim = SkyScoutSim(gui=True, start_height=probe_start, terrain=True, attach=True)

    vis = VisualLayer(sim, follow=not args.no_follow)
    print(f"gravity {MARS_G} m/s^2 | terrain {sim.terrain.summary()['extent_m']:.0f} m "
          f"| control {CONTROL_HZ:.0f} Hz | window {args.width}x{args.height}")

    if args.capture:
        os.makedirs(args.capture, exist_ok=True)

    total = CLIMB + HOLD_S + CRUISE_X + 3.0
    t_wall0, tick, shots = time.time(), 0, 0
    while sim.sim_time < total:
        target, tvel, phase = reference(sim.sim_time, probe_start)
        for _ in range(STEPS_PER_CONTROL):
            sim.apply_hover()
            sim.apply_track(target, tvel)
            sim.step_sim()
            if args.realtime:
                time.sleep(SIM_DT)
        rt = sim.sim_time / max(time.time() - t_wall0, 1e-6)
        vis.update(phase, rt)
        tick += 1
        if args.capture and tick % args.every == 0:
            capture_frame(sim, os.path.join(args.capture, f"f{shots:04d}.png"),
                          960, 600, vis=vis, phase=phase, rt_factor=rt)
            shots += 1

    s = sim.get_true_state()
    print(f"final pos {s['pos'].round(3).tolist()}  "
          f"|v| {np.linalg.norm(s['lin_vel']):.4f}  "
          f"|w| {np.linalg.norm(s['ang_vel']):.4f}")
    if args.capture:
        print(f"captured {shots} frames -> {args.capture}/")
    else:
        print("still: " + capture_frame(sim, "b1_5_visual.png", 960, 600,
                                       vis=vis, phase=phase, rt_factor=rt))
        print("holding the window open; Ctrl-C to quit")
        try:
            while True:
                sim.apply_hover()
                sim.apply_track(target, tvel)
                sim.step_sim()
                vis.update(phase, rt)
                time.sleep(0.02)
        except KeyboardInterrupt:
            pass
    p.disconnect()


if __name__ == "__main__":
    main()
