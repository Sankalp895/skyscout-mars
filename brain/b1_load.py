"""B1.3 Checkpoint 1 -- load the drone, render it, prove the GUI is usable."""
import argparse
import time

import pybullet as p

from skyscout_sim import SkyScoutSim, MARS_G, SIM_HZ, CONTROL_HZ, SIM_DT


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--seconds", type=float, default=0.0,
                    help="0 = run until the window is closed")
    ap.add_argument("--direct", action="store_true")
    ap.add_argument("--shot", default="b1_checkpoint1.png")
    args = ap.parse_args()

    sim = SkyScoutSim(gui=not args.direct, start_height=1.0)

    print(f"gravity      {MARS_G} m/s^2  (Mars, not Earth)")
    print(f"rates        sim {SIM_HZ:.0f} Hz / control {CONTROL_HZ:.0f} Hz")
    print(f"drone        {sim.n_joints + 1} links, {sim.n_joints} joints, "
          f"{sim.mass:.3f} kg  -> weight {sim.mass * -MARS_G:.2f} N")

    frames, t0, last = 0, time.time(), time.time()
    while True:
        sim.apply_hover()
        sim.step_sim()
        frames += 1

        if args.seconds and sim.sim_time >= args.seconds:
            break
        if not args.seconds:
            dt = time.time() - last
            if dt < SIM_DT:
                time.sleep(SIM_DT - dt)
            last = time.time()

    wall = time.time() - t0
    print(f"\nrendered     {frames} steps in {wall:.2f} s wall")
    print(f"throughput   {frames / wall:.0f} steps/s  "
          f"({frames / wall / SIM_HZ:.2f}x realtime)")

    shot = sim.screenshot(args.shot)
    print(f"screenshot   {shot}")

    s = sim.get_true_state()
    print(f"final pos    {s['pos'].round(4).tolist()}   (held by the hover stand-in)")
    sim.disconnect()


if __name__ == "__main__":
    main()
