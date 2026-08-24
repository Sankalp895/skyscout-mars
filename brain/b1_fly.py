"""B1.4 Checkpoint 2 -- hold altitude, then fly one hardcoded path."""
import argparse
import time

import numpy as np

from skyscout_sim import SkyScoutSim, SIM_DT, STEPS_PER_CONTROL, CONTROL_HZ

START_Z = 1.0
CLIMB = 2.0
CRUISE_X = 5.0
CRUISE_SPEED = 1.0
HOLD_S = 2.0


def reference(t):
    """Hardcoded path. Returns (target_xyz, target_vel, phase_name)."""
    climb_s = CLIMB / CRUISE_SPEED
    cruise_s = CRUISE_X / CRUISE_SPEED
    top = START_Z + CLIMB
    if t < climb_s:
        return (np.array([0.0, 0.0, START_Z + CRUISE_SPEED * t]),
                np.array([0.0, 0.0, CRUISE_SPEED]), "climb")
    t -= climb_s
    if t < HOLD_S:
        return np.array([0.0, 0.0, top]), np.zeros(3), "hold"
    t -= HOLD_S
    if t < cruise_s:
        return (np.array([CRUISE_SPEED * t, 0.0, top]),
                np.array([CRUISE_SPEED, 0.0, 0.0]), "cruise")
    return np.array([CRUISE_X, 0.0, top]), np.zeros(3), "settle"


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--direct", action="store_true")
    ap.add_argument("--realtime", action="store_true", help="pace to wall clock")
    args = ap.parse_args()

    sim = SkyScoutSim(gui=not args.direct, start_height=START_Z)
    total = CLIMB / CRUISE_SPEED + HOLD_S + CRUISE_X / CRUISE_SPEED + 3.0

    print(f"path: climb {CLIMB} m, hold {HOLD_S} s, cruise {CRUISE_X} m "
          f"@ {CRUISE_SPEED} m/s   (control {CONTROL_HZ:.0f} Hz)")
    print(f"{'t':>6} {'phase':>7} {'target(x,z)':>18} {'actual(x,z)':>18} "
          f"{'err':>7} {'|w|':>6}")

    errs, next_report, last_phase = [], 0.0, None
    while sim.sim_time < total:
        target, tvel, phase = reference(sim.sim_time)
        for _ in range(STEPS_PER_CONTROL):
            sim.apply_hover()
            sim.apply_track(target, tvel)
            sim.step_sim()
            if args.realtime:
                time.sleep(SIM_DT)

        s = sim.get_true_state()
        err = float(np.linalg.norm(target - s["pos"]))
        errs.append(err)
        if sim.sim_time >= next_report or phase != last_phase:
            print(f"{sim.sim_time:6.2f} {phase:>7} "
                  f"{target[0]:8.2f},{target[2]:7.2f} "
                  f"{s['pos'][0]:8.2f},{s['pos'][2]:7.2f} "
                  f"{err:7.3f} {np.linalg.norm(s['ang_vel']):6.3f}")
            next_report = sim.sim_time + 1.0
            last_phase = phase

    s = sim.get_true_state()
    goal = np.array([CRUISE_X, 0.0, START_Z + CLIMB])
    print(f"\nfinal pos     {s['pos'].round(3).tolist()}   goal {goal.tolist()}")
    print(f"final error   {np.linalg.norm(goal - s['pos']):.4f} m")
    print(f"tracking err  mean {np.mean(errs):.3f} m   max {np.max(errs):.3f} m")
    print(f"final |v|     {np.linalg.norm(s['lin_vel']):.4f} m/s")
    print(f"final |omega| {np.linalg.norm(s['ang_vel']):.4f} rad/s  (spin check)")
    print(f"screenshot    {sim.screenshot('b1_checkpoint2.png', 640, 400)}")
    sim.disconnect()


if __name__ == "__main__":
    main()
