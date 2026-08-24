"""B1.5 -- fly the B1 scripted path over procedural Mars terrain."""
import argparse
import numpy as np

from skyscout_sim import SkyScoutSim, STEPS_PER_CONTROL, CONTROL_HZ, MARS_G
import mars_terrain as mt

CRUISE_SPEED = 1.0
CLIMB = 2.0
CRUISE_X = 5.0
HOLD_S = 2.0


def reference(t, start_z):
    top = start_z + CLIMB
    climb_s, cruise_s = CLIMB / CRUISE_SPEED, CRUISE_X / CRUISE_SPEED
    if t < climb_s:
        return (np.array([0.0, 0.0, start_z + CRUISE_SPEED * t]),
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
    ap.add_argument("--shot", default="b1_5_terrain.png")
    args = ap.parse_args()

    probe = SkyScoutSim(gui=False, terrain=True, start_height=50)
    max_h = probe.terrain.max_h
    probe.disconnect()

    start_z = round(max_h + 2.0, 2)
    sim = SkyScoutSim(gui=not args.direct, start_height=start_z, terrain=True)
    t = sim.terrain

    print("terrain:")
    for k, v in t.summary().items():
        if k != "features":
            print(f"  {k:16s} {v}")
    for f in t.summary()["features"]:
        print(f"  feature          {f}")
    worst, n = t.verify_against_raycast()
    print(f"  height check     max |terrain_height - raycast| = {worst:.4f} m over {n} pts")
    print(f"\ngravity {MARS_G} m/s^2   start_z {start_z} m "
          f"(max terrain {max_h:.2f} m)   control {CONTROL_HZ:.0f} Hz")

    total = CLIMB + HOLD_S + CRUISE_X + 3.0
    clearances, errs = [], []
    while sim.sim_time < total:
        target, tvel, phase = reference(sim.sim_time, start_z)
        for _ in range(STEPS_PER_CONTROL):
            sim.apply_hover()
            sim.apply_track(target, tvel)
            sim.step_sim()
        s = sim.get_true_state()
        ground = sim.terrain_height(s["pos"][0], s["pos"][1])
        clearances.append(s["pos"][2] - ground)
        errs.append(float(np.linalg.norm(target - s["pos"])))

    s = sim.get_true_state()
    print(f"\nfinal pos      {s['pos'].round(3).tolist()}")
    print(f"tracking err   mean {np.mean(errs):.3f} m  max {np.max(errs):.3f} m")
    print(f"ground clear   min {np.min(clearances):.3f} m  "
          f"(over the flown path; terrain peak {max_h:.2f} m)")
    print(f"contacts       {len(p_contacts(sim))} with terrain  "
          f"{'-> NO CLIPPING' if not p_contacts(sim) else '-> CLIPPED'}")
    print(f"final |v|      {np.linalg.norm(s['lin_vel']):.4f} m/s   "
          f"|omega| {np.linalg.norm(s['ang_vel']):.4f} rad/s")
    print(f"screenshot     {sim.screenshot(args.shot, 640, 400)}")
    sim.disconnect()


def p_contacts(sim):
    import pybullet as p
    return p.getContactPoints(sim.drone, sim.plane)


if __name__ == "__main__":
    main()
