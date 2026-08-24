"""Does the stand-in attitude hold keep the airframe level over a full survey? Uncontrolled, ti."""
import numpy as np
from skyscout_sim import SkyScoutSim, STEPS_PER_CONTROL
from nav_math import quat_to_R
from path_follow import PathFollower

ALT = 12.0
REGION_C, REGION, N_LEGS = (-15.0, 10.0), 40.0, 5
x0, x1 = REGION_C[0]-REGION/2, REGION_C[0]+REGION/2
ys = np.linspace(REGION_C[1]-REGION/2, REGION_C[1]+REGION/2, N_LEGS)
pts = []
for k, y in enumerate(ys):
    xs = (x0, x1) if k % 2 == 0 else (x1, x0)
    pts += [[xs[0], y, ALT], [xs[1], y, ALT]]
path = np.array(pts, float)

for hold in (False, True):
    sim = SkyScoutSim(start_pos=tuple(path[0]), logging_level="error")
    fol = PathFollower(path, speed=1.5, lookahead=2.5)
    done, tilts, ts = False, [], []
    while sim.sim_time < 400.0 and not done:
        st = sim.get_true_state()
        tgt, tvel, frac, done = fol.target(st["pos"])
        for _ in range(STEPS_PER_CONTROL):
            sim.apply_hover()
            if hold:
                sim.apply_attitude_hold()
            sim.apply_track(tgt, tvel)
            sim.step_sim()
        R = quat_to_R(sim.get_true_state()["quat"])
        tilts.append(np.degrees(np.arccos(np.clip(R[2,2], -1, 1))))
        ts.append(sim.sim_time)
    t = np.array(tilts)
    tag = "WITH hold " if hold else "no hold   "
    print(f"{tag} survey {ts[-1]:6.1f} s  done={done}  "
          f"tilt mean {t.mean():6.2f}  max {t.max():7.2f} deg  final {t[-1]:6.2f}")
    bad = np.where(t > 7.66)[0]
    print(f"            first exceeds 7.66 deg (arm-in-frame limit) at "
          f"{'t=%.1f s' % ts[bad[0]] if len(bad) else 'never'}")
    del sim
