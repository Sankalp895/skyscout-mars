"""Is the drone TUMBLING during survey turns, or merely yawing? 60% of survey captures were rej."""
import numpy as np

from skyscout_sim import SkyScoutSim, STEPS_PER_CONTROL
from nav_math import quat_to_R
from path_follow import PathFollower

ALT = 12.0
path = np.array([[-5.0, -10.0, ALT],
                 [5.0, -10.0, ALT],
                 [5.0, 2.0, ALT],
                 [-5.0, 2.0, ALT]])

sim = SkyScoutSim(start_pos=tuple(path[0]), logging_level="error")
follower = PathFollower(path, speed=1.5, lookahead=2.5)

print(f"{'t':>6} {'x':>7} {'y':>7} {'z':>7} | {'tilt':>7} {'yaw':>8} "
      f"{'|w|':>8} {'|a_cmd|':>9}")
done = False
rows = []
while sim.sim_time < 60.0 and not done:
    st = sim.get_true_state()
    tgt, tvel, frac, done = follower.target(st["pos"])
    for _ in range(STEPS_PER_CONTROL):
        sim.apply_hover()
        sim.apply_track(tgt, tvel)
        sim.step_sim()
    s = sim.get_true_state()
    R = quat_to_R(s["quat"])
    tilt = np.degrees(np.arccos(np.clip(R[2, 2], -1, 1)))
    yaw = np.degrees(np.arctan2(R[1, 0], R[0, 0]))
    total = np.degrees(2 * np.arccos(np.clip(abs(s["quat"][0]), 0, 1)))
    w = np.linalg.norm(s["ang_vel"])
    acmd = np.linalg.norm(6.0 * (tgt - s["pos"]) + 4.0 * (tvel - s["lin_vel"]))
    rows.append((sim.sim_time, tilt, yaw, total, w, acmd))
    if len(rows) % 8 == 0:
        print(f"{sim.sim_time:6.2f} {s['pos'][0]:7.2f} {s['pos'][1]:7.2f} "
              f"{s['pos'][2]:7.2f} | {tilt:6.2f}d {yaw:7.2f}d {w:8.3f} {acmd:9.2f}")

r = np.array(rows)
print(f"\n  TILT from vertical : mean {r[:,1].mean():6.2f}  max {r[:,1].max():6.2f} deg")
print(f"  YAW                : range [{r[:,2].min():.1f}, {r[:,2].max():.1f}] deg")
print(f"  TOTAL rotation ang : mean {r[:,3].mean():6.2f}  max {r[:,3].max():6.2f} deg")
print(f"  |ang vel|          : mean {r[:,4].mean():6.3f}  max {r[:,4].max():6.3f} rad/s")
print(f"  |commanded accel|  : mean {r[:,5].mean():6.2f}  max {r[:,5].max():6.2f} m/s^2")
print()
if r[:, 1].max() > 10:
    print("  -> the airframe genuinely TILTS; the torque-free claim fails under")
    print("     the accelerations these corners command.")
else:
    print("  -> tilt stays small; the large 'rotation angle' was YAW, which is")
    print("     harmless to a straight-down camera. The rejection cause is")
    print("     something else.")
