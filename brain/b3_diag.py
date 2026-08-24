"""Diagnostic: are the two guidance modes really producing different flights? Checkpoint 4 repo."""
import numpy as np
from b3_flight import build_plan, fly

tm, r, path, ground, route_max, cruise_z = build_plan()

T = fly("truth", path, tm)
E = fly("estimate", path, tm)

print("=== are the TRUE trajectories different at all? ===")
n = min(len(T["true"]), len(E["true"]))
d = np.linalg.norm(T["true"][:n] - E["true"][:n], axis=1)
print(f"  samples compared      {n}  (truth {len(T['true'])}, est {len(E['true'])})")
print(f"  |truth_traj - est_traj|  mean {d.mean():.4f}  max {d.max():.4f} m")
print(f"  identical samples: {int((d < 1e-12).sum())} / {n}")

print("\n=== is the estimate branch actually being followed? ===")
eskf_err = np.linalg.norm(E["true"] - E["est"], axis=1)
print(f"  ESKF error         mean {eskf_err.mean():.4f}  max {eskf_err.max():.4f} m")
print(f"  TRUE xtrack (est run)  mean {E['xtrack_true'].mean():.4f}  "
      f"max {E['xtrack_true'].max():.4f} m")
print(f"  EST  xtrack (est run)  mean {E['xtrack_guid'].mean():.4f}  "
      f"max {E['xtrack_guid'].max():.4f} m")
print("  EXPECTATION: if the controller drives the ESTIMATE onto the path,")
print("  then est-xtrack should be SMALL and true-xtrack should be ~ESKF error.")
print(f"  observed: est-xtrack {E['xtrack_guid'].mean():.3f} > "
      f"true-xtrack {E['xtrack_true'].mean():.3f}  -> BACKWARDS")

print("\n=== xtrack array equality between runs ===")
m = min(len(T["xtrack_true"]), len(E["xtrack_true"]))
print(f"  max|dx| = {np.abs(T['xtrack_true'][:m] - E['xtrack_true'][:m]).max():.3e}")

print("\n=== where does the max occur? ===")
for nm, h in (("truth", T), ("estimate", E)):
    i = int(np.argmax(h["xtrack_true"]))
    print(f"  {nm:9s} argmax at sample {i}/{len(h['xtrack_true'])} "
          f"t={h['t'][i]:.2f}s  val {h['xtrack_true'][i]:.4f}")
