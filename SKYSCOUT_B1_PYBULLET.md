# SkyScout — Track B, Phase B1: PyBullet Foundation

First software phase of the brain. Goal: prove the drone loads into PyBullet, renders, and moves under a
scripted command. **No autonomy yet** — no ESKF, no A*, no perception. This is the R1 of Track B: the
smallest thing that runs, verified before anything is built on it.

Read `SKYSCOUT_RESUME.md` for project context if this is a fresh session. Simulator decision is settled:
**PyBullet**, chosen because the dev machine is an AMD Ryzen 7 5700U with *integrated* Radeon graphics on
WSL — Gazebo's GPU/display path is too high-risk there. PyBullet renders fine on integrated graphics and
reads URDF directly. The ESKF + A* brain is pure Python and stays portable to another sim later.

## B1.0 Scope discipline

- Build ONLY what's listed here. If you find yourself writing a Kalman filter or a planner, stop — that's
  B2/B3.
- Small verifiable steps; stop at the checkpoint for the user's review.
- No new physics tuning beyond what's needed to make the drone sit still and respond to a command.
- This is a throwaway-quality scaffold to prove the pipeline, not production code. Clarity over polish.

## B1.1 Environment

- Python 3.12 in a fresh venv (keep it separate from the text-to-cad `.venv`):
  `python3 -m venv ~/skyscout-brain/.venv` (native WSL filesystem, NOT /mnt/c — Python sim work wants
  fast disk; this is compute, not CAD, so the /mnt/c reasoning is reversed here).
- `pip install pybullet numpy` to start. Nothing else yet.
- Confirm: `python -c "import pybullet as p; print(p.getAPIVersion())"` prints a version.

## B1.2 Get the drone URDF

The drone URDF + meshes already exist at
`/mnt/c/Users/sanka/Desktop/Personal Project/Mars_rover/text-to-cad/models/skyscout_drone/`.

1. Copy the drone model folder (URDF + `meshes/`) into the brain project, e.g.
   `~/skyscout-brain/models/skyscout_drone/`. Keep the relative mesh paths intact.
2. PyBullet reads URDF but is pickier than the CAD viewer about a few things. Before loading, check and
   report:
   - mesh format: PyBullet wants OBJ or STL for collision, and handles OBJ/STL/DAE for visual. If the
     meshes are STL, good. If any are a format PyBullet won't load, convert and note it.
   - `package://` or absolute mesh URIs will fail — they must be relative to the URDF. Fix if present.
   - the `base_footprint` root and multi-shape links should load, but report any link PyBullet drops.

## B1.3 Load and render — the first checkpoint

Write `b1_load.py`:
- Connect in GUI mode (`p.connect(p.GUI)`).
- Set gravity to **Mars: −3.72 m/s²** (not Earth's −9.81 — get this right now, it matters for everything
  downstream).
- Load a ground plane.
- Load the drone URDF at a height of ~1 m, deployed pose (wings out, nacelles vertical).
- Run the sim loop so the window stays open and you can orbit the camera.

**Checkpoint 1 (the real test):** the GUI window opens, renders smoothly (not black, not <5 fps), and the
drone appears intact — body, wings, four nacelles, rotors. Report: does it render? frame rate feel? all
links present? Take a screenshot. **If rendering fails or crawls, stop — that's the integrated-graphics
risk surfacing, and we solve display before anything else.**

## B1.4 Make it hold still, then move — the second checkpoint

The drone will fall under gravity (it has mass, no lift yet). Two steps:

1. **Hover by script, not physics.** Don't model aerodynamics yet. Either fix the base in place, or apply
   a constant upward force equal to weight (mass × 3.72) at the center of mass each step, so it holds
   altitude. This is a fake — a placeholder for the real rotor thrust that comes later. Label it clearly
   as a stand-in.
2. **Command one scripted motion.** Make the drone rise 2 m, hold, then translate 5 m forward at constant
   velocity, along a hardcoded path. No planning — just prove you can command position/velocity and the
   drone follows.

**Checkpoint 2:** the drone holds altitude and follows the scripted path smoothly. Report the approach
used (fixed base vs applied force), and confirm the motion is stable (no spinning off, no sinking).

## B1.5 Expose the interface B2 will need

The whole point of B1 is to hand B2 (localization) a clean seam. Structure the code so these are already
available, even if trivially:
- `get_true_state()` → returns the drone's ground-truth position, orientation (quaternion), linear and
  angular velocity from PyBullet. This is the "truth" the ESKF will later be compared against.
- `step_sim(dt)` → advances one timestep.
- a fixed sim timestep (e.g. 1/240 s, PyBullet default) and a separate, slower control rate — note both.

Do NOT implement sensors or noise yet — just expose the true state cleanly. B2 will add simulated IMU +
noisy measurements on top of this seam.

## B1.6 Report and stop

Report:
- render result + fps feel + screenshot (B1.3)
- any mesh conversions or URI fixes needed (B1.2)
- scripted-motion result (B1.4)
- confirmation the `get_true_state` / `step_sim` seam exists
- Mars gravity confirmed set to −3.72

Then stop. Do not start B2 (the ESKF). Next phase, once B1 is confirmed visually, is simulated IMU +
error-state Kalman filter for GPS-denied localization — built from scratch, since the Inertial Ghost code
isn't available to port.

## B1.7 Anti-goals

- No ESKF, no A*, no perception, no HUD. Those are B2–B5.
- No aerodynamics model — the hover is a scripted stand-in, and that's fine for now.
- Don't load the rover yet — the drone flies first; the rover is a later phase.
- Don't fight to make it pretty — beauty is a Blender pass at the very end, not now.
