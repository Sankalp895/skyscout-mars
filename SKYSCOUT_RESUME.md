# SkyScout — Project Context (read this first)

Fresh-session entry point. Every phase document (`SKYSCOUT_B*.md`) opens by pointing here for
context; this file is that context. For the physical design spec see
[`docs/design/SKYSCOUT_BRIEF.md`](docs/design/SKYSCOUT_BRIEF.md) and
[`docs/design/SKYSCOUT_DESIGN_v3.md`](docs/design/SKYSCOUT_DESIGN_v3.md). For the runtime see
[`brain/README.md`](brain/README.md).

## What this is

**SkyScout** — a Mars aerial-scout simulation. An autonomous rotorcraft (Mars-helicopter class)
flies ahead over Martian terrain, perceives the ground from above, builds a live
hazard/traversability map, and hands safe routes down to a ground rover that follows them.

**The hook:** Ingenuity's mission ended due to navigation errors over featureless terrain. This
project targets exactly that failure mode — localization and hazard perception over low-texture
ground, with no GPS and no pre-loaded map.

## Two tracks

| Track | Scope | Where it lives |
|---|---|---|
| **A — Body** | The drone and rover as parametric CAD + URDF. The physical bodies that load into the simulator. | [`docs/design/`](docs/design/) · built against the `text-to-cad` toolchain |
| **B — Brain** | Localization, planning, perception, the rover loop, and the mission HUD. | [`brain/`](brain/) · phase docs `SKYSCOUT_B1`–`B5C` |

## Hard scope constraints

- **Simulation only. No physical hardware.** No slicing, no G-code, no printer workflows.
- **Zero spend.** Everything runs locally on free/open-source tooling — no hosted APIs, no cloud
  compute, no paid services.
- Engineering choices stay **credible and physically plausible for Mars**: thin atmosphere, low
  gravity (**−3.72 m/s²**), no GPS.

## Environment

- Windows 11 host, working inside **WSL2 / Ubuntu 24.04 LTS**.
- **Python 3.12.3.** There is no `python3.11` on this machine.
- **AMD Ryzen 7 5700U, integrated Radeon graphics, no GPU.** Everything is CPU-only.
- Track A (CAD) work lives on the `/mnt/c` Windows mount; Track B (sim) work lives on the
  **native WSL filesystem**, because sim compute wants fast disk while the CAD side does not.

**Simulator decision — note the change.** [B1](SKYSCOUT_B1_PYBULLET.md) was written for
**PyBullet**, chosen because Gazebo's GPU/display path was too risky on integrated graphics. The
project subsequently moved to **Genesis (headless)**, which is what B2–B5C and all verified
results actually use. B1's filename and text still say PyBullet; that is a historical record, not
the current stack.

## Phase map

| Phase | Adds | Doc |
|---|---|---|
| **B1** | Headless world, Mars gravity, procedural terrain, `terrain_height`, scripted flight | [B1](SKYSCOUT_B1_PYBULLET.md) |
| **B2** | Simulated IMU + **from-scratch ESKF** (15-dim error state, Joseph-form update, reset Jacobian) → `get_estimated_state()` | — |
| **B3** | Traversability cost map (slope + top-hat prominence) and **A\*** flown on the estimate → `plan(tmap, start, goal)` | — |
| **B4** | **Perception** — depth + GLCM per-patch hazard score → persistent world map → plan over *that* | [B4](SKYSCOUT_B4_PERCEPTION.md) · [Step 3](SKYSCOUT_B4_STEP3_MAPPING.md) |
| **B5** | **Rover** — own traversability, A\* route, rocker-bogie differential, mission HUD | [B5](SKYSCOUT_B5_ROVER_HUD.md) |
| **B5C** | Blender cinematic pass driven from the recorded logs (presentation only) | [B5C](SKYSCOUT_B5C_BLENDER.md) |

## Verified results

| Capability | Result |
|---|---|
| GPS-denied navigation | dead-reckoning drifts **197.6 m** in 40 s → ESKF holds **0.54 m** mean, 1.80 m max |
| Path planning | A\* detours 1.03× / 1.16× / 1.26×, zero blocked cells entered, unreachable pair correctly refused |
| Terrain perception | **0 missed hazards**, 2.3% false alarms on static captures |
| Autonomous mapping | 100% coverage, 2.5% miss / 6.1% false alarm, IoU 0.86 |
| Closing the loop | flew a route planned on its **own** map; closest real hazard 1.89 m vs a 1.16 m stacked error budget |
| Rover autonomy | 85.0 m route over its own traversability, 0 undrivable cells entered |
| Rocker-bogie differential | chassis tilt 32.8° → 20.2°, rocker sum 62.2° → 1.4° |

## Engineering rules — carried through every phase

These are standing constraints, not incidental notes:

- **Act only on what was perceived.** Perception is *scored* against ground truth; the system never
  *acts* on truth. The drone plans over the map it built.
- **`unknown` is never `safe`.** Unobserved cells stay a distinct third state — letting unknown read
  as safe is a miss generator.
- **Honest coupling.** `capture()` uses the TRUE body pose (a bolted-on camera cannot teleport to the
  estimate); `project()` uses the ESTIMATE. The gap between them is the map smear — the real B2↔B4
  coupling, surfaced rather than hidden.
- **Judge in truth space, one hand-check per claim.** No claim lands on an aggregate alone.
- **Report both failure directions** — miss vs. false-alarm — separately, never a single blended number.
- **Judge on written artifacts.** Genesis runs headless with no viewer: read the emitted PNGs, JSON,
  and NPZ files, not a live window.
- **Scope discipline.** Build only what the current phase lists. If a Kalman filter or a planner
  starts appearing early, stop — that belongs to a later phase.

---

<sub>Reconstructed as the shared project context the phase documents reference, from the Track A
design brief, the phase specs, and the verified runtime results in <code>brain/README.md</code>.</sub>
