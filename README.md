# SkyScout — Mars Aerial Scout + Ground Rover

**Thesis: the scout maps hazards from the air, the rover drives the safe route it found.**

A drone flies a GPS-denied, unknown Mars world, localizes itself with a from-scratch
error-state Kalman filter, judges terrain through its own downward camera, accumulates a
persistent world hazard map, and hands that map to a rocker-bogie ground rover that plans
and drives a safe route across it — presented on a single mission-control HUD.

Mars gravity −3.72 m/s². Developed on an AMD Ryzen 7 5700U with integrated graphics under
WSL2 and **no GPU**, which drove most of the engineering choices below.

---

## Phases

| Phase | Deliverable |
|---|---|
| **B1** — [PyBullet Foundation](SKYSCOUT_B1_PYBULLET.md) | World + `terrain_height`; prove the drone loads, renders, and moves under a scripted command. No autonomy. |
| **B2** — ESKF | From-scratch error-state Kalman filter → `get_estimated_state()`. GPS-denied localization. |
| **B3** — A* planner | `plan(tmap, start, goal)` over a `TraversabilityMap` that takes a heightmap/cost array. |
| **B4** — [Perception](SKYSCOUT_B4_PERCEPTION.md) · [Step 3: Mapping](SKYSCOUT_B4_STEP3_MAPPING.md) | The drone builds the hazard map *itself* from depth + from-scratch GLCM texture analysis, replacing B3's ground-truth cheat. |
| **B5** — [Rover + HUD](SKYSCOUT_B5_ROVER_HUD.md) | Rover (35 links / 34 joints, rocker-bogie, 6 wheels, helipad bay) plans over the drone's *perceived* map; mission HUD makes it legible. |
| **B5-C** — [Blender cinematic pass](SKYSCOUT_B5C_BLENDER.md) | CPU Cycles renders driven from the recorded trajectory logs — presentation only, no functional change. |

## Engineering rules carried through every phase

These are the project's standing constraints, not incidental notes:

- **Act only on what was perceived.** Perception is *scored* against ground truth, but the
  system never *acts* on truth. The drone plans over the map it built.
- **`unknown` is never `safe`.** Unobserved cells stay distinct — letting unknown read as
  safe is a miss generator.
- **Truth-space judgement, one hand-check per claim.** Judge on plots and offline PNGs.
- **Report both failure directions** — miss vs. false-alarm — never a lone aggregate.
- **Honest coupling.** `capture()` uses the TRUE body pose (a bolted-on camera can't
  teleport to the estimate); `project()` uses the ESTIMATE. The gap between them is the map
  smear — the real B2↔B4 coupling, surfaced rather than hidden.

## Renders

Cinematic stills and mission videos from the B5-C Blender pass, driven by the logged
trajectories of the verified run — see [`renders/`](renders/).

## Scripts

- [`scripts/check-skyscout-poses.py`](scripts/check-skyscout-poses.py) — validates SRDF named
  poses, the deployment choreography, and the lift-off-before-tilt constraint. FK from the
  URDFs plus OBB separating-axis tests on the collision pairs that stay enabled. Cylinders and
  meshes are bounded by their enclosing box, so the test is deliberately **conservative**: a
  reported clear pair is truly clear; a reported hit may be the bounding box and prints its
  depth for triage.
- [`scripts/check-robot-descriptions.py`](scripts/check-robot-descriptions.py) — robot
  description validation.

## External reference

The CAD/body side of this project was built against
**[earthtojake/text-to-cad](https://github.com/earthtojake/text-to-cad)** (v0.3.9), which played
a significant part in the work. It is a separate 2.1 GB upstream clone and is **intentionally
not vendored** into this repository — it is `.gitignore`d and referenced here instead. To work
with that part of the pipeline, clone it alongside this repo:

```bash
git clone https://github.com/earthtojake/text-to-cad.git
```

The two validation scripts above were written for that toolchain and live in
[`scripts/`](scripts/) here so they are version-controlled with this project.

## Note on scope of this repository

This repo holds the **phase specifications, validation scripts, and rendered output**. The
simulation runtime (ESKF, planner, perception, HUD) was developed in a separate native-WSL
working tree for disk-speed reasons documented in
[B1.1](SKYSCOUT_B1_PYBULLET.md) and is not included here.
