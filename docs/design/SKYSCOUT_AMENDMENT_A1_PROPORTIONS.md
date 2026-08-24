# SkyScout — Design Amendment A1: Rover Proportions

**Applies to `SKYSCOUT_DESIGN_v3.md`. Patch sections 2, 3 and 4 with the values below.**
Everything not listed here is unchanged — same 28 joints, same helipad mechanism, same drone.

## Why we are changing this

R1 was reviewed and the silhouette does not read as a Mars rover. The diagnosis is proportions, not
missing detail: a 3.00 × 1.40 × 0.70 m plain box with wheels at ±0.90 (only 0.20 m outboard of the
chassis wall) reads as a truck. Detail passes cannot fix this — R3/R4 would just produce a detailed slab.

**The single most recognisable trait of a Mars rover is that its wheels are held well away from the body
on visibly articulated arms.** Every change below serves that, plus a more compact, faceted body.

This is exactly why R1 exists as a checkpoint. Rebuilding it now is cheap.

## A1.1 Revised global parameters

| Parameter | Was | **Now** |
|---|---|---|
| `chassis_length` | 3.00 | **2.40** |
| `chassis_width` | 1.40 | **1.30** |
| `chassis_height` | 0.70 | **0.62** |
| `wheel_diameter` | 0.50 | **0.58** |
| `wheel_width` | 0.40 | **0.36** |
| `ground_clearance` | 0.60 | **0.66** |
| `track_centreline` | ±0.90 | **±0.98** |
| `wheel_station_front` | +1.35 | **+1.30** |
| `wheel_station_mid` | −0.05 | **−0.10** |
| `wheel_station_rear` | −1.35 | **−1.30** |
| `mast_height` | 1.30 | **1.45** |
| `base_link` height above ground | 0.95 | **0.97** |

Consequences worth noting:
- Wheels now sit **0.33 m outboard** of the chassis wall (0.65), up from 0.20 m. The suspension arms
  become a visible structural feature rather than a hidden bracket.
- Overall width 2.32 m on a 1.30 m body — the body is now clearly narrower than the track.
- Wheel stations ±1.30 on a 2.40 m body means **front and rear wheels extend beyond the body**, as on
  real rovers. This is intentional.
- Wheel diameter is now 24% of chassis length, up from 17%.
- Deck top sits at ground z = **1.28**; `base_link` z_rel of deck = **+0.31**.

## A1.2 Revised joint origins (replaces the section 4 table rows listed)

| Joint | Parent | **New origin** | Axis | Limits |
|---|---|---|---|---|
| `rocker_port` | `base_link` | (+0.20, +0.66, +0.02) | Y | ±35° |
| `rocker_stbd` | `base_link` | (+0.20, −0.66, +0.02) | Y | ±35° |
| `bogie_port` | `rocker_port` | (−0.82, 0, −0.26) | Y | ±35° |
| `bogie_stbd` | `rocker_stbd` | (−0.82, 0, −0.26) | Y | ±35° |
| `steer_fl` | `rocker_port` | (+1.10, +0.32, −0.70) | Z | ±45° |
| `steer_fr` | `rocker_stbd` | (+1.10, −0.32, −0.70) | Z | ±45° |
| `steer_rl` | `bogie_port` | (−0.68, +0.32, −0.44) | Z | ±45° |
| `steer_rr` | `bogie_stbd` | (−0.68, −0.32, −0.44) | Z | ±45° |
| `drive_ml` | `bogie_port` | (+0.52, +0.32, −0.44) | Y | continuous |
| `drive_mr` | `bogie_stbd` | (+0.52, −0.32, −0.44) | Y | continuous |
| `mast_pan` | `base_link` | (−0.95, +0.34, +0.31) | Z | ±180° |
| `mast_tilt` | `mast_pan` | (0, 0, +1.45) | Y | −90°…+45° |
| `arm_shoulder_az` | `base_link` | (+1.25, 0, −0.14) | Z | ±120° |
| `helipad_frame` | `base_link` | (+0.20, 0, +0.31) | — | fixed |

`drive_fl/fr/rl/rr` remain at (0,0,0) on their steering knuckles. All other joints unchanged.

Fixed accessories: `rtg` at (−1.28, 0, +0.10) protruding aft; `antenna` at (−0.70, −0.42, +0.31).
Note the mast is offset to **port** and the antenna to **starboard** — deliberate asymmetry reads as
purposeful engineering rather than a symmetrical toy.

## A1.3 Closure check — re-derive and confirm

Wheel centres must land at **z_rel = −0.68** (ground z = 0.29 = wheel radius).

- front = rocker (+0.20, ±0.66, +0.02) + (+1.10, ±0.32, −0.70) = **(+1.30, ±0.98, −0.68)** ✔
- bogie pivot = (+0.20 − 0.82, ±0.66, +0.02 − 0.26) = (−0.62, ±0.66, −0.24) → ground 0.73
- middle = (−0.62 + 0.52, ±0.98, −0.24 − 0.44) = **(−0.10, ±0.98, −0.68)** ✔
- rear = (−0.62 − 0.68, ±0.98, −0.68) = **(−1.30, ±0.98, −0.68)** ✔

Report these three results back before authoring geometry.

## A1.4 Silhouette requirements — build these at R1, not R4

These are structural, not cosmetic, so they belong in the skeleton. Without them the shape stays bland
no matter how much surface detail arrives later.

**1. Faceted chassis, not a plain box.** Replace the box primitive with a faceted solid:
- Front face rakes back ~20° from vertical (a sloped nose, not a wall).
- Lower side rails chamfered ~30° along the full length.
- Stepped deck: the aft third of the deck sits **0.08 m higher** than the forward portion, forming a
  raised shoulder the mast and RTG mount to.

**2. Visible triangulated suspension arms.** Not 0.10 m box sections. Each rocker and bogie is a
**tube-and-strut truss**: main tubes of 0.13 m diameter, a diagonal brace across each V, and a visible
**pivot boss of 0.19 m diameter** at every articulation. These arms are the most identifiable part of the
whole vehicle — they should look like structure carrying load.

**3. Wheel hub bosses.** Even at R1, give each wheel a raised central hub (0.18 m diameter, 0.06 m
proud) so it reads as a driven wheel, not a disc.

**4. Mast with a head.** Tapered column, a collar at the base, and a camera head with **two distinct
camera barrels** protruding forward, set 0.24 m apart. The two "eyes" are what make a rover read as
purposeful rather than a machine. Add a small mast-top low-gain antenna.

**5. RTG canted.** Cylinder tilted ~15° nose-up at the rear with **8 radial cooling fins**. It should
break the rear silhouette, not hide inside it.

**6. Arm parked visibly.** In the stowed pose, fold the arm along the front-lower face so it is visible
in the silhouette rather than tucked out of sight.

## A1.5 Helipad — verify the new fit

Chassis is now 2.40 × 1.30, so re-audit. Expected values, to confirm with measurements:

| Check | Expected |
|---|---|
| Well footprint | 1.10 (X) × 1.00 (Y) × 0.24 deep — unchanged |
| Well X span | −0.35 … +0.75 (within chassis ±1.20) ✔ |
| Deck margin outboard of each hinge | 0.15 m (hinge ±0.50, chassis wall ±0.65) ✔ |
| Clearance to `mast_pan` (x −0.95) | 0.60 m aft ✔ |
| Clearance to `arm_shoulder_az` (x +1.25) | 0.50 m fwd ✔ |
| Clearance to `antenna` (x −0.70) | 0.35 m aft ✔ |
| Panel tip at full open | y = ±1.00, directly above the wheels |
| Panel tip to wheel top (ground 0.58) | 0.70 m ✔ |

The 0.15 m deck margin outboard of the hinges is tight but intentional — it makes the pad read as
occupying nearly the full body width. **If any of these fail, report the number; do not resize silently.**

## A1.6 Drone — unchanged

The well dimensions are identical, so section 6 of v3 stands as written. No changes.

## A1.7 What to do now

1. Re-derive the three closure checks in A1.3 and report them.
2. Rebuild R1 with the revised parameters and the A1.4 silhouette features.
3. Re-run the helipad clearance audit in A1.5 and report the filled table.
4. Stop at the R1 checkpoint again for visual review.

Phases R2–R8 are unchanged.
