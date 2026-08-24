# SkyScout — Complete Design Specification (v3)

Read `SKYSCOUT_BRIEF.md` first. **This supersedes all earlier rover specs.**

This is not a brief asking you to design something. **The design is already made.** Every dimension,
joint origin, axis, and limit is given below. Your job is to execute it faithfully, flag any internal
contradiction you find, and report measured results. If you believe a number is wrong, say so and show
the calculation — do not silently change it.

---

## 0. Conventions

- **Frame:** ROS/URDF standard. **X forward, Y port (left), Z up.** Right-handed.
- **Units:** metres, radians. Angles below are given in degrees for readability — convert.
- **`base_link` origin:** geometric centre of the chassis box, **0.95 m above ground plane**.
- All joint origins in the tables are **relative to the parent link's origin** unless stated.
- Mirrored parts: port side given, starboard is the Y-mirror. Author both explicitly; do not rely on a
  mirror operation that breaks the joint tree.

## 1. Skill utilisation plan

Use the full toolchain, in this order:

1. **`cad`** — author all parametric solids. Export STEP + GLB (+ STL where useful).
2. **`urdf`** — build both robot descriptions, validate tree/limits/inertials.
3. **`srdf`** — define the named poses in section 8 and prune the collision pairs in section 9.
   This is how the deployment choreography becomes verifiable instead of hand-tracked.
4. **`sdf`** — export SDF for both bodies. Track B needs it; generate it now while geometry is fresh.
5. **`cad-viewer`** — review every checkpoint. Use the bundled MoveIt2 server to sanity-check the arm's
   reachability once R5 is done.

State which skill and which interpreter you are using at each step.

## 2. Global parameters (expose all of these)

| Parameter | Value | Notes |
|---|---|---|
| `wheel_diameter` | 0.50 m | radius 0.25 |
| `wheel_width` | 0.40 m | |
| `ground_clearance` | 0.60 m | chassis underside to ground |
| `chassis_length` | 3.00 m | X |
| `chassis_width` | 1.40 m | Y |
| `chassis_height` | 0.70 m | Z; deck top at ground z = 1.30 |
| `track_centreline` | ±0.90 m | overall width 2.20 m |
| `wheel_station_front` | +1.35 m | |
| `wheel_station_mid` | −0.05 m | |
| `wheel_station_rear` | −1.35 m | |
| `rocker_pivot` | (+0.25, ±0.72, +0.05) | rel `base_link` |
| `well_length` | 1.10 m | X |
| `well_width` | 1.00 m | Y |
| `well_depth` | 0.24 m | |
| `riser_travel` | 0.26 m | |
| `mast_height` | 1.30 m | above deck |
| `wing_span_deployed` | 1.56 m | |
| `wing_span_inner` | 0.84 m | fixed portion |
| `rotor_diameter` | 0.46 m | |
| `sweep_le` | 28° | leading-edge sweep |

## 3. Rover — link geometry

| Link | Primitive (R1) | Dimensions | Mass (kg) |
|---|---|---|---|
| `base_link` | box | 3.00 × 1.40 × 0.70 | 300 |
| `rocker_*` | 2 boxes (V-arm) | 0.10 × 0.10 sections | 18 |
| `bogie_*` | 2 boxes (V-arm) | 0.09 × 0.09 sections | 14 |
| `wheel_*_steer` | box (knuckle) | 0.14 × 0.12 × 0.20 | 4 |
| `wheel_*` | cylinder | d 0.50, w 0.40 | 12 |
| `mast_pan` | cylinder | d 0.14, h 0.10 | 6 |
| `mast_tilt` | tapered cylinder | d 0.12→0.09, h 1.30 | 12 |
| `camera_head` | box | 0.34 × 0.16 × 0.14 | 5 |
| `arm_shoulder_az` | cylinder | d 0.20, h 0.14 | 8 |
| `arm_shoulder_el` | box | 0.80 × 0.14 × 0.14 | 14 |
| `arm_elbow` | box | 0.70 × 0.12 × 0.12 | 10 |
| `arm_wrist` | box | 0.35 × 0.10 × 0.10 | 6 |
| `arm_turret` | cylinder | d 0.16, h 0.22 | 7 |
| `rtg` (fixed) | cylinder + fins | d 0.45, L 0.60 | 45 |
| `antenna` (fixed) | dish | d 0.30 | 5 |
| `helipad_frame` | well shell | 1.10 × 1.00 × 0.24 | 30 |
| `pad_panel_*` | box | 1.10 × 0.50 × 0.04 | 9 |
| `pad_riser` | telescopic column | d 0.16, stroke 0.26 | 11 |
| `latch_*` | box | 0.10 × 0.06 × 0.08 | 1 |

Fixed accessories: `rtg` at (−1.55, 0, +0.15) protruding aft; `antenna` at (−0.85, −0.55, +0.35).

## 4. Rover — joint table (authoritative)

| Joint | Type | Parent | Origin (rel. parent) | Axis | Limits | Vel / Eff |
|---|---|---|---|---|---|---|
| `rocker_port` | revolute | `base_link` | (+0.25, +0.72, +0.05) | Y | ±35° | 0.5 / 400 |
| `rocker_stbd` | revolute | `base_link` | (+0.25, −0.72, +0.05) | Y | ±35° | 0.5 / 400 |
| `bogie_port` | revolute | `rocker_port` | (−0.90, 0, −0.30) | Y | ±35° | 0.5 / 300 |
| `bogie_stbd` | revolute | `rocker_stbd` | (−0.90, 0, −0.30) | Y | ±35° | 0.5 / 300 |
| `steer_fl` | revolute | `rocker_port` | (+1.10, +0.18, −0.75) | Z | ±45° | 0.6 / 150 |
| `steer_fr` | revolute | `rocker_stbd` | (+1.10, −0.18, −0.75) | Z | ±45° | 0.6 / 150 |
| `steer_rl` | revolute | `bogie_port` | (−0.70, +0.18, −0.45) | Z | ±45° | 0.6 / 150 |
| `steer_rr` | revolute | `bogie_stbd` | (−0.70, −0.18, −0.45) | Z | ±45° | 0.6 / 150 |
| `drive_fl` | continuous | `steer_fl` | (0, 0, 0) | Y | — | 2.0 / 250 |
| `drive_fr` | continuous | `steer_fr` | (0, 0, 0) | Y | — | 2.0 / 250 |
| `drive_ml` | continuous | `bogie_port` | (+0.60, +0.18, −0.45) | Y | — | 2.0 / 250 |
| `drive_mr` | continuous | `bogie_stbd` | (+0.60, −0.18, −0.45) | Y | — | 2.0 / 250 |
| `drive_rl` | continuous | `steer_rl` | (0, 0, 0) | Y | — | 2.0 / 250 |
| `drive_rr` | continuous | `steer_rr` | (0, 0, 0) | Y | — | 2.0 / 250 |
| `mast_pan` | revolute | `base_link` | (−1.15, 0, +0.35) | Z | ±180° | 0.8 / 40 |
| `mast_tilt` | revolute | `mast_pan` | (0, 0, +1.30) | Y | −90°…+45° | 0.8 / 30 |
| `arm_shoulder_az` | revolute | `base_link` | (+1.40, 0, −0.10) | Z | ±120° | 0.5 / 200 |
| `arm_shoulder_el` | revolute | `arm_shoulder_az` | (0, 0, +0.10) | Y | −30°…+100° | 0.5 / 250 |
| `arm_elbow` | revolute | `arm_shoulder_el` | (+0.80, 0, 0) | Y | −140°…+10° | 0.5 / 180 |
| `arm_wrist` | revolute | `arm_elbow` | (+0.70, 0, 0) | Y | ±100° | 0.6 / 90 |
| `arm_turret` | continuous | `arm_wrist` | (+0.35, 0, 0) | X | — | 1.5 / 40 |
| `helipad_frame` | fixed | `base_link` | (+0.30, 0, +0.35) | — | — | — |
| `pad_panel_port` | revolute | `helipad_frame` | (0, +0.50, 0) | X | 0°…+180° | 0.15 / 120 |
| `pad_panel_stbd` | revolute | `helipad_frame` | (0, −0.50, 0) | X | 0°…−180° | 0.15 / 120 |
| `pad_riser` | prismatic | `helipad_frame` | (0, 0, −0.24) | Z | 0…0.26 m | 0.08 / 300 |
| `pad_mount` | fixed | `pad_riser` | (0, 0, +0.02) | — | — | — |
| `latch_fore` | revolute | `helipad_frame` | (+0.50, 0, −0.20) | Y | 0°…+80° | 1.0 / 25 |
| `latch_aft` | revolute | `helipad_frame` | (−0.50, 0, −0.20) | Y | 0°…−80° | 1.0 / 25 |

**Verification the design closes** (check this yourself and confirm):
front wheel absolute = rocker (+0.25, ±0.72, +0.05) + (+1.10, ±0.18, −0.75) = **(+1.35, ±0.90, −0.70)** → ground z = 0.25 = wheel radius ✔
middle = (+0.25 − 0.90 + 0.60, ±0.90, +0.05 − 0.30 − 0.45) = **(−0.05, ±0.90, −0.70)** ✔
rear = (+0.25 − 0.90 − 0.70, ±0.90, −0.70) = **(−1.35, ±0.90, −0.70)** ✔

Note the deliberate asymmetry: **middle wheels are not steerable**, matching real rover practice.

**Closed-loop warning.** Real rocker-bogie rovers couple port and starboard rockers with a differential
bar so the chassis bisects their angles. That is a closed kinematic loop and URDF is strictly a tree —
do not author it. Leave both rockers independent, and put an explicit comment in the URDF stating the
differential must be enforced by a simulator constraint or controller in Track B.

## 5. The helipad — reveal mechanism

Design intent: on landing the deck must read as **solid and uninterrupted**. Panel seams should look like
ordinary structural panel lines. Nothing in the silhouette may suggest a cavity.

- **Panels hinge at their outer edges** (y = ±0.50) and close inward to the centreline. Each is
  1.10 × 0.50 × 0.04.
- **They rotate a full 180° outward**, so the face that was pointing down into the well ends up facing
  up and flush with the deck. **Deployed pad = 2.00 m (Y) × 1.10 m (X)** — nearly double the stowed area.
- Those inner faces are seen close-up. Detail them: perimeter border, centre circle-and-cross marking,
  shallow non-slip tread, and four tie-down points.
- **Well interior** is visible once open: transverse ribbing, the riser column, harness runs, and the two
  hold-down latches. Floor at z_rel = +0.11 relative to `base_link`.
- **Riser** is a visible telescopic column, not an invisible translation. 0.26 m stroke lifts the drone
  from well floor to 0.02 m above deck level.

**Clearance audit — verify and report actual numbers:**

| Check | Expected |
|---|---|
| Panel swept envelope (X range) | −0.25 … +0.85 |
| Panel max height above deck mid-swing | 0.50 m |
| Panel tip outboard at full open | y = ±1.00 |
| Clearance to `mast_pan` (x = −1.15) | 0.90 m aft ✔ |
| Clearance to `antenna` (x = −0.85) | 0.60 m aft ✔ |
| Clearance to `arm_shoulder_az` (x = +1.40) | 0.55 m fwd ✔ |
| Panel tip to wheel top (z 0.50 ground) | 0.80 m ✔ |

## 6. The drone — tilt-rotor blended-wing-body

**Why this configuration.** A pure flying wing is fixed-wing: it makes lift from forward airspeed and
**cannot hover**. Mars' surface atmosphere is roughly 1% of Earth's density, which further penalises
fixed-wing flight — the reason NASA flew a rotorcraft, not an aeroplane. A tilt-rotor blended-wing-body
resolves it: rotors vertical for hover, then tilting forward to transition to wing-borne cruise. The
silhouette is preserved, the hover is real, and the transition is a better visual than a plain liftoff.

Original blended-wing-body: swept centre section blending smoothly into the wing, **no vertical tail, no
distinct fuselage.** This is not a replica of any specific aircraft.

### Link geometry

| Link | Dimensions | Mass (kg) |
|---|---|---|
| `drone_base_link` | blended body, 0.70 L × 0.34 W × 0.16 T | 1.20 |
| `wing_inner_port/stbd` | fixed, span to y = ±0.42, chord 0.55 root → 0.34 tip | 0.22 ea |
| `wingtip_port/stbd` | span 0.36, chord 0.34 → 0.20, washout twist −3° at tip | 0.14 ea |
| `nacelle_fwd_*` | 0.18 L × d 0.08 | 0.10 ea |
| `nacelle_aft_*` | 0.18 L × d 0.08 | 0.10 ea |
| `rotor_*` | 2-blade, d 0.46, tapered with visible twist | 0.07 ea |
| `elevon_port/stbd` | 0.32 × 0.09 × 0.015 | 0.05 ea |
| `sensor_pod` | 0.14 × 0.10 × 0.09 | 0.16 |
| `skid_port/stbd` | fixed, 0.24 L | 0.06 ea |

Total ≈ **2.9 kg**.

### Joint table

| Joint | Type | Parent | Origin | Axis | Limits |
|---|---|---|---|---|---|
| `wing_fold_port` | revolute | `wing_inner_port` | (−0.05, +0.42, 0) | **Z** | 0°…+90° |
| `wing_fold_stbd` | revolute | `wing_inner_stbd` | (−0.05, −0.42, 0) | **Z** | 0°…−90° |
| `tilt_fwd_port` | revolute | `wing_inner_port` | (+0.16, +0.26, +0.02) | Y | 0°…90° |
| `tilt_fwd_stbd` | revolute | `wing_inner_stbd` | (+0.16, −0.26, +0.02) | Y | 0°…90° |
| `tilt_aft_port` | revolute | `wing_inner_port` | (−0.20, +0.34, +0.02) | Y | 0°…90° |
| `tilt_aft_stbd` | revolute | `wing_inner_stbd` | (−0.20, −0.34, +0.02) | Y | 0°…90° |
| `rotor_fwd_port` | continuous | `nacelle_fwd_port` | (0, 0, +0.09) | Z (nacelle-local) | — |
| *(same pattern for the other three rotors)* | | | | | |
| `elevon_port` | revolute | `wing_inner_port` | (−0.26, +0.30, 0) | Y | ±25° |
| `elevon_stbd` | revolute | `wing_inner_stbd` | (−0.26, −0.30, 0) | Y | ±25° |
| `sensor_pod` | revolute | `drone_base_link` | (+0.20, 0, −0.09) | Y | −90°…0° |

**Tilt convention: 0° = thrust vertical (hover), 90° = thrust forward (cruise).**

### The fold axis is Z, and that is deliberate

The wing panels fold **in-plane, sweeping aft** — not upward. Reason: the well is only 0.24 m deep, so a
0.36 m panel folded vertically would stick out 0.12 m above the deck and break the flush-deck reveal that
the whole mechanism exists for. Folding aft keeps the stowed package flat. Confirm this reasoning holds
when you compute the actual envelope.

### Stowed-fit budget — hard constraint

Stowed pose: wing folds 0°, tilts 0°, **rotor blades parked fore-aft** (blades aligned with X, so each
rotor's Y footprint is just its hub — no extra fold mechanism needed, the continuous rotor joint parks
there).

| Axis | Computed | Well interior | Clearance |
|---|---|---|---|
| Y | 0.84 (inner wing span) | 1.00 | 0.08 each side |
| X | ~0.76 (body 0.70, folded tips reach −0.41) | 1.10 | ~0.17 each end |
| Z | 0.20 | 0.24 | 0.04 |

**Verify these geometrically and report the actual measured clearances.** If it does not fit, tell me
with numbers — do **not** silently shrink the wingspan or the rotors.

### Rotor interference check (deployed)

Forward rotors at y = ±0.26, radius 0.23 → inner edges at y = ±0.03, gap **0.06 m** across the
centreline. Aft rotors at y = ±0.34 sit at a different X station, so fore/aft discs do not overlap.
Confirm both.

## 7. Materials

Render quality comes from the viewer's PBR lighting, not geometry. **GLB is the appearance-critical
export; STEP carries no PBR data.** Assign per group, with meaningfully varied metalness/roughness:

| Group | Appearance |
|---|---|
| Chassis panels | brushed aluminium, mid roughness |
| Joint housings, rocker arms | dark anodised, low roughness |
| Camera bodies, apertures | matte black, high roughness |
| Thermal blanket accents | gold/amber, low roughness, high metalness |
| Wheel grousers | dark rubber-like, high roughness, near-zero metalness |
| Helipad markings | high-contrast white/orange, matte |
| Drone skin | pale grey composite, satin |
| Rotor blades | dark carbon-like |
| Lower surfaces | slight warm dust tint overlay |

Uniform material assignment is precisely what makes CAD models look like CAD models rather than hardware.

## 8. SRDF named poses — author these

This is why we use the `srdf` skill. Define joint groups (`suspension`, `steering`, `drive`, `mast`,
`arm`, `helipad`, `wings`, `tilts`, `rotors`) and these named states:

**Rover:** `landed_sealed` (all zero) · `latches_released` · `pad_open` (panels ±180°) ·
`pad_raised` (riser 0.26) · `mast_deployed` · `arm_stowed` · `arm_extended`

**Drone:** `stowed` (folds 0, tilts 0, blades fore-aft) · `wings_deployed` (folds ±90°) ·
`hover` (tilts 0°, rotors spinning) · `transition` (tilts 45°) · `cruise` (tilts 90°)

## 9. Collision pairs to disable in SRDF

Adjacent-by-construction pairs that will always report contact: each rocker↔chassis, bogie↔rocker,
knuckle↔rocker/bogie, wheel↔knuckle, mast_tilt↔mast_pan, consecutive arm segments, panel↔helipad_frame,
riser↔helipad_frame, latch↔helipad_frame, wingtip↔wing_inner, nacelle↔wing_inner, rotor↔nacelle.
Keep **enabled**: panel↔mast, panel↔arm, panel↔wheel, drone↔any rover link — those are the ones that
actually matter.

## 10. Deployment choreography

Track B animates this. Track A must prove the joint ranges permit it, collision-free.

| # | Step | Joints |
|---|---|---|
| 1 | Landed, deck flush, nothing visible | all zero |
| 2 | Hold-downs release | `latch_fore`, `latch_aft` |
| 3 | Panels swing outward 180°, becoming the pad. **Slow — 0.15 rad/s.** | `pad_panel_*` |
| 4 | Riser lifts drone to pad level | `pad_riser` |
| 5 | Wings unfold aft→spanwise | `wing_fold_*` |
| 6 | Rotors spin up, tilts confirm vertical | 4 rotors, 4 tilts |
| 7 | Vertical lift-off, hover | — |
| 8 | Tilts rotate to 90°, transition to cruise | 4 tilts |
| 9 | Departs | — |

Steps 3→5 are the money shot: solid deck opens into a wide pad, revealing a folded aircraft that spreads
its wings. Compose it to read from a mid-distance camera.

## 11. Build phases — stop at each checkpoint

- **R1 — rover skeleton.** Every joint from section 4, primitives only, exact origins/axes/limits.
  *Checkpoint: I articulate all 28 joints and confirm none rotates about a wrong axis. Nothing proceeds
  until this is clean.*
- **R2 — helipad mechanism.** Panels, riser, latches, well interior, `pad_mount`. Run the section 5
  clearance audit and report the table filled in with measured values.
- **R3 — wheels and suspension detail.** Grousers (parametric count), sidewall flexure pattern, arm housings.
- **R4 — chassis, deck, mast, arm detail.** Panel lines, chamfers, camera housings, RTG with fins,
  antenna, harness runs. *Checkpoint: stowed deck reads as solid.*
- **R5 — drone.** Section 6, then the stowed-fit and rotor-interference verifications.
- **R6 — SRDF.** Sections 8 and 9.
- **R7 — materials.** Section 7.
- **R8 — SDF export + full choreography validation.** Walk all nine steps, report clearances.

Report at each checkpoint: link count, joint count, rebuild time, failed solids.

## 12. Anti-goals

- No slicing, G-code, print jobs, or fabrication skills. Model files only.
- No simulator, physics, controllers, or perception code. The brain is a later phase.
- Do not convert the reference STEP into our model — it has no kinematics, and this design replaces it.
- Do not gold-plate before R1 validates.
- Do not silently alter a specified dimension. Name the conflict, show the arithmetic, propose the fix.

## 13. Start here

1. Confirm which skills are linked and which interpreter you will use.
2. Re-derive the three wheel-position closure checks in section 4 and confirm they land where stated.
3. Raise any internal contradiction you find in this document **before** authoring geometry.
4. Build R1 only, then stop.
