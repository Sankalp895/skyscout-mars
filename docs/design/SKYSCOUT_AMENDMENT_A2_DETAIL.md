# SkyScout — Design Amendment A2: Reference-Informed Detail Language

**Applies on top of `SKYSCOUT_DESIGN_v3.md` and `SKYSCOUT_AMENDMENT_A1_PROPORTIONS.md`.**
A1 fixed proportions. A2 fixes **part-level richness** — the reason the reference reads as hardware.

## A2.0 Provenance — read this first

`models/reference/Lunar_Rover_v35.step` is third-party geometry (Autodesk export, 2022, author not us).
We take **design language and proportions** from it. We do **not** copy geometry, part layout, or part
names, and we do not convert or re-export it. Our model stays an original design. This matters because
the finished project is a portfolio piece.

## A2.1 Measure the reference yourself — my parse was inconclusive

A text-level parse of the STEP established these facts reliably:

- 40 MB, millimetre units, **255 solids**, **818 product definitions**, 15,834 faces, 202,093 points.
- Overall bounding box ≈ 2100 × 3730 × 4620 mm, but **the axis convention did not resolve cleanly** —
  mirrored feature pairs do not sit where wheels should. Treat the axis assignment as unknown.
- Named parts: `Fuselage`, `Suspension`, `Suspension (Mirror)`, `Strut`, `Strut_1/2/3`, `Tires`,
  `Tires_1/2`, `Tires(Mirror)`, `Wheelsb`, `Wheelsb_1/2`, `RTG6`, `RTG6_1`, `Mast Cam-Z`, `Mast Cam-Z_1`,
  `Wide NAv cam`, `rearcam`, `Ripper and Arm`, `Jackhammer`, `Push Arm`, `Rivets`, plus ~100 generic
  `ComponentNN` parts.
- One cylinder family repeats **461 times** — consistent with the `Rivets` part being an array of
  fasteners.

**Your task:** load the STEP with OpenCASCADE via the `cad` skill and report, from real geometry:

1. Which axis is vertical, which is longitudinal, which is lateral.
2. Per-solid bounding boxes for `Fuselage`, `Tires`, `Wheelsb`, `Strut`, `RTG6`, `Mast Cam-Z`.
3. Wheel count, wheel diameter, wheel width, track width, and the wheelbase stations.
4. A **ratio table**: wheel diameter ÷ body length, track ÷ body width, ground clearance ÷ wheel
   diameter, mast height ÷ body length, body length ÷ body height.

Then compare that ratio table against A1's revised numbers and **tell me where A1 is off.** If a ratio
disagrees materially, propose the corrected value with the arithmetic. Do not silently adopt either.

## A2.2 The core lesson: part count, not surfacing

The reference is 255 solids. My design at R1 is roughly 30 primitives. It does not look like hardware
because it is not *assembled* like hardware — it is carved from single blocks.

**Principle for R3 onward: build each subassembly from discrete parts with visible joints between them,
rather than one solid with detail engraved on it.** A wheel is a rim plus a tire plus a hub plus fasteners
— four bodies — not one cylinder with grooves.

Target roughly **120–180 solids** for the finished rover. Below ~80 it will still read as a toy; above
~250 rebuild times on `/mnt/c` become painful.

## A2.3 Specific changes to adopt

### 1. Wheels — two bodies, not one
Split every wheel into **rim** and **tire**:
- **Rim:** inner drum, diameter 0.40, with a visible hub boss (0.18 dia, 0.06 proud) and **6 lightening
  cutouts** or spoke flexures between hub and rim.
- **Tire:** a separate shell over the rim, outer diameter 0.58, wall 0.02, carrying **20 grousers**
  (parametric count) as raised cleats 0.012 proud.
- Assign **different materials** to rim and tire (see A2.4). This single split does more for realism than
  any amount of surface detail, because the material break reads instantly at distance.

### 2. Suspension — discrete struts, not just tubes
A1 called for tube-and-strut trusses. Make the struts **separate named bodies**:
- Each rocker and bogie: 2 main tubes + **1 diagonal strut** + pivot bosses at both ends.
- Each strut gets a visible **end clevis** (a forked fitting, 0.06 wide) where it meets a boss.
- Mirror port/starboard as explicitly authored parts, not a mirror operation on the joint tree.

### 3. Two RTGs, not one
Replace the single `rtg` with **`rtg_port` and `rtg_stbd`**, each 0.32 dia × 0.52 long with 8 radial
fins, mounted on the raised aft deck shoulder at y = ±0.30, canted 15° nose-up. Two units add rear
visual mass and break the single-cylinder blandness.

### 4. Four camera stations, not one
- `camera_head` on the mast: **two stereo barrels** 0.24 apart (as in A1), each barrel a separate body
  with a visible lens hood.
- **`nav_cam_wide`** — fixed, on the forward deck edge at (+0.95, 0, +0.31), a wide short barrel.
- **`rear_cam`** — fixed, on the aft face at (−1.20, +0.20, +0.10), angled down 20°.
- Each camera is a housing body + a barrel body + a dark lens face. Three parts each.

Sensor multiplicity is a strong "this is a real spacecraft" cue and it is nearly free geometry.

### 5. Excavation end effector instead of a generic turret
Replace `arm_turret`'s bare cylinder with a **ripper/scoop head**: a turret drum (0.16 dia) carrying a
**ripper tine** (a curved tooth, 0.18 long) on one face and a **percussive tool barrel** (0.06 dia,
0.16 long) on the opposite face. This suits a Mars ISRU / regolith-sampling story better than a vague
tool mount, and it gives the arm a recognisable silhouette when parked.

Keep the joint tree unchanged — this is geometry hung on the existing `arm_turret` link.

### 6. Fastener arrays — the highest-value detail per unit effort
Add a parametric fastener generator and apply it at:
- Panel seams on the chassis: rings of bolts at **0.09 m pitch**.
- Every suspension pivot boss: **6-bolt** circular pattern.
- RTG mounting flanges: **8-bolt** rings.
- Rim-to-hub interface: **6-bolt** ring per wheel.

Expose `fastener_diameter` (default 0.012) and `fastener_pitch`. Model each as a simple hex head, 0.004
proud — deliberately crude, because they are read as texture, never inspected. The reference carries
hundreds of these and it is a large part of why it looks manufactured.

**Budget guard:** fasteners are the one place part count can explode. Cap the total at ~400 fastener
instances and report the count. If rebuild time exceeds ~3 minutes, reduce pitch density and tell me.

### 7. Soften the body — "fuselage", not "crate"
The reference names its main body `Fuselage`, which is a design cue worth taking. Amend A1's faceted
chassis:
- Keep the 20° raked nose, the chamfered lower rails, and the stepped aft deck.
- **Add radiused vertical shoulders**: fillet the four vertical body corners at **R 0.10**, and the top
  deck edges at **R 0.04**.
- Add **two recessed side panels** per flank (inset 0.015) framed by raised ribs, so the flank has
  structure rather than being a flat plane.

The silhouette should read as a purpose-built enclosure, not a shipping container.

## A2.4 Material assignments — updated

Split materials so the new part boundaries actually show. Replaces the v3 section 7 table:

| Part group | Appearance |
|---|---|
| Chassis panels | brushed aluminium, roughness 0.45, metalness 0.85 |
| Recessed side panels | same aluminium, roughness 0.60 (a shade duller — reads as inset) |
| **Wheel rims** | **light machined alloy, roughness 0.35, metalness 0.9** |
| **Tires + grousers** | **near-black, roughness 0.85, metalness 0.05** |
| Suspension tubes, struts, clevises | dark anodised, roughness 0.40, metalness 0.8 |
| Pivot bosses | mid-grey steel, roughness 0.3, metalness 0.95 |
| Fasteners | bright steel, roughness 0.25, metalness 1.0 |
| RTG bodies | dull graphite, roughness 0.7 |
| RTG fins | brighter alloy, roughness 0.35 |
| Camera housings | matte black, roughness 0.9 |
| Lens faces | very dark, roughness 0.05, metalness 0.6 (reads as glass) |
| Thermal blanket accents | gold/amber, roughness 0.3, metalness 0.9 |
| Helipad markings | high-contrast white/orange, matte |
| Ripper tine | worn bright steel, roughness 0.3 |
| Lower surfaces | warm dust tint overlay, additive |

The fastener/boss/tube materials being *distinct* is what makes the truss read as assembled hardware.

## A2.5 Revised phase plan

R1 and R2 are unchanged (A1 governs). From R3 on:

- **R3 — wheels as assemblies.** Rim + tire + hub + fasteners, 6×. Report solid count.
- **R4 — suspension as assemblies.** Tubes + struts + clevises + bosses + fasteners.
- **R5 — body.** A2.3 item 7, plus panel seams and their fastener rings.
- **R6 — sensors and payload.** Four camera stations, two RTGs, antenna, ripper head, harness runs.
- **R7 — drone** (v3 section 6, unchanged).
- **R8 — SRDF** named poses and collision pruning.
- **R9 — materials** per A2.4.
- **R10 — SDF export + full choreography validation.**

Report at every checkpoint: **solid count**, link count, joint count, rebuild time, failed solids.

## A2.6 Unchanged

- All 28 joints, all origins from A1, the helipad mechanism, the drone, the deployment choreography.
- Every anti-goal from v3 section 12. In particular: no fabrication, no simulator, no controllers, and
  do not convert the reference STEP into our model.

## A2.7 Do this now

1. Run the OpenCASCADE measurement in A2.1 and give me the ratio table plus where A1 disagrees.
2. Wait for my confirmation on any proportion change before rebuilding.
3. Then proceed from whichever phase we agree on.
