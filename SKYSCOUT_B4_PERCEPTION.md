# SkyScout — Track B, Phase B4: Perception (the drone builds its own hazard map)

Until now the planner (B3) was handed a **ground-truth** cost map computed from the known heightfield.
That's the cheat B4 removes. B4 gives the drone **eyes**: it looks down through its own camera, decides
which terrain is safe, and builds the hazard map *itself* — which the B3 planner then routes over. This
is the piece that makes SkyScout an actual **scout** rather than a drone flying a map someone already
drew. It is also the single most novel build in the brain and the one that leans directly on your
from-scratch GLCM texture-analysis work.

Context if fresh: read `SKYSCOUT_RESUME.md`. Sim = Genesis headless, Mars gravity −3.72. Done: B1 world +
`terrain_height`; B2 from-scratch ESKF (`get_estimated_state()`); B3 A* over a `TraversabilityMap` that
**already takes a heightmap array as input**, so B4's whole job is to *produce that array from the camera
instead of from ground truth*, and nothing downstream changes. Judge on plots/images, not the live view.

## B4.0 The core idea and the one honest rule

- The drone flies (on its B2 estimate, per B3), pointing a downward camera at the terrain.
- From camera frames it estimates, per ground patch: **is this traversable or hazardous?**
- It accumulates those judgements into a map in world coordinates → feeds `TraversabilityMap` → B3 plans
  over the **perceived** map.
- **The honest rule (carry it from B3):** perception is judged against ground truth, but the *system*
  must only ever ACT on what it perceived. The drone plans over the map it built, not the real one. We
  compare perceived-vs-true to score the perception; we never let truth leak into the flight decision.

## B4.1 Decide the sensing modality first — report before building

Genesis can render, from the drone's downward camera, some combination of: an **RGB image**, a **depth
image**, and a **segmentation mask** (ground-truth object IDs per pixel). Before writing perception, probe
what's actually available headless on this CPU build and report:

1. Which of RGB / depth / segmentation Genesis exposes from a mounted camera, and at what cost/frame
   (remember the ~86 s cold-shader warm-up; warm it first).
2. Your recommendation for the primary hazard cue, with the tradeoff stated:
   - **Depth-driven** (geometric): reconstruct local terrain height/slope from the depth image, threshold
     on slope + prominence exactly like B1's ground-truth cost map — but now from *sensed* depth. This is
     the most physically honest "the drone measured the shape of the ground" path.
   - **Texture/appearance-driven** (your GLCM strength): classify each patch of the RGB image as
     safe-ground vs rock/hazard from texture features (GLCM: contrast, energy, homogeneity, correlation),
     since hazards (rocks, rims) read as different texture than smooth regolith.
   - **Both fused:** depth for geometry, GLCM texture for appearance, combined into one hazard score.
**Checkpoint 0:** report available modalities + your recommended approach and why. STOP for my confirmation
before building the classifier — this choice shapes everything, and depth-vs-texture is a real fork.

## B4.2 Build order — checkpoints, stop and report at each

### Step 1 — Mount the camera, capture, and project to the ground
- Rigidly mount a downward camera on the drone (known offset/orientation from the body frame).
- Each capture: record the frame(s) AND the drone's **estimated** pose (B2) at that instant.
- Implement ground projection: for each pixel, using the camera intrinsics + the drone's estimated pose,
  compute where that pixel's ray hits the ground → a world (x, y) footprint per frame.
**Checkpoint 1:** fly a short leg, capture N frames, and plot each frame's projected ground footprint
over the true terrain map. Confirm footprints land where the camera was actually looking. Report the
footprint size at cruise altitude (how much ground one frame covers). Hand-check: at altitude h with
FOV θ, footprint ≈ 2·h·tan(θ/2) — verify the measured footprint matches. STOP.

### Step 2 — Per-patch hazard classification (the perception core)
Implement the modality chosen in B4.1 to output, per ground patch, a hazard score ∈ [0,1]:
- If depth-driven: sensed local slope + prominence → score (mirror B1's thresholds, but on sensed data).
- If texture-driven: compute GLCM features per patch, classify safe vs hazard. Start rule-based /
  thresholded on the GLCM features (you built GLCM from scratch before — reuse that understanding); only
  reach for a learned classifier if thresholds are insufficient, and say so.
- If fused: combine into one score with stated weights.
**Checkpoint 2:** on a set of static captures over known terrain, plot perceived hazard score vs the
ground-truth hazard label, patch by patch. Report a confusion matrix / precision-recall against truth:
how often it flags a real hazard (recall) and how often it false-alarms on safe ground (precision). This
is the "does the drone actually see hazards" gate. STOP.

### Step 3 — Accumulate into a world hazard map (mapping)
- Maintain a persistent world-frame grid (same resolution as B1's cost map). As frames arrive, fuse each
  patch's score into the grid cell it projects to (running average / Bayesian update / max — pick one,
  justify it). Unobserved cells stay "unknown."
- Handle overlap (a cell seen from multiple frames) and the fact that pose is the **estimated** pose, so
  projection carries the ESKF error — patches land slightly off. Report how much that smears the map.
**Checkpoint 3:** fly a lawnmower/survey pattern over a region containing the crater + a boulder, build
the map live, and plot the **perceived hazard map vs the B1 ground-truth cost map** side by side. Report:
coverage (% of region observed), and agreement (IoU or cell-agreement of perceived-hazard vs true-hazard).
STOP.

### Step 4 — Close the loop: plan over the PERCEIVED map
- Feed the perceived hazard map (Step 3) into `TraversabilityMap` → A* (B3) → fly it on the B2 estimate.
- The drone now scouts, builds its own map, plans over what it saw, and flies that plan — the full
  autonomy chain with **no ground truth anywhere in the decision path**.
**Checkpoint 4 — the B4 payoff:** one plot showing perceived hazard map + planned path (over perceived) +
TRUE trajectory. Report, in TRUTH space: did the true trajectory reach the goal; closest true approach to
a REAL hazard (the map was imperfect — did a perception miss route it into a genuine hazard?); and the
combined error budget statement (ESKF position error + perception mapping error vs the safety margin).
This is where perception error and localization error stack, so the margin math from B3 gets a second
term. STOP with the plot.

## B4.3 Validation discipline (same habits, higher stakes now)

- Perception fails in TWO directions and both are dangerous: a **miss** (real hazard scored safe → the
  drone flies into it) and a **false alarm** (safe ground scored hazard → the planner wastes range or
  declares no path). Report both rates every step; don't optimize one silently at the other's expense.
- Always plot perceived-vs-true; never report only an aggregate score (the recurring "right on one
  metric, wrong underneath" trap — an 80% cell-agreement can still miss the one boulder that matters).
- The map is built on ESTIMATED pose. Quantify the projection smear that the ESKF error injects — this is
  the honest coupling between B2 and B4.
- One hand-check per step (footprint geometry, a known-hazard patch scoring high, coverage arithmetic).

## B4.4 Structure for B5

- Keep perception a clean module: frames + estimated pose → hazard-score patches → world map. B5 (rover)
  will consume the **finished perceived map** as the route surface for the ground vehicle.
- The perceived world hazard map is the actual deliverable the drone hands to the rover — this is the
  "scout maps hazards, rover follows safe routes" concept made real. Log it to disk (array + PNG).
- Log frames, poses, per-frame footprints, and the evolving map to JSON/PNG for plotting and Blender.

## B4.5 Report and stop

Checkpoint 0 (modalities + recommendation) FIRST, and stop for confirmation. Then per step: the plot, the
hand-check, the miss/false-alarm numbers. Final deliverable: the Checkpoint-4 plot (perceived map + plan +
true trajectory) with the reach-goal / closest-real-hazard / stacked-error-budget numbers. Then stop —
B5 (rover consumes the perceived map + mission-control HUD) is the last phase.

## B4.6 Anti-goals

- No ground truth in the decision path — the drone plans and flies on what it PERCEIVED. Truth is only
  for scoring.
- Don't jump to a heavy deep-learning segmentation model first. Start with the geometric/GLCM approach
  that's explainable and that you can hand-verify; escalate only if it demonstrably fails, and say why.
- No rover yet (B5). No aerodynamics/actuator model (waypoint controller stand-in is fine).
- No Blender yet — but keep every log Blender-ready.
- Don't skip Checkpoint 0. The depth-vs-texture-vs-fused choice is a real fork and I want to weigh in
  before you build the classifier.
