# SkyScout — Track B, Phase B5: The Rover Consumes the Map + Mission HUD

The last phase, and a different kind of work: B1–B4 *built* the intelligence (localization, planning,
perception). B5 *closes the loop and makes it legible*. Two deliverables: (1) the ground rover plans and
drives a safe route over the hazard map the DRONE built, and (2) a mission-control HUD that shows the
whole story on one screen. This is the concept's thesis made literal — **scout maps hazards from the
air, rover drives the safe route** — and the piece that turns a pile of correct plots into something a
person who never reads your code can understand in ten seconds.

Context if fresh: read `SKYSCOUT_RESUME.md`. Sim = Genesis headless, Mars gravity −3.72. DONE: B1 world;
B2 from-scratch ESKF (`get_estimated_state()`); B3 A* (`plan(tmap, start, goal)`, `TraversabilityMap`
takes a heightmap/cost array); B4 the drone builds a **perceived world hazard map** from its downward
camera (depth + GLCM-on-depth), with `unknown` kept distinct from `safe`. The rover URDF/SDF exists
(35 links/34 joints, rocker-bogie, 6 wheels, the deep helipad bay). Judge on plots + offline PNGs.
Carry every habit: hand-check each claim, judge in TRUTH space, report failure directions separately,
never let `unknown` read as `safe`.

## B5.0 The honest framing

The drone already flies, localizes, perceives, and maps (B1–B4). B5 does NOT re-derive any of that. It:
1. Takes the drone's finished **perceived hazard map** as the route surface for the rover.
2. Plans and drives the rover over it — a NEW planning problem because the rover is a ground vehicle with
   different constraints than the flying drone.
3. Presents everything on a mission HUD.

The rover's constraints differ from the drone's, and that difference is the real content of B5's Part A —
don't just reuse the drone's plan.

## B5.A — Rover planning + driving

### A1 — Rover traversability is NOT the drone's
The drone cared about *clearance to fly over*. The rover must *drive on the surface*, so its cost map is
different from the same terrain:
- **Slope** matters directly: the rover can climb gentle grades but not steep ones (pick a max drivable
  slope, e.g. ~25°, justify it against the rocker-bogie design).
- **Steps/obstacles:** a boulder the drone flew over is an absolute block for the rover; a small step the
  rover can climb (rocker-bogie handles ~a wheel radius) is fine. Wheel radius is known from the body.
- **`unknown` cells:** the rover must NOT drive into unmapped terrain — treat unknown as blocked (the
  drone can only route the rover through what it actually scouted). State whether this forces a
  scout-then-drive ordering (it should: the drone surveys first, the rover drives after).
Build `rover_cost_map(perceived_map, terrain)` producing the rover's own traversability from the drone's
perceived hazard map + sensed/known heights. **Checkpoint A1:** plot rover cost map vs drone hazard map
side by side; show where they DIFFER (a rock that's a fly-over for the drone but a hard block for the
rover; a slope flyable but not drivable). Report the differences explicitly. STOP.

### A2 — Plan and drive the rover
- A* (reuse B3's `plan`) over the rover cost map, from the rover's landing/start cell to a goal.
- Drive the rover along the path in Genesis. Two honesty levels — pick and state which:
  - **Kinematic** (simplest): move the base along the path, wheels turning for show. Fine for the demo.
  - **Physics-driven** (harder, and it's what the SDF/rocker-bogie was FOR): command wheel velocities,
    let Genesis's contact physics carry it over the terrain — this is the ONLY place the rocker-bogie
    differential and real ground contact get exercised (the long-deferred unverified plugin). If it's
    not too costly on this CPU, do at least a short physics-driven segment to finally test the
    differential; otherwise kinematic for the full route + one physics-driven clip, and say so.
**Checkpoint A2:** plot the rover's driven path over the rover cost map + the drone's survey track that
produced the map. Report: reached goal, stayed on drivable cells, and — if physics-driven — whether the
rocker-bogie kept the chassis level over a step (the differential's actual job). STOP.

## B5.B — Mission-control HUD

One screen that narrates the whole mission. This is the portfolio artifact — build it as an offline-
rendered sequence (PNMG frames → video), NOT a live window (llvmpipe can't). It composites the logs
every phase has been producing.

Show, ideally as a top-down map with side panels:
- The **terrain** (Mars).
- The **drone's survey track** and current position, with the **hazard map filling in** as it scouts
  (the map should visibly grow from unknown→mapped over time — the scout working).
- The **rover** waiting, then driving its planned route once the map is ready.
- **Telemetry panels:** drone ESKF position vs (dev-only) truth, localization error, coverage %, and a
  clear phase label (SURVEY → MAP READY → ROVER DRIVING → ARRIVED).
- A legend distinguishing unknown / safe / hazard so a viewer reads the map instantly.

**Checkpoint B:** produce the composited sequence (a handful of keyframe PNGs first for review, then the
full clip). It must be legible to someone who has never seen the project — the phase labels and the
growing map carry the story. STOP for review.

## B5.C — (optional, after B is approved) Blender beauty pass
Only once the HUD tells the story correctly: bring the geometry + the trajectory logs into Blender for
the cinematic shots — the deck-opening reveal, the drone unfolding and flying, a rover drive on a
lit Martian surface. This is where the deferred ~16-group materials pass finally happens. CPU-rendered
(Cycles) is fine — slow but GPU-independent. This is presentation, not engineering; keep it clearly
separate from the functional HUD. Do NOT start it until Checkpoint B is approved.

## B5.D — Validation + structure
- Judge rover arrival and hazard-clearance in TRUTH space (the rover on the true terrain), even though it
  planned on the perceived map — a perception miss could still route the rover at a real rock.
- Report the FULL stacked story once: drone localization error + perception/map error + rover-following
  error, vs the rover's safety margin. This is the whole system's honesty statement.
- Hand-check: the rover cost map differs from the drone's in the specific cells you'd predict (rocks
  block, gentle slopes open up), not everywhere and not nowhere.

## B5.E — Report and stop at each checkpoint
A1 (cost-map difference), A2 (driven route + differential test), B (HUD sequence), then optionally C.
This completes SkyScout end to end: a scout that flies an unknown GPS-denied world, maps it from its own
camera, and a rover that drives the safe routes it found — shown on one screen.

## B5.F — Anti-goals
- Don't re-derive B1–B4; consume their outputs.
- `unknown` is blocked for the rover — never silently drivable.
- Don't fake the rover planning by reusing the drone's flight path — the rover cost map is genuinely
  different and that difference is the point.
- No live-window reliance for the HUD — offline frames.
- Don't start the Blender pass (C) until the functional HUD (B) is approved.
