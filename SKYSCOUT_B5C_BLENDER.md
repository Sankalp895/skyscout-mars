# SkyScout — B5 Part C: Blender Cinematic Pass

The functional system is complete and verified end to end: the drone flies a GPS-denied unknown Mars
world, perceives terrain from its own camera, builds a hazard map, and the rover drives the safe routes
it found — all shown on the mission HUD. This phase is PRESENTATION, not engineering: turn the verified
motion into cinematic, beautiful renders for the portfolio / LinkedIn / the "impress NASA a bit" goal.
Keep it cleanly separate from the functional code — nothing here changes the sim or the results.

Context if fresh: read `SKYSCOUT_RESUME.md`. Machine is AMD Ryzen 7 5700U, integrated graphics, WSL2,
NO GPU. Blender's Cycles renderer runs on CPU here — slower per frame but GPU-independent and genuinely
film-grade; that's the whole reason beauty was deferred to Blender instead of the sim. Every phase logged
trajectories (drone + rover poses over time) and there are GLB geometry exports from the body phase.

## C.0 Principle
- This is the FIRST time materials/lighting matter. The deferred ~16-group materials pass (from the body
  work) happens HERE, on the Blender side, not back in CAD.
- Source of truth for motion = the recorded trajectory logs. Do NOT re-simulate; import the geometry and
  drive it along the logged poses so the beauty render matches the verified run exactly.
- CPU Cycles: budget time. Render at a sane resolution (1080p), use adaptive sampling + denoising to keep
  per-frame cost survivable, and start with a few hero STILLS before committing to a full animation.

## C.1 Import + rig
- Import the drone and rover GLB geometry into Blender (the body-phase exports; if only STL/STEP exist,
  convert). Preserve the link hierarchy so joints can be posed.
- Write an importer that reads the trajectory JSON logs and keyframes each body's base pose + joint
  angles per timestep, so the geometry moves exactly as the verified sim ran (the reveal fold sequence,
  the flight, the rover drive).
- Confirm one static frame poses correctly (drone deployed, rover on terrain) before animating.

## C.2 The Mars world
- Build the terrain mesh from the SAME heightfield the sim used (seed 20260808, 80×80 m) so the rendered
  ground matches the mission exactly — the crater, ridge, and boulders line up with where the drone
  actually flew.
- Materials: reddish basalt/regolith for ground, a dust tint, distinct materials for the drone (composite
  body, dark rotors, metallic nacelles) and rover (the ~16 groups: chassis, wheels/grousers, RTG, mast,
  cameras — the deferred pass). PBR, varied roughness/metalness so parts read as hardware.
- Lighting: low, warm Martian sun (long shadows), a dim pinkish sky/ambient. This single choice is 80% of
  "looks like Mars."

## C.3 The hero shots (stills first, then clips)
Produce these as stills for review, THEN render the approved ones as short clips:
1. **The reveal:** rover on the surface, deck panels opening, riser lifting the drone, wings unfolding —
   the choreography from the body phase, now lit and materialed. This is the signature shot.
2. **Drone in flight:** low pass over the crater/ridge, Martian sun behind it.
3. **The scout-and-drive story:** a wide shot conveying the drone surveying while the rover waits, then
   drives. (Can pair with the functional HUD as a picture-in-picture in editing, later.)
4. **A detail beauty still:** the rover parked, mast + cameras + wheels catching the light — the CAD
   craftsmanship on display.

## C.4 Output + workflow
- Render hero STILLS first (cheap, fast feedback). Report render time per still so we can judge whether
  full animation is affordable on CPU, or whether we do stills + a few short clips only.
- Keep a .blend file + a render script (headless `blender -b -P script.py` so it can run without the GUI,
  which suits this machine).
- Output PNGs / MP4 to a clear folder. These are the portfolio deliverables.

## C.5 Checkpoint
Produce the 4 hero stills, report per-frame render cost, and confirm the geometry poses match the logged
mission (spot-check: drone deployed-pose matches the sim's deployed pose; rover sits on the terrain at
its logged position). STOP for review before rendering any full animation — CPU render time makes a full
clip a real commitment, so we decide scope from the still costs.

## C.6 Anti-goals
- Do NOT change or re-run the functional sim; this consumes logs only.
- Don't chase real-time; Cycles is offline by nature — that's fine, it's why it looks good.
- Don't render a full 43-second animation blind before seeing still costs and quality.
- This is the LAST build phase — after this, the remaining work is the write-up / LinkedIn narrative,
  not more engineering.
