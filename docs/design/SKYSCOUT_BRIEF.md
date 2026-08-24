# SkyScout — Project Brief for Claude Code

Read this file at the start of every session. It is the source of truth for scope and environment.

## 1. What we are building

**SkyScout** — a Mars aerial-scout simulation. An autonomous rotorcraft (Mars-helicopter class) flies
ahead over Martian terrain, perceives the ground from above, builds a live hazard/traversability map,
and hands safe, ranked routes down to a ground rover that follows them.

The project has two tracks:

- **Track A (current):** author the drone and rover as parametric CAD + URDF using this repo's skills.
  These are the physical bodies that will later load into the simulator.
- **Track B (later, not yet designed):** the AI brain (aerial hazard/traversability perception, robust
  localization over low-texture terrain, route ranking) and the 3D simulation world + mission-control HUD.
  Sim stack is undecided (Three.js vs. a physics engine such as MuJoCo/Genesis). Do not start Track B
  work or scaffold a simulator unless explicitly asked.

**Why this framing matters:** the technical hook is that Ingenuity's mission ended due to navigation
errors over featureless terrain. The eventual AI contribution targets exactly that failure mode. Keep
engineering choices credible and physically plausible for Mars (thin atmosphere, low gravity, no GPS).

## 2. Hard scope constraints

- **Simulation only. No physical hardware.**
- **Do NOT use the fabrication half of this repo.** No slicing, no G-code, no printer/slicer CLIs,
  no Bambu Lab or LAN print handoff, no dry-run or live print jobs. Ignore those skills entirely.
- We use this repo for **one purpose**: exporting model files (STEP / GLB / URDF) that will feed a simulator.
- **Zero spend beyond an existing Claude subscription.** Do not suggest paid services, hosted APIs,
  or cloud compute. Everything must run locally on free/open-source tooling.
- Prefer free, open-source, locally-runnable dependencies for every recommendation.

## 3. Environment (verify, don't assume)

| Item | State |
|---|---|
| OS | Windows 11 host, working inside **WSL2 / Ubuntu 24.04.4 LTS** |
| Project path | `/mnt/c/Users/sanka/Desktop/Personal Project/Mars_rover/text-to-cad` (note: spaces in path — quote it) |
| Repo | `earthtojake/text-to-cad`, cloned on the **`main`** branch |
| Python | **3.12.3** (`python3`). There is no `python3.11` on this machine — do not call it. |
| venv | `.venv/` exists in the repo root, pip upgraded to 26.1.2 |
| Requirements file | **`requirements-dev.txt`** — the old `requirements-cad.txt` no longer exists in this repo |
| Node | v18.19.1, npm available |
| Editor | VS Code connected via the WSL remote extension |

**Install status is UNCERTAIN.** `./.venv/bin/python -m pip install -r requirements-dev.txt` may or may
not have completed. Verify before doing anything else — e.g. check that `cadpy` and `build123d` import
successfully from `./.venv/bin/python`. Report what you find rather than guessing.

Because the repo lives on `/mnt/c/...`, filesystem operations cross the Windows/Linux boundary and are
slower than native WSL paths. This is accepted; do not relocate the project.

## 4. Repo conventions to follow

- Read `AGENTS.md` and `CONTRIBUTING.md` in this repo before acting — they define the expected layout.
- Write all generated CAD / robot-description artifacts under **`models/`**. Do not invent artifact
  directories elsewhere.
- When running a skill's tooling manually, use that skill's own interpreter, e.g.
  `.venv/skills/cad/bin/python skills/cad/scripts/step --help`.
- The CAD viewer is a local browser app under `viewer/`.

## 5. Working style

- I am a B.Tech student in AI & Data Science. Strong on Python, ML, and simulation; **newer to CAD,
  URDF, and robotics tooling.** Explain unfamiliar concepts with brief analogies the first time they
  appear, then move on.
- Be direct and concise. No filler, no restating my request back to me.
- Work in small verifiable steps. After each step, tell me what to run and what a correct result looks like.
- **Ask before** long installs, large downloads, destructive commands (`rm -rf`), or anything touching
  files outside this repo.
- If an instruction here is stale or contradicts what you actually find on disk, say so instead of
  forcing it through.

## 6. Immediate task queue

1. **Verify the Python environment.** Confirm `requirements-dev.txt` installed cleanly and the CAD
   packages import. If it failed, diagnose and fix (a common miss is a system library such as
   `libgl1` needed by OpenCASCADE).
2. **Install viewer dependencies:** `npm --prefix viewer install`.
3. **Link the skills into Claude Code:** `scripts/install/install-skills.sh --agent claude`
   (destination is `~/.claude/skills`). Then tell me to restart Claude Code so it rescans.
4. **Start the viewer** and give me the URL to open.
5. **Author the drone.** Using the `cad` and `urdf` skills, create a parametric Mars aerial scout —
   a hexacopter — under `models/skyscout_drone/`:
   - Links: central body, six radial arms, a rotor hub at each arm tip, four landing legs,
     a downward-facing camera mast.
   - Joints: one continuous spin joint per rotor; one pitch joint for the camera mast.
   - Parameters: arm length, body radius, rotor diameter.
   - Exports: STEP, GLB, and URDF.
   - Validate the URDF (link/joint tree, joint limits, no collision-geometry errors).
6. **Author the rover** under `models/skyscout_rover/` once the drone is validated. Six wheels with
   continuous drive joints, steerable corner wheels, a sensor mast. Same export + validation set.

Keep this first pass **simple and correct** rather than detailed. The bodies only need to be
believable and correctly jointed so they load cleanly into a simulator — all the impressive behavior
lives in Track B. Do not gold-plate the geometry.

## 7. Definition of done for Track A

- `models/skyscout_drone/` and `models/skyscout_rover/` each contain a valid URDF plus STEP and GLB exports.
- Both load and render in the local CAD viewer without errors.
- Joint trees validate, and rotor/wheel joints actually articulate.
- No fabrication artifacts were produced anywhere.
