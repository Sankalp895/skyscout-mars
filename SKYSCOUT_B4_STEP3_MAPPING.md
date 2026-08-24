# SkyScout — B4 Step 3: Accumulate into a World Hazard Map

Step 2 gave a per-patch hazard score from a single frame (depth + GLCM-on-depth), validated by a
confusion matrix against truth. Step 3 turns a *stream* of frames, captured along a survey flight, into
one persistent world-frame hazard map — the actual artifact the drone hands downstream. This is the
"scout builds the map" deliverable.

Carry every B4 rule: `capture()` uses TRUE body pose (physics — a bolted-on camera can't teleport to the
estimate); `project()` uses the ESTIMATE (interpretation); the gap between them is the map smear and it
is the honest B2↔B4 coupling, not a nuisance to hide. Judge on plots. One hand-check per claim. Report
both failure directions (miss vs false-alarm), never a lone aggregate.

## Setup
- Survey altitude 12 m (confirmed), straight-down camera, ~13.9 m footprint, cost-map-resolution grid
  matching B1. Fly a lawnmower/survey pattern (≈5 legs) over a 40×40 m region that contains the crater
  AND at least one boulder, so there are real hazards to map and open ground to not-flag.
- Fly on the B2 ESKF estimate (as B3 Step 4), 1 Hz fixes — the survey itself carries localization error.

## Build
1. **Persistent world grid.** Same resolution as B1's cost map, world-aligned. Each cell holds an
   accumulated hazard score + an observation count; unobserved cells = "unknown" (distinct from
   "observed-and-safe" — don't let unknown read as safe, that's a miss generator).
2. **Fusion rule.** When a frame's patch projects onto a cell, fuse its score into that cell. Pick ONE
   rule and justify it in one line: running average (smooths noise, dilutes a real spike seen once),
   max (never forgets a hazard, amplifies a single false alarm), or a Bayesian log-odds update
   (principled, needs a sensor model). State the choice and why; a Bayesian or max-biased rule is usually
   right for hazards because a MISS is the dangerous error and you'd rather over-remember a hazard.
3. **Overlap handling.** Cells seen by multiple frames from multiple poses: confirm the fusion behaves
   sensibly (repeat observations should sharpen, not thrash).
4. **Smear from estimated pose.** Because projection uses the estimate, patches land offset by the ESKF
   error (Checkpoint 1 measured ~1.0 cell). Quantify how much this blurs hazard edges in the final map.
   Fold in the small straight-down-camera attitude term flagged in Step 1 (note it, size it).

## Checkpoint 3 — report + plots
- Side-by-side: **perceived hazard map** vs **B1 ground-truth cost map**, same axes.
- **Coverage:** % of the 40×40 region observed (unknown cells named, not hidden).
- **Agreement in truth space:** over OBSERVED cells only, hazard-cell IoU (or precision/recall of
  perceived-hazard vs true-hazard). Report MISS rate (true hazard mapped safe) and FALSE-ALARM rate
  (true safe mapped hazard) separately — miss is the dangerous one, weight it.
- **Edge smear:** how far, in metres/cells, hazard boundaries in the perceived map are displaced/blurred
  vs truth, and how much of that traces to ESKF projection error.
- Hand-check: coverage arithmetic (legs × footprint vs region), and confirm at least one known boulder
  and the crater rim appear as hazards in the perceived map with the count you'd expect from how many
  frames saw them.

## Structure for Step 4 / B5
- Perceived map is a clean array + PNG on disk — this is what Step 4 plans over and what B5's rover
  consumes. Keep `unknown` explicit in the format so the planner can treat unknown≠safe.
- Keep the fusion module swappable and its sensor model in nav_params.

## Anti-goals
- No ground truth in the map itself — truth is only for the scoring plots. The drone maps what it saw.
- `unknown` must never silently become `safe`.
- No planning-over-perceived yet (that's Step 4), no rover (B5), no aerodynamics, no Blender.
- Don't report only an aggregate agreement %; the miss rate on real hazards is the number that matters.

Stop at the checkpoint with the side-by-side plot and the miss / false-alarm / coverage / smear numbers.
