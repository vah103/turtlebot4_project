# MapEx G2-V1 R2 — Adaptive Room Viewpoint Sequence Planner

Standalone **offline/shadow** room-scoped planner, not connected to native robot control, G3 or MapEx's route executor. R2 corrects the three failures diagnosed in D02 without imposing any desired D02 goal count.

## Input (unchanged from V1)

- `observed` occupancy raster: `0=observed free, 1=unknown, 2=observed occupied`
- Current LaMa `mean`, `variance` (and optionally actual member maps `ensemble[K,H,W]`)
- Current G1 V4-R4.1 `semantic` map (`1=room`) and robot core-coordinate `(row,col)`
- Optional current frontier list, map resolution / LiDAR / navigation and planner configuration

Only use information available at the current decision. **No GT, future observation or later-run LaMa** may enter goal selection. The Room Node mask is the observed-nonwall, 4-connected `semantic==1` component containing the robot. Routes are on observed-free grid cells only, with a conservative clearance proxy; no goal may traverse prediction-only free.

## R2 pipeline

1. **Visible-by-scenario.** Use each supplied LaMa member map as a separate, thresholded geometry scenario. A binary LiDAR ray traces first hit; a target unknown cell counted at most once in *each scenario for the entire path*, regardless of how many beams or viewpoints observe it. If only mean/variance are available, use two explicitly **uncalibrated** proxies `mean<0.5` and `(mean+sigma*sqrt(variance))<0.5`; **never pretend these are member maps**. Unknown cells predicted walls can be observed as wall hits, but unknown cells can never be traversed. The score is weighted by uncertainty and averaged across scenario maps, with a separate spread/risk term. This is a nonprobabilistically-calibrated proxy for information gain, not entropy reduction measured from a real posterior.
2. **Route-first candidate evaluation.** Build a clearance-checked reachable vantage sample, optional frontier-adjacent hints, and use the **LiDAR union along shortest paths** to score *every one-goal candidate before pruning*. Preserve spatially diverse alternatives for the multi-goal beam search. This replaces V1's stationary-view top-12 prefilter. The first-stage search can still miss unsampled candidate locations, or beneficial future-condition-only viewpoints. Results are approximate.
3. **Conditional sequence optimization.** The number of goals is not forced to 1/2/3. Beam search considers complete route costs (distance, stop cost, currently unknown cells expected still unseen, a scenario-spread risk), allows an arbitrary number while incremental benefit exceeds cost, and retains strong short alternatives at each depth.
4. **Dense verification.** Evaluate shortlisted complete sequences again with **2,500 beams × 0.25-cell ray steps**, rather than trusting a 250-beam × 0.5-cell search. Check prefixes, remove any goal that reduces the dense objective, and return the best tested sequence. Dense validation does **not** cover every combinatorial sequence; no global optimum is claimed.
5. **Dynamic G2.** `assess_replan` is unchanged. At meaningful new LiDAR, **external orchestration** must run LaMa, Room Node, and then G2 on the new snapshot, and can abandon/reorder the old chain. This planner does not fake those calculations or run motion itself. Room `COMPLETE_OBSERVED` requires zero unknown in accepted room mask; `DEFER` is not completion.

`G2Config` preserves the prior cost coefficients as explicitly **uncalibrated**. R2 updates `planning_beams` default 128 → 250, `ray_step_cells` 1 → 0.5, `beam_width` 5 → 6, adds `final_beams=2500`, `final_ray_step_cells=0.25`, `fine_shortlist_per_depth=5`. These resource/search defaults are not fitted to D02.

## CLI

```bash
PYTHONPATH=/path/to/package_parent python -m mapex_g2_v1.cli \
  --snapshot /path/to/decision_000002_input.npz \
  --semantic /path/to/decision_000002_semantic.npz \
  --robot-rc 147,111 --decision-id RUN04_D02_R2 \
  --output /tmp/d02_r2.json \
  --visualization-png /tmp/d02_r2.png \
  --coverage-npz /tmp/d02_r2_coverage.npz
```

On Run04 the robot's native padded `row,col` subtracts (500,500) for these core rasters. CLI preserves the V1 flags and schema, adding only more detailed search/provenance fields. Note that a plan's returned goals are **conditional** and the future goals must be checked or replanned before actual movement.

## QA

```bash
PYTHONPATH=/path/to/package_parent python -m unittest discover -s /path/to/package_parent/mapex_g2_v1/tests -v
```

Automated tests cover malformed inputs, valid knownfree-only routes, whether one or several goals can be useful, unchanged-vs-new-obstacle replanning, ensemble-vs-proxy scenario semantics, first-hit unknown-wall visibility, idempotent ray/scan/leg coverage, dense goal deletion, and visualization/coverage consistency. Test numbers are technical QA only, not proof of scientific superiority.

## Limitations

The ray model is grid-based and not source-native `range_libc`/scanpolygon equivalence. The path planner is a 4-neighbor grid approximation and not physical safety certification. The scenario maps and cost coefficients are not calibrated predictive probabilities or empirically measured robot time; the unknown weighting is a heuristic. The beam search limits candidates for tractability, and dense re-ranking of a shortlist does **not** guarantee the globally optimal goal chain. G2-V1 R2 is suitable for controlled offline comparison; do not promote to deployed robot control without independent methods QA and evaluation.
