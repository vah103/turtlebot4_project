# STATUS

## Current stage

**Stage 3 — MapEx closed-loop instrumentation + offline evaluation**

## Done

- Research workspace is `mapex_lab/`; official MapEx source is pinned at `53636bd1c79153acc3c74a532837d78c926bae5e`.
- Hospital v2 target runtime SLAM/policy grid is `0.10 m/cell`; fixed evaluation canvas is `hospital_canvas_v1` at `0.05 m/cell` with canonical ROI `hospital_connected_free_v1`, denominator `215435`.
- **Canonical Nearest Frontier policy/runner:** `scripts/nf_basic.py`. `scripts/nf_run.py` is the measurement wrapper and shares the exact execution/recovery/completion semantics.
- Nearest frontier semantics remain fixed: free `==0`, unknown `<0`, 8-neighbour frontier, 8-connected regions, region size `>10`, representative frontier cell nearest arithmetic mean, Euclidean ranking.
- Exact frontier `x/y` is the navigation target. Main-goal planner-blocking `206 GOAL_OCCUPIED` and `208 NO_VALID_PATH` are temporarily suppressed for ordinary selection and can be revived by terminal `ComputePathToPose` revalidation.
- `nearest_001` is retained with a manual-termination caveat: final coverage `0.996184464`, distance `578.24 m`.
- `nearest_002` completed automatically: coverage `0.993965697`, distance `525.85 m`, benchmark time `5897.07 s`, 50/62 successful main attempts.
- `nearest_003` completed automatically: coverage `0.992034720`, distance `479.20 m`, benchmark time `5663.42 s`, 42/58 successful main attempts, terminal reason `no_planner_reachable_frontier`.
- The interrupted old `nearest_004` is invalid because it exposed the pre-fix `206` infinite-retry pathology and must not be used as a completed benchmark run.
- **Canonical Stage-3 MapEx policy:** `scripts/mapex.py`. It subclasses the shared Nearest execution layer and replaces only the policy with: MapEx frontier set -> 3-member LaMa ensemble -> mean/variance -> probabilistic visibility -> variance information gain -> `IG / EuclideanDistance` -> highest-score unsuppressed frontier.
- `config/mapex.yaml` is the Stage-3 policy source of truth: 3 LaMa members, `0.10 m/cell`, `default_map_eval`, frontier region `>10`, 20 m / 250-ray visibility, epsilon `0.8`, ensemble-variance IG, Euclidean `IG/distance`, exact frontier execution.
- `scripts/mapex.py` uses official MapEx LaMa preprocessing and the reference buffered-polygon/Bresenham/flood-fill visibility construction. The probabilistic ray accumulator follows the paper definition (accumulate along each ray) rather than reproducing the upstream helper reset bug.
- Auxiliary `new_room` closed-loop testing successfully loaded all three LaMa models on CPU, produced finite IG/distance/score values, sent exact frontier goals to Nav2, and reached multiple goals. This is an auxiliary cross-environment check, not formal `hospital_v2` benchmark evidence.
- `scripts/mapex_run.py` records MapEx runs under `experiments/mapex/<run_id>/`: coverage/known fraction vs time/distance, trajectory, goals, plans, candidate set, selected IG/distance/score, prediction/scoring/total computation timing, exact decision maps, periodic/final maps, and `summary.json`/`metadata.json` provenance.
- `scripts/mapex_lama_worker.py` now exports the complete three-member LaMa prediction stack `P1/P2/P3` in addition to mean/variance, and `scripts/mapex_lama_bridge.py` exposes the canonical `predict_maps` / mean / variance interface to the ROS-side MapEx policy.
- `scripts/mapex_run.py` now captures and saves **five prediction products per completed prediction decision by default**: `G1`, `G2`, `G3`, ensemble `mean`, and ensemble `variance`. The three member maps are the individual LaMa outputs before the recorder combines them; mean/variance remain the maps used by MapEx policy/evaluation.
- Each saved prediction NPZ carries source resolution, source shape, padding, `/map` origin `x/y`, and a `member` label. `decisions.csv` now includes `g1_map`, `g2_map`, `g3_map`, `mean_map`, and `variance_map` paths.
- `scripts/mapex_run.py` records the map-frame robot pose at the first policy decision as `evaluation_start_x/y` for Topological Understanding.
- Added `scripts/evaluate_mapex_run.py` and integrated it into `scripts/mapex_run.py`. The evaluator runs only **after recorder CSV files are closed**, so IoU/TU computation does not contaminate online benchmark time.
- Occupied IoU follows the official MapEx threshold convention: ensemble-mean prediction `> 0.5` is occupied; evaluation is against occupied structural ground truth inside the structural evaluation mask.
- Topological Understanding follows the MapEx evaluation idea: 100 deterministic free-space goals, 4-connected predicted-map planning, success only if a predicted path exists and no path cell intersects occupied ground truth. `pyastar2d` is used when available; otherwise a deterministic 4-neighbour BFS shortest-path fallback is recorded in provenance.
- The evaluator writes `evaluation.csv` and `evaluation.json`, updates `summary.json` with final IoU/TU and time/distance AUC values, saves the TU goal set, and post-run backfills the existing `occupied_iou`/`tu` columns in `metrics.csv` using the latest available decision prediction at each metrics timestamp.
- Added `ground_truth/hospital/structural_gt_v1.yaml`, defining the local-only structural artifact contract `generated/hospital_structural_gt_v1.npz` on `hospital_canvas_v1`.
- The evaluator was syntax-checked and passed a synthetic consistency test where matching prediction/ground truth produced `occupied IoU = 1.0` and `TU = 1.0`, including successful `metrics.csv` backfill.

## In progress

- Runtime-smoke-test the updated bridge/recorder on the currently used auxiliary map and verify every prediction decision produces exactly `G1/G2/G3/mean/variance` NPZ files with matching shape/metadata.
- Generate and validate structural ground truth for the actual map selected for quantitative IoU/TU evaluation. Until the correct map-specific GT exists, real IoU/TU are intentionally not produced.
- Validate structural GT alignment against the SLAM map: collision source, world -> map transform, resolution, building footprint/evaluation mask, wall/door treatment, and free start pose.
- Continue runtime validation of shared `206/208` suppression, path-guided recovery, and terminal planner-reachability completion.
- Resolve the current debug `local.launch.py` runtime profile vs formal `hospital_v2` protocol before treating long Nearest/MapEx batches as formal benchmark comparisons.

## Next actions

1. Pull the latest repo.
2. Run one short MapEx recorder test with the normal single entry point:
   `python mapex_lab/scripts/mapex_run.py --run-id mapex_test_001`
3. Check `experiments/mapex/mapex_test_001/predictions/`; each prediction decision must contain matching `_g1.npz`, `_g2.npz`, `_g3.npz`, `_mean.npz`, `_variance.npz` files.
4. Check `decisions.csv` and verify all five prediction-path columns are populated for decisions where LaMa inference completed.
5. Build/validate the structural ground truth for the actual evaluation map, then rerun `scripts/evaluate_mapex_run.py` or let `mapex_run.py` invoke it automatically at run end.
6. Only after the single-run validation passes, run repeated MapEx/Nearest experiments under one frozen runtime profile.

## Important decisions

- **Official Nearest policy:** `mapex_lab/scripts/nf_basic.py`.
- **Official MapEx policy:** `mapex_lab/scripts/mapex.py`.
- **Official MapEx recorded-run entry point:** `mapex_lab/scripts/mapex_run.py`.
- **Offline MapEx structural evaluator:** `mapex_lab/scripts/evaluate_mapex_run.py`.
- The canonical MapEx policy file remains untouched by the new G1/G2/G3 recording feature; individual-member capture is instrumentation in the worker/bridge/recorder path only.
- IoU/TU are post-run metrics. They must not be computed inside the online decision loop because TU is expensive and would change exploration timing.
- If structural GT is missing/invalid, the evaluator writes a `skipped_*` status and leaves IoU/TU unavailable; it must never invent placeholder scores.
- Structural GT is local/generated data and is not committed. The lightweight semantic contract is committed in `ground_truth/hospital/structural_gt_v1.yaml`.
- `metrics.csv` IoU/TU values are post-run step-held values from the latest evaluated MapEx decision; exact decision-level values live in `evaluation.csv`.
- Report/plot exact offline structural values from `evaluation.csv`; use `metrics.csv` backfill for convenient aligned time-series plotting only.
- Official Nearest, MapEx, and any proposed method must use the same Hospital adaptations, SLAM/Nav2 runtime, canvas, ROI, stopping conditions, and structural GT when compared.
- Auxiliary `new_room` / House tests must remain separate from formal `hospital_v2` statistics.

## Latest result

2026-09-03: MapEx run instrumentation was extended to preserve all three individual LaMa outputs. The legacy worker now returns `P1/P2/P3`; the ROS bridge exposes them through the canonical MapEx inference interface; and `mapex_run.py` stores `G1/G2/G3/mean/variance` per completed prediction decision and records their paths in `decisions.csv`. The canonical `mapex.py` frontier-selection policy was not changed. Runtime file-output validation on the user's current map is the next gate; map-specific structural ground truth can be added afterward for IoU/TU.