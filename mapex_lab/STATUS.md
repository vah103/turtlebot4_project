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
- `scripts/mapex_lama_worker.py` exports the complete three-member LaMa prediction stack `P1/P2/P3` in addition to mean/variance, and `scripts/mapex_lama_bridge.py` exposes the canonical `predict_maps` / mean / variance interface to the ROS-side MapEx policy.
- `scripts/mapex_run.py` saves **five prediction products per completed prediction decision by default**: `G1`, `G2`, `G3`, ensemble `mean`, and ensemble `variance`. Each NPZ carries source resolution, source shape, padding, `/map` origin `x/y`, environment, and member label; `decisions.csv` stores all five paths.
- `scripts/mapex_run.py` records the map-frame robot pose at the first policy decision as `evaluation_start_x/y` for Topological Understanding.
- Added `scripts/evaluate_mapex_run.py`; IoU/TU run only after recorder CSV files are closed, so evaluation does not contaminate online benchmark time.
- Occupied IoU uses ensemble-mean prediction `> 0.5` as occupied and compares it with occupied structural ground truth inside the evaluation mask.
- TU uses 100 deterministic free-space goals, 4-connected planning, and counts success only when a path exists on the predicted map and does not intersect GT occupied cells. `pyastar2d` is used when available, otherwise deterministic 4-neighbour BFS is recorded.
- The evaluator writes `evaluation.csv` and `evaluation.json`, updates `summary.json` with final IoU/TU and time/distance AUC values, saves the TU goal set, and backfills `occupied_iou`/`tu` in `metrics.csv` using the latest available decision prediction.
- Added `ground_truth/hospital/structural_gt_v1.yaml` for Hospital structural-GT semantics.
- Added `scripts/generate_new_room_ground_truth.py` plus `ground_truth/new_room/structural_gt_v1.yaml`. New Room GT is rasterized directly from all `mini_hospital_structure` box collisions intersecting `z=0.20 m`, including walls, room obstacles, and rotated central screen `c66`; the ground plane is excluded.
- New Room generated artifacts are local-only: `new_room_structural_gt_v1.npz`, `new_room_connected_free_v1.npy`, preview PGM, and summary JSON under `ground_truth/new_room/generated/`.
- `mapex_run.py` now has explicit `new_room` and `hospital` environment profiles. **`new_room` is the default profile.** New Room uses `new_room_connected_free_v1` for Coverage and `new_room_structural_gt_v1` + the same ROI for IoU/TU; Hospital retains its own ROI/GT paths.
- For `new_room`, `mapex_run.py` automatically generates GT/ROI before the ROS benchmark begins when artifacts are missing, and regenerates them when the tracked `new_room.sdf` SHA-256 changes. This work occurs before the benchmark clock starts.
- Added `scripts/evaluate_mapex_profiled.py` so the existing evaluator receives the active environment's GT + ROI pair. This prevents New Room TU goals from accidentally being sampled from Hospital's ROI.
- Coverage denominator is now environment-specific in `mapex_run.py`: it is computed from the actual loaded connected-free ROI rather than always using Hospital's fixed `215435` cells.
- The standalone evaluator synthetic consistency test previously returned occupied IoU `1.0` and TU `1.0` for matching prediction/ground truth and successfully backfilled `metrics.csv`.
- `launch/stock.launch.py` now defaults to `new_room` with spawn `(0.0, 0.0, 0.0)`, matching `stock2.launch.py` and the New Room GT/ROI assumptions. Hospital can still be selected explicitly with launch arguments when needed.
- `launch/submap.launch.py` now also defaults to `new_room` with spawn `(0.0, 0.0, 0.0)`, while retaining its segmented local scan frontend + `slam_submap.yaml` stack. Hospital remains selectable explicitly.
- For the current auxiliary New Room smoke test, `config/nav2.yaml` now allows `velocity_smoother` linear commands up to `±0.75 m/s` while keeping angular commands at `±1.9 rad/s`; the Gazebo Create3 hard linear cap remains `0.8 m/s`. This speed setting is not yet validated as a formal Hospital benchmark setting.

## In progress

- Runtime-smoke-test the new `new_room` profile end-to-end: confirm auto-generated GT/ROI, numeric New Room Coverage, five prediction files per decision, and final `evaluation.json` with `status: ok`.
- Visually validate `ground_truth/new_room/generated/new_room_structural_gt_v1.pgm` against Gazebo/RViz before treating New Room IoU/TU as research results.
- Confirm stable SLAM/Nav2 behavior at the new `0.75 m/s` New Room linear limit before using it for repeated runs.
- Compare `stock.launch.py` and `submap.launch.py` on the same New Room setup if the segmented scan frontend is being evaluated.
- Continue runtime validation of shared `206/208` suppression, path-guided recovery, and terminal planner-reachability completion.
- Hospital structural GT still needs generation/alignment validation before real Hospital IoU/TU are claimed.
- Resolve the current debug `local.launch.py` runtime profile vs formal `hospital_v2` protocol before treating long Hospital Nearest/MapEx batches as formal benchmark comparisons.

## Next actions

1. Pull the latest repo.
2. Launch New Room with either `ros2 launch mapex_lab/launch/stock.launch.py` or `ros2 launch mapex_lab/launch/submap.launch.py`, depending on which SLAM frontend is being tested.
3. Run one short New Room MapEx recorder test with the normal single entry point:
   `python mapex_lab/scripts/mapex_run.py --run-id mapex_test_001`
   (`--environment new_room` is optional because New Room is now the default.)
4. During the smoke test, watch tight turns/doorways and confirm `0.75 m/s` does not increase collision-monitor stops, controller failures, or SLAM instability.
5. Verify `ground_truth/new_room/generated/` contains GT + ROI + preview + summary, and inspect the PGM alignment.
6. Verify `experiments/mapex/mapex_test_001/predictions/` contains matching `_g1.npz`, `_g2.npz`, `_g3.npz`, `_mean.npz`, `_variance.npz` files for each completed prediction decision.
7. Verify `evaluation.json` reports `status: ok`; inspect `evaluation.csv`, `metrics.csv`, and `summary.json` for Coverage + Time + Distance + IoU + TU.
8. Only after this single-run validation passes, run repeated New Room experiments. Use `--environment hospital` explicitly when switching back to Hospital.

## Important decisions

- **Official Nearest policy:** `mapex_lab/scripts/nf_basic.py`.
- **Official MapEx policy:** `mapex_lab/scripts/mapex.py`.
- **Official MapEx recorded-run entry point:** `mapex_lab/scripts/mapex_run.py`.
- **Base offline structural evaluator:** `mapex_lab/scripts/evaluate_mapex_run.py`.
- **Environment-aware evaluator adapter:** `mapex_lab/scripts/evaluate_mapex_profiled.py`.
- The canonical MapEx policy file remains untouched by G1/G2/G3 recording and environment-specific evaluation; these are instrumentation/evaluation changes only.
- IoU/TU are post-run metrics and must not run inside the online decision loop.
- New Room GT is derived from collision geometry rather than SLAM output, so it is independent evaluation truth. Generated files remain local-only and are reproducible from the tracked SDF + generator.
- New Room and Hospital must use their own GT/ROI pairs. Never mix New Room runs with Hospital ROI, denominator, or structural GT.
- `metrics.csv` IoU/TU values are post-run step-held values from the latest evaluated MapEx decision; exact decision-level values live in `evaluation.csv`.
- Report/plot exact offline structural values from `evaluation.csv`; use `metrics.csv` backfill for convenient aligned time-series plotting only.
- Official Hospital Nearest, MapEx, and any proposed method must use the same Hospital adaptations, SLAM/Nav2 runtime, canvas, ROI, stopping conditions, and structural GT when compared.
- Auxiliary `new_room` results remain separate from formal `hospital_v2` statistics unless a dedicated New Room benchmark protocol is frozen for every compared method.
- The `0.75 m/s` linear limit is currently an auxiliary New Room runtime choice; do not treat it as a frozen `hospital_v2` protocol value without an explicit protocol update and rerun decision.

## Latest result

2026-09-03: `submap.launch.py` was aligned with the active New Room workflow. It now defaults to `new_room` and spawn `(0.0, 0.0, 0.0)`, matching the New Room GT/ROI assumptions, while preserving the segmented local scan frontend and `slam_submap.yaml`. `stock.launch.py` and `submap.launch.py` can now be launched on the same New Room geometry for direct runtime comparison.
