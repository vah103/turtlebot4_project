# mapex_lab status

## Current focus

- Build a reproducible New Room exploration benchmark around the shared Nearest-Frontier execution layer and MapEx policy.
- Compare stock SLAM, local scan correction, and segmented submap correction without duplicating exploration logic.
- Keep experiment provenance, map snapshots, prediction artifacts, and evaluation outputs tied to the exact runtime configuration.
- Runtime-validate Adaptive Temporal Anchor V1 as a diagnostic SLAM ablation before considering any protocol change.

## Current runtime stack

- `launch/stock.launch.py`: New Room by default, stock scan path into SLAM Toolbox, stock TurtleBot4 Nav2 plus `config/nav2.yaml` overrides.
- `launch/local.launch.py`: local scan frontend + SLAM Toolbox + Nav2. It reuses `config/slam.yaml` and overrides the scan topic at launch time.
- `launch/submap.launch.py`: segmented local ICP frontend (`scripts/submap.py`) + SLAM Toolbox + Nav2. It reuses `config/slam.yaml` and overrides the scan topic at launch time.
- `launch/toolbox.launch.py`: conservative Toolbox A/B baseline. The vendored Ceres solver now contains Adaptive Anchor V1 code but this launch explicitly sets `adaptive_anchor_enabled=false`, so ordinary Toolbox constraint weights remain unchanged.
- `launch/new_toolbox.launch.py`: current experimental Adaptive Temporal Anchor V1. It keeps the stable Toolbox graph construction, loop closure, whole-graph Ceres optimization, scan cadence, loop thresholds and Nav2 settings, but now explicitly uses `scan_buffer_size=30` instead of the upstream default `10` to test whether a larger local scan-matching reference window reduces progressive map drift. Strict sequential edges receive `w(n)=1+2*exp(-n/50)` temporal weighting; strong consistent long-gap evidence can regionally release that added weight `1.0 -> 0.5 -> 0.0`.
- `src/slam/adaptive_anchor_v1.md`: exact current Adaptive V1 algorithm, parameters, fallback edge classification, release logic and validation checklist.
- `src/slam/temporal_anchor_ceres.patch`: earlier fixed soft-anchor prototype retained only as historical diagnostic material; its obsolete launch file has been removed.
- `scripts/apply_hard_chain.py`: earlier strict hard-chain diagnostic helper retained only for history; its obsolete launch file has been removed. The first runtime test was not satisfactory and this is no longer the current direction.
- The launchers use world-specific automatic spawn resolution; New Room is the default world while Hospital remains selectable explicitly where supported.
- `config/` is intentionally reduced to three source-of-truth files: `nav2.yaml`, `slam.yaml`, and `mapex.yaml`.

## Benchmark / recorder state

- `scripts/nf_basic.py` is the shared canonical Nearest-Frontier execution layer and remains unchanged for Adaptive Anchor tests.
- `scripts/nf_run.py` adds benchmark recording and provenance for Nearest-Frontier runs.
- `scripts/mapex.py` contains the MapEx exploration policy.
- `scripts/mapex_run.py` adds MapEx recording, saved prediction maps, environment-aware coverage, and post-run IoU/TU evaluation.
- `scripts/nf_new_toolbox_run.py` registers `new_room_new_toolbox_adaptive_v1_buffer30` provenance and records Nearest runs under `launch/new_toolbox.launch.py` without changing the policy.
- `scripts/mapex_new_toolbox_run.py` registers the same `new_room_new_toolbox_adaptive_v1_buffer30` provenance for MapEx and also hashes the vendored adaptive Ceres solver.
- New Room uses the generated `new_room_connected_free_v1` ROI and `new_room_structural_gt_v1` structural ground truth.

## Ground truth / evaluation

- Added `ground_truth/hospital/structural_gt_v1.yaml` for Hospital structural-GT semantics.
- Added `scripts/generate_new_room_ground_truth.py` plus `ground_truth/new_room/structural_gt_v1.yaml`. New Room GT is rasterized directly from all `mini_hospital_structure` box collisions intersecting `z=0.20 m`, including walls, room obstacles, and rotated central screen `c66`; the ground plane is excluded.
- New Room generated artifacts are local-only: `new_room_structural_gt_v1.npz`, `new_room_connected_free_v1.npy`, preview PGM, and summary JSON under `ground_truth/new_room/generated/`.
- `mapex_run.py` has explicit `new_room` and `hospital` environment profiles. **`new_room` is the default profile.** New Room uses `new_room_connected_free_v1` for Coverage and `new_room_structural_gt_v1` + the same ROI for IoU/TU; Hospital retains its own ROI/GT paths.
- For `new_room`, `mapex_run.py` automatically generates GT/ROI before the ROS benchmark begins when artifacts are missing, and regenerates them when the tracked `new_room.sdf` SHA-256 changes. This work occurs before the benchmark clock starts.
- Added `scripts/evaluate_mapex_profiled.py` so the existing evaluator receives the active environment's GT + ROI pair. This prevents New Room TU goals from accidentally being sampled from Hospital's ROI.
- Coverage denominator is environment-specific in `mapex_run.py`: it is computed from the actual loaded connected-free ROI rather than always using Hospital's fixed `215435` cells.
- The standalone evaluator synthetic consistency test previously returned occupied IoU `1.0` and TU `1.0` for matching prediction/ground truth and successfully backfilled `metrics.csv`.
- `launch/stock.launch.py` now defaults to `new_room`; its world-specific spawn is resolved automatically, and Hospital can still be selected explicitly.
- `launch/submap.launch.py` now also defaults to `new_room`, while retaining its segmented local scan frontend. Hospital remains selectable explicitly.
- For the current auxiliary New Room smoke test, `config/nav2.yaml` allows `velocity_smoother` linear commands up to `±0.75 m/s` while keeping angular commands at `±1.9 rad/s`; the Gazebo Create3 hard linear cap remains `0.8 m/s`. This speed setting is not yet validated as a formal Hospital benchmark setting.

## Adaptive Anchor V1 implementation state

- Vendored `slam_toolbox/solvers/ceres_solver.cpp` contains the current Adaptive V1 implementation and defaults to disabled.
- Strict sequential local edges use temporal weight `1 + 2*exp(-n/50)`; long-gap edges remain at normal `1x` weight.
- V1 deliberately has no extra covariance confidence multiplier because Karto covariance already forms the Ceres information matrix.
- Since `karto::LinkInfo` does not expose an edge-origin type through the ScanSolver API, V1 uses `node_gap<=1` for temporal local edges and `node_gap>=30` only as conservative loop-evidence candidates.
- Long-gap evidence is captured at `AddConstraint()` before Karto loop correction can optimize away the initial residual. Strong evidence requires at least three recent independent edges with consistent correction and squared normalized/Mahalanobis residual >= `9.0`.
- A strong evidence cluster releases only its related trajectory interval to factor `0.5`, then Ceres solves the whole graph. Stage 2 recomputes current residuals for the same evidence edges; only persistent strong conflict releases that interval to factor `0.0`, which returns those local edges to ordinary Toolbox `1x`, followed by another whole-graph solve.
- Release is one-way for a mapping session; no historical pose is hard-frozen by Adaptive V1.
- The vendored `slam_toolbox` compiled successfully on `com1`, and the first `new_toolbox` smoke test was reported as running stably. This is still a diagnostic observation, not an official benchmark result.
- `new_toolbox` now uses `scan_buffer_size=30` as a separate local-scan-matching diagnostic change. Adaptive Anchor V1 equations and Ceres implementation are unchanged.
- This remains a diagnostic experiment and does **not** modify `hospital_v2` official benchmark protocol.

## In progress / next action

- Collect three fresh Nearest runs on the current `new_toolbox` buffer-30 runtime: `nf_001`, `nf_002`, `nf_003`.
- Collect three fresh MapEx runs on the same current `new_toolbox` buffer-30 runtime: `mpx_001`, `mpx_002`, `mpx_003`.
- Do not mix earlier `new_toolbox` buffer-10 or buffer-20 runs with the new buffer-30 batch; runtime provenance must match before comparison.
- Restart `new_toolbox.launch.py` from a fresh simulation/SLAM state before every recorded run; do not chain multiple recorders onto one mapping session.
- During adaptive diagnostic runs, verify that one long-gap edge cannot release history, stage-1 logs a finite interval with `release=0.50`, and stage-2 occurs only if the same evidence edges remain strongly inconsistent after the stage-1 solve.
- Runtime-smoke-test the new `new_room` profile end-to-end: confirm auto-generated GT/ROI, numeric New Room Coverage, five prediction files per decision, and final `evaluation.json` with `status: ok`.
- Visually validate `ground_truth/new_room/generated/new_room_structural_gt_v1.pgm` against Gazebo/RViz before treating New Room IoU/TU as research results.
- Compare the repeated Nearest and MapEx trials only after all runs share the same runtime profile and provenance.
