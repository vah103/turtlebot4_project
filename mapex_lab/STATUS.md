# mapex_lab status

## Current focus

- Build a reproducible New Room exploration benchmark around the shared Nearest-Frontier execution layer and MapEx policy.
- Compare stock SLAM, local scan correction, segmented submap correction, Adaptive Temporal Anchor V1, and an old-map-first temporal anchoring diagnostic without duplicating exploration logic.
- Keep experiment provenance, map snapshots, prediction artifacts, and evaluation outputs tied to the exact runtime configuration.
- Runtime-validate SLAM diagnostics before considering any protocol change.

## Current runtime stack

- `launch/stock.launch.py`: New Room by default, stock scan path into SLAM Toolbox, stock TurtleBot4 Nav2 plus `config/nav2.yaml` overrides.
- `launch/local.launch.py`: local scan frontend + SLAM Toolbox + Nav2. It reuses `config/slam.yaml` and overrides the scan topic at launch time.
- `launch/submap.launch.py`: segmented local ICP frontend (`scripts/submap.py`) + SLAM Toolbox + Nav2. It reuses `config/slam.yaml` and overrides the scan topic at launch time.
- `launch/toolbox.launch.py`: conservative Toolbox A/B baseline. The vendored Ceres solver contains Adaptive Anchor V1 code but this launch explicitly sets `adaptive_anchor_enabled=false`, so ordinary Toolbox constraint weights remain unchanged.
- `launch/new_toolbox.launch.py`: Adaptive Temporal Anchor V1 diagnostic. It uses `scan_buffer_size=30`, strict sequential temporal edges (`node_gap<=1`) with `w(n)=1+2*exp(-n/50)`, and strong-loop regional release `1.0 -> 0.5 -> 0.0`.
- `launch/oldmap_toolbox.launch.py`: new old-map-first V1 diagnostic. The first node remains hard-fixed by upstream Ceres, local edges with `node_gap<=5` use `w(n)=1+4*exp(-n/70)` (`5x -> 1x`), long-gap/loop edges remain `1x`, `scan_buffer_size=30`, and adaptive release is disabled with release factors `1.0 -> 1.0` so early constraints are not weakened by the adaptive fallback.
- `src/slam/adaptive_anchor_v1.md`: exact Adaptive V1 algorithm, parameters, fallback edge classification, release logic and validation checklist.
- `src/slam/oldmap_anchor_v1.md`: exact old-map-first V1 idea, parameters, expected behavior, limitations and runtime validation checklist.
- `src/slam/temporal_anchor_ceres.patch`: earlier fixed soft-anchor prototype retained only as historical diagnostic material; its obsolete launch file has been removed.
- `scripts/apply_hard_chain.py`: earlier strict hard-chain diagnostic helper retained only for history; its obsolete launch file has been removed. The first runtime test was not satisfactory and this is no longer the current direction.
- The launchers use world-specific automatic spawn resolution; New Room is the default world while Hospital remains selectable explicitly where supported.
- `config/` is intentionally reduced to three source-of-truth files: `nav2.yaml`, `slam.yaml`, and `mapex.yaml`.

## Benchmark / recorder state

- `scripts/nf_basic.py` is the shared canonical Nearest-Frontier execution layer and remains unchanged across SLAM diagnostics.
- `scripts/nf_run.py` adds benchmark recording and provenance for Nearest-Frontier runs.
- `scripts/mapex.py` contains the MapEx exploration policy.
- `scripts/mapex_run.py` adds MapEx recording, saved prediction maps, environment-aware coverage, and post-run IoU/TU evaluation.
- `scripts/nf_new_toolbox_run.py` registers `new_room_new_toolbox_adaptive_v1_buffer30` provenance and records Nearest runs under `launch/new_toolbox.launch.py` without changing the policy.
- `scripts/mapex_new_toolbox_run.py` registers the same `new_room_new_toolbox_adaptive_v1_buffer30` provenance for MapEx.
- `scripts/nf_oldmap_toolbox_run.py` registers `new_room_oldmap_toolbox_v1` provenance and records Nearest runs under `launch/oldmap_toolbox.launch.py` without changing the policy.
- `scripts/mapex_oldmap_toolbox_run.py` registers the same `new_room_oldmap_toolbox_v1` provenance for MapEx.
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
- The first graph node is already set constant by upstream Ceres behavior.
- Temporal local edges use the existing age-weight function, and launch profiles choose `max_weight`, `decay_nodes`, and `local_edge_max_gap`.
- Karto covariance already forms the Ceres information matrix; no second covariance multiplier is used.
- Since `karto::LinkInfo` does not expose an edge-origin type through the ScanSolver API, temporal-local and loop-like edges are classified by node-gap heuristics.
- `new_toolbox` keeps the original Adaptive V1 release behavior.
- `oldmap_toolbox` deliberately reuses the same solver implementation but sets release factors to `1.0 -> 1.0`; therefore strong-loop evidence cannot reduce its temporal local weights. This avoids a Ceres source change for the first old-map-first test and isolates the weighting policy cleanly.
- `oldmap_toolbox` expands local weighting to `node_gap<=5`, starts early local information at `5x`, and decays with `70` nodes. This is intended to make the old part of the trajectory progressively stiffer than the new part while leaving loop constraints at ordinary `1x`.
- The vendored `slam_toolbox` previously compiled successfully on `com1`. The new old-map launch does not modify C++ and therefore does not require another build; runtime validation is still pending.
- These remain diagnostic experiments and do **not** modify `hospital_v2` official benchmark protocol.

## In progress / next action

- Pull the new `oldmap_toolbox` launch and run a clean New Room smoke test before collecting formal-looking repeated runs.
- Verify startup log reports approximately `weight=5.00->1.00`, `decay=70.0`, `local_gap<=5`, and `release=1.00->1.00`.
- Verify first node remains constant, local gap 2..5 edges receive temporal weight, and long-gap edges remain `1x`.
- Drive/explore far enough to revisit early mapped space and compare whether early walls remain more stable than under `new_toolbox`.
- If valid loop closure cannot correct the recent trajectory enough, first reduce `adaptive_anchor_max_weight` or `adaptive_anchor_decay_nodes`; do not add a new release mechanism until the basic old-map-first hypothesis is evaluated.
- Only after the smoke test is stable, collect repeated Nearest/MapEx runs with `nf_oldmap_toolbox_run.py` / `mapex_oldmap_toolbox_run.py`, restarting simulation/SLAM fresh before every run.
- Do not mix `toolbox`, `new_toolbox`, and `oldmap_toolbox` results under the same runtime provenance.
- Runtime-smoke-test the New Room recording profile end-to-end: confirm auto-generated GT/ROI, numeric Coverage, and final evaluation artifacts before treating results as research evidence.
