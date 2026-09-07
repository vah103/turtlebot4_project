# mapex_lab status

## Current focus

- Build a reproducible New Room exploration benchmark around the shared Nearest-Frontier execution layer and MapEx policy.
- Compare stock SLAM, local scan correction, and segmented submap correction without duplicating exploration logic.
- Keep experiment provenance, map snapshots, prediction artifacts, and evaluation outputs tied to the exact runtime configuration.

## Current runtime stack

- `launch/stock.launch.py`: New Room by default, stock scan path into SLAM Toolbox, stock TurtleBot4 Nav2 plus `config/nav2.yaml` overrides.
- `launch/local.launch.py`: local scan frontend + SLAM Toolbox + Nav2. It reuses `config/slam.yaml` and overrides the scan topic at launch time.
- `launch/submap.launch.py`: segmented local ICP frontend (`scripts/submap.py`) + SLAM Toolbox + Nav2. It reuses `config/slam.yaml` and overrides the scan topic at launch time.
- `launch/toolbox.launch.py`: conservative Toolbox baseline; when the experimental patched solver is built this launch explicitly keeps `temporal_anchor_enabled=false`.
- `launch/toolbox_anchor.launch.py`: experimental A/B variant with the same Toolbox scan/loop/Nav2 settings but `temporal_anchor_enabled=true`, early-local max weight `5.0`, decay `50` nodes, and local-edge gap threshold `5`.
- The launchers use world-specific automatic spawn resolution; New Room is the default world while Hospital remains selectable explicitly where supported.
- `config/` is intentionally reduced to three source-of-truth files: `nav2.yaml`, `slam.yaml`, and `mapex.yaml`.

## Benchmark / recorder state

- `scripts/nf_basic.py` is the shared canonical Nearest-Frontier execution layer.
- `scripts/nf_run.py` adds benchmark recording and provenance for Nearest-Frontier runs.
- `scripts/nf_anchor_run.py` keeps the canonical Nearest-Frontier policy unchanged while recording the `toolbox_anchor.launch.py` runtime provenance and hashing the temporal-anchor patch.
- `scripts/mapex.py` contains the MapEx exploration policy.
- `scripts/mapex_run.py` adds MapEx recording, saved prediction maps, environment-aware coverage, and post-run IoU/TU evaluation.
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

## In progress

- Runtime-smoke-test the new `new_room` profile end-to-end: confirm auto-generated GT/ROI, numeric New Room Coverage, five prediction files per decision, and final `evaluation.json` with `status: ok`.
- Visually validate `ground_truth/new_room/generated/new_room_structural_gt_v1.pgm` against Gazebo/RViz before treating New Room IoU/TU as research results.
- Run repeated Nearest and MapEx trials under the same runtime profile before drawing conclusions from single-run outcomes.
- Experimental temporal-anchor A/B: apply `src/slam/temporal_anchor_ceres.patch`, rebuild vendored `slam_toolbox`, run `toolbox_anchor.launch.py`, and use `nf_anchor_run.py` for an NF pilot. This is a diagnostic/proposed-SLAM variant, not the current benchmark baseline.
