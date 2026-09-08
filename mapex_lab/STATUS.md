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
- `launch/oldmap_toolbox.launch.py`: current Old-Map-First V2 diagnostic. It applies old>new confidence in both local scan-matching stages and at the Ceres pose-graph level. Initial local matching keeps the normal recent buffer (`30` scans), adds nearby trusted historical keyframes (`0.5 m` spacing, `3.0 m` radius, max `40`), retains scan 1 as trusted history, and uses `c_raw(i)=0.25+0.75*exp(-i/70)` with active-reference normalization. Graph construction mirrors Karto topology, but near-chain local matching now uses the same temporal weighted correlation rule rather than equal-confidence `MatchScan`. The first node remains hard-fixed by Ceres; local edges with `node_gap<=5` use `w(n)=1+4*exp(-n/70)` (`5x -> 1x`); long-gap/loop edges remain `1x`; adaptive release is disabled with `1.0 -> 1.0`. Karto loop-closure matching remains upstream/unweighted.
- `src/slam/adaptive_anchor_v1.md`: exact Adaptive V1 algorithm, parameters, fallback edge classification, release logic and validation checklist.
- `src/slam/oldmap_anchor_v1.md`: historical pose-graph-only Old-Map-First V1 diagnostic.
- `src/slam/oldmap_anchor_v2.md`: current two-layer Old-Map-First V2 algorithm, weighted initial + near-chain local matching, temporal Ceres policy, limitations and validation checklist.
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
- `scripts/nf_oldmap_toolbox_run.py` registers `new_room_oldmap_toolbox_v2` provenance and hashes the V2 matcher/glue/Ceres implementation.
- `scripts/mapex_oldmap_toolbox_run.py` registers the same `new_room_oldmap_toolbox_v2` provenance for MapEx.
- `oldmap_mapper.hpp` is included in provenance hashes, so the weighted-near-chain implementation is distinguishable from earlier V2 code even though the profile ID remains V2. Do not mix implementation hashes in one formal batch.
- Old-Map-First V1 and V2 runs must not be mixed because V2 changes C++ local scan matching in addition to the pose-graph weighting.
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

## Old-Map-First V2 implementation state

- `slam_toolbox/include/slam_toolbox/oldmap_mapper.hpp` adds `OldMapMapper`, a subclass of `karto::Mapper` whose weighted path is disabled by default.
- `slam_toolbox/src/slam_mapper.cpp` constructs `OldMapMapper` as a strict superset of the upstream mapper and exposes the V2 ROS parameters. Because `oldmap_scan_weighting_enabled` defaults to `false`, `toolbox`, `new_toolbox`, and other profiles call upstream `karto::Mapper::Process()` unchanged.
- When enabled by `oldmap_toolbox.launch.py`, the initial local match uses the normal recent running scans plus sparse historical keyframes near the predicted pose. The first scan is retained as a historical keyframe but participates directly only when spatially relevant.
- Raw scan confidence is `c_raw(i)=0.25+0.75*exp(-i/70)`. Active reference weights are divided by the strongest active raw confidence, preserving old>new ordering while preventing matcher response from collapsing in regions where only late scans are available.
- The custom correlation grid keeps Karto's Gaussian smear shape but scales each reference scan's kernel amplitude by its active temporal confidence. Grid fusion remains max-based.
- `OldMapMapper` now mirrors Karto `AddEdges()` internally when the feature is enabled. Previous-scan linking, running-chain linking, near-chain discovery, thresholding, closest-scan linking and covariance-weighted mean fusion follow the upstream logic; the intentional difference is that near-chain `MatchScan(..., doPenalize=false)` is replaced by `WeightedMatchScan(..., do_penalize=false)` with old>new weights computed inside that chain.
- Therefore both local frontend stages that can alter a new scan pose are temporal-weighted: initial sequential/local matching and near-chain local matching.
- Karto loop-closure coarse/fine matching remains upstream/unweighted so it stays an independent source of evidence. Accepted loop constraints remain `1x` in Ceres.
- Vendored `slam_toolbox/solvers/ceres_solver.cpp` still provides the pose-graph half of the method: first node constant, local edge `node_gap<=5`, `5x -> 1x` decay over `70` nodes, loop/long-gap edges `1x`, no release under this profile.
- This V2 **modifies C++** and therefore requires rebuilding the vendored `slam_toolbox` before runtime testing.
- First weighted-near-chain rebuild attempt on `com1` reached the linker but failed because Karto defines several public `MapperSensorManager` accessors as `inline` only in `Mapper.cpp`; the new `OldMapMapper` calls them from `slam_mapper.cpp`, so GCC omitted the externally-callable symbols. `slam_toolbox/CMakeLists.txt` now adds GNU `-fkeep-inline-functions` to `kartoSlamToolbox` so those existing accessors are emitted without changing algorithm behavior. Rebuild validation is pending.
- Compile/runtime validation on `com1` is still pending for the weighted-near-chain version. The previous successful build only validated the earlier Ceres implementation, not this current frontend implementation.
- These remain diagnostic experiments and do **not** modify `hospital_v2` official benchmark protocol.

## In progress / next action

- Pull current `main`, rebuild only vendored `slam_toolbox` sequentially on `com1`, and confirm the linker error for `MapperSensorManager::{GetLastScan,GetScans,GetRunningScans,AddRunningScan}` is gone.
- Verify the workspace package is being used after the successful rebuild.
- Launch `oldmap_toolbox.launch.py` and verify startup reports `Old-map sequential matcher: enabled=true, min_conf=0.25, decay=70.0, keep_first=true, keyframe_dist=0.50, history_radius=3.00, history_max=40`.
- Verify Ceres startup reports approximately `weight=5.00->1.00`, `decay=70.0`, `local_gap<=5`, and `release=1.00->1.00`.
- Run `nf_basic.py` first rather than a recorder. Check that mapping starts normally, the first node remains constant, and no custom AddEdges / weighted near-chain assertion or correlation-grid error occurs.
- Drive/explore far enough to revisit early mapped space and compare whether early walls remain more stable while the recent trajectory is pulled back toward trusted history.
- If historical matching creates false attraction in repeated geometry, reduce `oldmap_history_search_radius` or historical keyframe density before changing Ceres weights.
- If weighted near-chain matching makes graph construction too rigid or rejects too many near-chain links, inspect response/covariance first; do not immediately loosen loop closure.
- If valid loop closure cannot correct the recent trajectory enough, first reduce `adaptive_anchor_max_weight` or `adaptive_anchor_decay_nodes`; do not add a release mechanism until the two-layer hypothesis is evaluated.
- Only after the smoke test is stable, collect repeated Nearest/MapEx runs with `nf_oldmap_toolbox_run.py` / `mapex_oldmap_toolbox_run.py`, restarting simulation/SLAM fresh before every run.
- Do not mix `toolbox`, `new_toolbox`, Old-Map V1, and Old-Map V2 results under the same runtime provenance.