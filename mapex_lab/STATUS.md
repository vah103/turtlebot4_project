# STATUS

## Current stage

**Stage 2 — MapEx Nearest baseline (Hospital v2 resolution-change validation before official runs)**

## Done

- Renamed research folder from `mapex_hospital_research/` to `mapex_lab/`; runtime files remain unchanged apart from path references.
- Removed obsolete legacy/debug Nearest branches that are no longer part of the active roadmap: `autonomous_exploration/`, `control_tb4_hospital.py`, `launch/hospital_nearest.launch.py`, `launch/nearest_frontier.launch.py`, `launch/nearest_frontier_full.launch.py`, `scripts/mapex_nearest_simple.py`, `scripts/mapex_nearest_full.py`, `scripts/test_nearest_frontier_nav.py`, and `scripts/test_nearest_frontier_run.py`. Active `nf_basic.py`, stock runtime, MapEx preparation, recorders, analysis, GT, protocol, references, and research artifacts are retained.
- Fixed canvas `hospital_canvas_v1`: `0.05 m`, `1504 x 2123`, origin `(-25.6,-60.1)`.
- Frozen ROI `hospital_connected_free_v1`: denominator `215435`, SHA-256 `05d45b7aba66dbb71e0406005f4a3e21875901af3ae72164b17f1d3add8d1`.
- Hospital runtime protocol advanced to `hospital_v2`: SLAM/policy map target is `0.10 m/cell`, while fixed evaluation canvas/ROI remain frozen at `0.05 m/cell`.
- Recorder reprojection updated: axis-aligned runtime maps whose resolution is an integer multiple of `0.05 m` are expanded nearest-neighbour onto `hospital_canvas_v1`; a `0.10 m` SLAM cell becomes a `2 x 2` block on the evaluation canvas.
- `hospital_slam_no_loop.yaml` remains at runtime `resolution: 0.10`; `hospital_slam.yaml` is temporarily rolled back to the historical `0.05` version from commit `9f404a9` only to regression-test old `control_tb4.py` behavior. Restore `0.10` before any Hospital v2 official run.
- Official MapEx source pinned: `53636bd1c79153acc3c74a532837d78c926bae5e`.
- Nearest frontier semantics ported: 8-neighbour frontier, 8-connected regions, region `>10`, representative gần arithmetic mean nhất, Euclidean ranking.
- Hospital bỏ original MapEx `<1 m` rejection vì `nearest_pilot_005` chứng minh startup deadlock; `below_1m` chỉ còn diagnostic.
- `nearest_pilot_007` phát hiện planner endpoint tolerance-snapped bị dùng sai làm NavigateToPose goal; đã sửa exact frontier x/y execution và `GridBased.tolerance=0.0`.
- `nearest_pilot_008` xác nhận exact-frontier logging nhưng lộ long-run map warping; mapping profile đổi sang `0.45 m/s` + SLAM keyframe `0.10 m / 0.10 rad`.
- `nearest_pilot_009` lộ thêm runtime/controller problem: planner replan liên tục nhưng `/cmd_vel_nav`, `/cmd_vel`, `/odom` gần zero.
- Rà soát code sau pilot_009 phát hiện và đã sửa các vấn đề:
  - frontier policy là position-only nhưng code từng ép quaternion yaw `0`; Hospital giờ bỏ final-yaw objective (`yaw_goal_tolerance=pi`, `GoalAngleCritic=false`);
  - `ComputePathToPose` chỉ được accept khi ROS action `STATUS_SUCCEEDED` **và** path non-empty;
  - stable non-empty no-path set phải planner revalidation mỗi `2 s`, không cache một lần rồi completion;
  - planner-valid goal execution failure chỉ suppress `0.25 m / 30 s`, không permanent;
  - `selected_path_length_m` được định nghĩa rõ là validation-path length, không phải executed trajectory;
  - recorder archive/hash cả base Nav2 params và `runtime_nav2_merged.yaml` effective config;
  - validator kiểm effective Nav2 config, action status, terminal revalidation và guard `SUCCEEDED` nhưng odometry gần như không di chuyển.
- Preflight parse YAML thực, kiểm source + installed configs thay vì chỉ tìm chuỗi text.
- Added `launch/stock.launch.py` as a reference/debug stack: project Hospital simulation + stock TurtleBot4 Nav2 + near-stock TurtleBot4 SLAM. Debug SLAM uses `config/slam.yaml`; this stack is **not** part of the official `hospital_v2` benchmark protocol.
- Added `step.py` as a debug-only wrapper around `control_tb4.py`: one frontier goal at a time, waits for the real Nav2 terminal result, prints compact motion/navigation diagnostics, then pauses for ENTER before the next goal.
- Added `step0.py` as a temporary debug wrapper that sets `min_distance_threshold = 0.0` so the centroid-selection behavior can be observed without the rejection filter; this is diagnostic only.
- Previously added debug-only path-guided intermediate-goal recovery to `nf_basic.py` plus `behavior_trees/navigate_to_pose_subgoal.xml`; this experiment was later rolled back from the active `nf_basic.py`.
- Previously tested event-driven global replanning and conservative 5-sweep completion in debug `nf_basic.py`; both were later rolled back from the active minimal `nf_basic.py`.
- Active debug `nf_basic.py` is now the minimal nearest-Euclidean version plus **planner reachability validation only**: candidates are sorted by Euclidean distance, checked by `ComputePathToPose` nearest-first, a candidate with failed/empty path is dropped for that candidate sweep, and `NavigateToPose` is sent only to the first planner-valid exact frontier x/y. No permanent blacklist is used, so no-path candidates can be revalidated on later sweeps.
- Added a Cartographer 2D debug alternative for long-run SLAM drift diagnosis: `config/cartographer_hospital_2d.lua` + `launch/carto.launch.py`. It uses `/scan` + wheel `/odom` + simulated `/imu`, 0.05 m internal submaps, pose-graph loop closure, and publishes a 0.10 m/cell ROS map for compatibility with `nf_basic.py`/Nav2. This is not yet an official `hospital_v2` protocol change.
- First Cartographer runtime attempt exposed a Gazebo/ROS IMU frame mismatch: `/imu` is stamped `turtlebot4/imu_link/imu`, while TF contains the physical `imu_link`. `launch/carto.launch.py` now publishes an identity static TF `imu_link -> turtlebot4/imu_link/imu` so Cartographer can retain IMU fusion instead of disabling IMU.
- Second Cartographer runtime reached `/map`, but `nf_basic.py` immediately accumulated no-frontier completion evidence. Root cause: Cartographer publishes observed occupancy probabilities in `[0,100]`, while MapEx frontier semantics require exact free `0`, unknown `<0`. Added `cartographer_map_bridge.py`; Cartographer now publishes raw probability map as `/cartographer_map`, and the bridge republishes `/map` as `-1/0/100` using a 50% occupancy threshold without changing `nf_basic.py`.
- Cartographer map-normalization bridge is now sufficient for `nf_basic.py` to start selecting frontiers and move the robot. During this debug run the robot appeared jerky in wall time; measured topic rates were approximately `/cmd_vel=14 Hz`, `/odom=14 Hz`, `/scan=2.6 Hz`.
- Performance A/B changes did not remove the visible jerk, so they were rolled back: online correlative scan matching is restored to `true` and Cartographer OccupancyGrid publication is restored from `3.0 s` to `1.0 s`. The current working hypothesis is Gazebo/physics real-time performance rather than Cartographer local-matching load.
- Added a debug-only SLAM Toolbox local-window frontend: `local_scan_window.py`, `config/slam_local_window.yaml`, `launch/local.launch.py`, and `docs/local_window_slam_debug.md`. It uses raw odometry only as the motion prediction, performs bounded 2-D ICP against a rolling recent-scan window, republishes `/scan_local_window`, and leaves SLAM Toolbox responsible for global scan matching, pose graph, loop closure, and optimization. The frontend has conservative per-scan and accumulated-correction guards and resets its local state instead of becoming a second unconstrained global SLAM estimator.
- Restored `control_tb4.py` exactly to historical blob `44f262b6ffa904042d1f2633d8f2ced95e645513` from commit `9f404a9`.
- Stage-3 preparation: `control_tb4_mapex.py` dùng `MAPEX_RESOLUTION_M = 0.10`; với Hospital v2 source map `0.10`, bước downsample trở thành factor `1`, nên frontier/prediction cùng grid 0.10.

## In progress

- Runtime validation of the debug local-window SLAM Toolbox frontend: compare corridor revisit wall overlap against `stock.launch.py`, inspect ICP acceptance/RMSE/reset diagnostics, and verify the Python frontend does not reduce Gazebo real-time performance enough to negate any mapping gain.
- Runtime validation of active minimal `nf_basic.py` reachability behavior: reproduce the closed-curtain/no-path case and confirm the nearest blocked frontier is skipped immediately and the next planner-reachable frontier is selected without a retry loop.
- Diagnose Gazebo/physics real-time performance while keeping the restored accuracy-first Cartographer configuration unchanged.
- Regression test: old `control_tb4.py` + historical `hospital_slam.yaml` (`0.05 m/cell`) to determine whether the new no-frontier startup behavior is caused by runtime-grid changes rather than the controller file itself.
- Hospital v2 official validation is paused while this diagnostic rollback is active.
- `control_tb4_mapex.py` vẫn là Stage-3 preparation only: chưa runtime-validate, chưa phải official MapEx benchmark runner và không thay đổi current stage.

## Next actions

1. Pull and run active `nf_basic.py` in the closed-curtain area; confirm a `ComputePathToPose` no-path result logs that the frontier is dropped for the current sweep.
2. Confirm `nf_basic.py` immediately checks the next Euclidean-ranked candidate and only sends `NavigateToPose` after a non-empty planner-valid path is returned.
3. Watch `/frontier_selected_path` and RViz markers: green candidates may include unreachable frontiers, but the red selected goal must be the first planner-reachable candidate.
4. Keep Cartographer accuracy settings fixed while checking Gazebo real-time factor and comparing headless-Gazebo + RViz against the normal GUI run.
5. While moving, measure `/cmd_vel`, `/odom`, and `/scan` rates and verify final Twist commands remain smooth while the simulated motion jerks.
6. Continue long enough to return to old corridors and inspect Cartographer wall overlap/map alignment under the restored accuracy-first configuration.
7. If Cartographer is clearly more stable overall, decide whether to promote it to the official benchmark stack. Promotion requires a new protocol version and all compared methods must use the same Cartographer stack.
8. Pull/rebuild `frontier_exploration`, run `hospital_flat_stack.launch.py`, then run restored `control_tb4.py` and check whether startup exploration works again at `0.05 m/cell`.
9. Record whether frontier groups/goals behave like the historical run.
10. After the regression test, restore `hospital_slam.yaml` to Hospital v2 `resolution: 0.10` before continuing official validation.
11. Resume Hospital v2 validation only after the diagnostic rollback is removed.
12. Run `launch/local.launch.py` on the same debug route as `stock.launch.py`; compare revisit wall overlap and record `/local_window_icp/accepted`, `/local_window_icp/rmse_m`, `/local_window_icp/correction`, reset count from logs, and wall-time topic rates before tuning any parameter.

## Important decisions

- Hospital implementation là MapEx frontier/ranking port + ROS/Nav2 execution adapter, không gọi là MapEx nguyên xi.
- Hospital v2 target runtime map/policy resolution = `0.10 m/cell`; the current `hospital_slam.yaml=0.05` state is temporary diagnostic-only.
- Evaluation vẫn dùng frozen `hospital_canvas_v1`/`hospital_connected_free_v1` ở `0.05 m/cell`; runtime 0.10 được reproject 2x trước metric calculation.
- Đổi runtime resolution là protocol change; mọi official comparison Nearest/MapEx phải dùng `hospital_v2`.
- Cartographer is currently a **debug mapping alternative**, not silently part of `hospital_v2`. If adopted for official runs, protocol identity/provenance must change and Nearest/MapEx/proposed method must all rerun under the same mapping stack.
- Cartographer probability output must be normalized to the same discrete occupancy convention expected by the MapEx frontier generator before policy comparison; this normalization is a mapping-adapter concern, not a change to Nearest ranking/frontier code.
- The temporary Cartographer performance A/B (`online correlative=false`, `/map` every 3 s) did not eliminate the jerk and is no longer active; current Cartographer is back to the accuracy-first matcher and 1 s map publication.
- The new local-window SLAM Toolbox frontend is **debug-only** and is not part of `hospital_v2`. Its first A/B deliberately keeps the normal `config/slam.yaml` backend parameters unchanged except for the input scan topic so any difference is attributable to the frontend. Adoption into official comparison would require protocol versioning and the same mapping stack for all methods.
- Active `nf_basic.py` does not use path-guided recovery or event-driven BT replanning. It performs nearest-Euclidean ranking, validates candidates with `ComputePathToPose` nearest-first, drops no-path candidates for the current sweep, and sends `NavigateToPose` only to a planner-valid exact frontier x/y.
- No-path evidence is not permanently blacklisted: ROS online maps can change, so the same frontier may be revalidated on a later sweep. This matches the protocol rule that planner no-path is transient evidence rather than permanent unreachable state.
- Nearest, full MapEx và proposed method phải dùng cùng Hospital adaptations.
- Exact frontier **x/y** là execution position; frontier không có terminal-yaw objective.
- Validation path chỉ chứng minh planner reachability; NavigateToPose có thể replan nên executed distance lấy từ odometry.
- Controller execution failure không được biến planner-reachable frontier thành permanent unreachable.
- Official `hospital_v2` termination/revalidation rules remain defined by `EXPERIMENT_PROTOCOL.md`; active `nf_basic.py` is still a debug runner and does not implement the full official completion guard/recorder semantics.
- Official run phải clean git và lưu effective runtime provenance.
- Simulator seed intentionally uncontrolled; repeated runs xử lý variability.
- Pilot/debug không tính vào official benchmark.

## Latest result

`nearest_pilot_005`: diagnostic 1 m deadlock (`hospital_v1`).  
`nearest_pilot_007`: diagnostic tolerance-snapped execution-goal bug (`hospital_v1`).  
`nearest_pilot_008`: exact-frontier execution PASS nhưng map warp (`hospital_v1`).  
`nearest_pilot_009`: phát hiện near-zero controller command và các execution-semantics gaps (`hospital_v1`).  
2026-08-28 historical debug experiments: path-guided recovery, event-driven replanning and robust completion were implemented and later rolled back from active `nf_basic.py`.  
2026-08-28 active `nf_basic.py`: minimal Nearest Frontier restored, then nearest-first `ComputePathToPose` reachability validation added so blocked/no-path frontiers are dropped for the current sweep instead of being repeatedly sent to `NavigateToPose`; runtime curtain-case validation pending.  
2026-08-28 Cartographer runtime #1: IMU sensor-frame TF mismatch fixed.  
2026-08-28 Cartographer runtime #2: Cartographer produced maps, but raw probability-valued OccupancyGrid was incompatible with `nf_basic.py` exact-free (`==0`) frontier semantics; `/cartographer_map -> /map` discrete normalization bridge committed.  
2026-08-28 Cartographer runtime #3: normalized `/map` allows `nf_basic.py` to select frontiers and move; visible wall-time jerk coincides with approximately `/cmd_vel=14 Hz`, `/odom=14 Hz`, `/scan=2.6 Hz`. Disabling online correlative matching and slowing `/map` publication to 3 s did not remove the jerk, so both changes were reverted. Current hypothesis: Gazebo/physics real-time performance.  
2026-08-28 local-window SLAM debug: rolling scan-to-local-window ICP frontend + matching SLAM/launch profile implemented with conservative correction bounds; runtime validation pending.  
Current diagnostic state: restored historical `control_tb4.py` and historical `hospital_slam.yaml` (`0.05 m/cell`) from commit `9f404a9` for regression testing only.  
`control_tb4_mapex.py`: Stage-3 policy implementation prepared from paper/source; no runtime result yet.
