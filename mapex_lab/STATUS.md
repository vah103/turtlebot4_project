# STATUS

## Current stage

**Stage 2 — MapEx Nearest baseline (Hospital v2 resolution-change validation before official runs)**

## Done

- Renamed research folder from `mapex_hospital_research/` to `mapex_lab/`; runtime files remain unchanged apart from path references.
- Removed obsolete legacy/debug Nearest branches that are no longer part of the active roadmap: `autonomous_exploration/`, `control_tb4_hospital.py`, `launch/hospital_nearest.launch.py`, `launch/nearest_frontier.launch.py`, `launch/nearest_frontier_full.launch.py`, `scripts/mapex_nearest_simple.py`, `scripts/mapex_nearest_full.py`, `scripts/test_nearest_frontier_nav.py`, and `scripts/test_nearest_frontier_run.py`. Active `nf_basic.py`, stock runtime, MapEx preparation, recorders, analysis, GT, protocol, references, and research artifacts are retained.
- Fixed canvas `hospital_canvas_v1`: `0.05 m`, `1504 x 2123`, origin `(-25.6,-60.1)`.
- Frozen ROI `hospital_connected_free_v1`: denominator `215435`, SHA-256 `05d45b7aba66cb6dbb71e0406005f4a3e21875901af3ae72164b17f1d3add8d1`.
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
- Added debug-only path-guided intermediate-goal recovery to `nf_basic.py` plus `behavior_trees/navigate_to_pose_subgoal.xml`: the selected frontier remains fixed; on a fast-fail Nav2 abort, `nf_basic.py` chooses a temporary point on the latest main-goal `/plan`, navigates there, then retries the same frontier from the new pose. No blacklist or alternate frontier selection is introduced.
- Added conservative completion verification to debug `nf_basic.py`: COMPLETE is only emitted after zero frontier regions `>10` persist across `5` distinct `/map` updates, sweeps are at least `2 s` apart, navigation has been idle for at least `10 s`, and node age is at least `20 s`. Remaining frontiers inside the debug `0.5 m` filter produce `BLOCKED_BY_MIN_DISTANCE`, not a false COMPLETE. Completion is published latched on `/frontier_exploration_complete` and machine-readable status on `/frontier_exploration_status`; once complete, no further goals are sent.
- Added a Cartographer 2D debug alternative for long-run SLAM drift diagnosis: `config/cartographer_hospital_2d.lua` + `launch/cartographer.launch.py`. It uses `/scan` + wheel `/odom` + simulated `/imu`, 0.05 m internal submaps, pose-graph loop closure, and publishes `/map` at 0.10 m/cell for compatibility with `nf_basic.py`/Nav2. This is not yet an official `hospital_v2` protocol change.
- Restored `control_tb4.py` exactly to historical blob `44f262b6ffa904042d1f2633d8f2ced95e645513` from commit `9f404a9`.
- Stage-3 preparation: `control_tb4_mapex.py` dùng `MAPEX_RESOLUTION_M = 0.10`; với Hospital v2 source map `0.10`, bước downsample trở thành factor `1`, nên frontier/prediction cùng grid 0.10.

## In progress

- Runtime validation of the Cartographer Hospital debug stack: verify `/scan`, `/odom`, `/imu`, TF `map -> odom -> base_link`, `/map.info.resolution=0.10`, then revisit previously mapped corridors and compare wall overlap/warp against the slam_toolbox run.
- Runtime validation of the debug `stock.launch.py + nf_basic.py` path-guided recovery and completion guard: verify fast controller failure return, temporary path subgoal behavior, retry of the same frontier, and final 5-sweep completion only after frontiers truly disappear.
- Regression test: old `control_tb4.py` + historical `hospital_slam.yaml` (`0.05 m/cell`) to determine whether the new no-frontier startup behavior is caused by runtime-grid changes rather than the controller file itself.
- Hospital v2 official validation is paused while this diagnostic rollback is active.
- `control_tb4_mapex.py` vẫn là Stage-3 preparation only: chưa runtime-validate, chưa phải official MapEx benchmark runner và không thay đổi current stage.

## Next actions

1. Install ROS 2 Jazzy Cartographer packages, pull the new debug launch/config, and run `launch/cartographer.launch.py` with `nf_basic.py`.
2. Verify Cartographer input/output and TF: `/scan`, `/odom`, `/imu`, `/map`, `map -> odom -> base_link`; confirm `/map.info.resolution=0.10`.
3. Continue long enough to return to old corridors; inspect whether wall overlap remains aligned and whether Nav2 avoids the previous immediate `PATIENCE_EXCEEDED` state.
4. If Cartographer is clearly more stable, decide whether to promote it to the official benchmark stack. Promotion requires a new protocol version and all compared methods must use the same Cartographer stack.
5. Runtime-test `stock.launch.py + nf_basic.py` and capture one case where the main frontier aborts, the path-guided subgoal is attempted, and the same main frontier is retried.
6. Continue the same run to terminal exploration state and verify `/frontier_exploration_complete=true` appears only after 5 distinct-map no-frontier sweeps and no new navigation goal follows.
7. Pull/rebuild `frontier_exploration`, run `hospital_flat_stack.launch.py`, then run restored `control_tb4.py` and check whether startup exploration works again at `0.05 m/cell`.
8. Record whether frontier groups/goals behave like the historical run.
9. After the regression test, restore `hospital_slam.yaml` to Hospital v2 `resolution: 0.10` before continuing official validation.
10. Resume Hospital v2 validation only after the diagnostic rollback is removed.

## Important decisions

- Hospital implementation là MapEx frontier/ranking port + ROS/Nav2 execution adapter, không gọi là MapEx nguyên xi.
- Hospital v2 target runtime map/policy resolution = `0.10 m/cell`; the current `hospital_slam.yaml=0.05` state is temporary diagnostic-only.
- Evaluation vẫn dùng frozen `hospital_canvas_v1`/`hospital_connected_free_v1` ở `0.05 m/cell`; runtime 0.10 được reproject 2x trước metric calculation.
- Đổi runtime resolution là protocol change; mọi official comparison Nearest/MapEx phải dùng `hospital_v2`.
- Cartographer is currently a **debug mapping alternative**, not silently part of `hospital_v2`. If adopted for official runs, protocol identity/provenance must change and Nearest/MapEx/proposed method must all rerun under the same mapping stack.
- Nearest, full MapEx và proposed method phải dùng cùng Hospital adaptations.
- Exact frontier **x/y** là execution position; frontier không có terminal-yaw objective.
- Validation path chỉ chứng minh planner reachability; NavigateToPose có thể replan nên executed distance lấy từ odometry.
- Planner no-path trong ROS online là transient evidence và phải được revalidate trước completion.
- Controller execution failure không được biến planner-reachable frontier thành permanent unreachable.
- `nf_basic.py` path-guided subgoal recovery và conservative completion guard hiện là **debug execution experiment only**, không phải `hospital_v2` official baseline semantics. Nếu sau này dùng trong benchmark chính thức thì execution adapter/completion policy phải được protocol-versioned và áp dụng công bằng cho Nearest/MapEx/proposed method.
- Debug completion intentionally prefers false-negative/non-termination over false-positive completion: a pending main frontier or any large frontier region prevents COMPLETE.
- Official run phải clean git và lưu effective runtime provenance.
- Simulator seed intentionally uncontrolled; repeated runs xử lý variability.
- Pilot/debug không tính vào official benchmark.

## Latest result

`nearest_pilot_005`: diagnostic 1 m deadlock (`hospital_v1`).  
`nearest_pilot_007`: diagnostic tolerance-snapped execution-goal bug (`hospital_v1`).  
`nearest_pilot_008`: exact-frontier execution PASS nhưng map warp (`hospital_v1`).  
`nearest_pilot_009`: phát hiện near-zero controller command và các execution-semantics gaps (`hospital_v1`).  
2026-08-28 stock/debug: path-guided intermediate-goal recovery + robust conservative completion guard implemented in `nf_basic.py`; runtime validation pending.  
2026-08-28 Cartographer debug alternative: Hospital launch/config implemented with odom + IMU + local submaps + pose-graph optimization; runtime validation pending.  
Current diagnostic state: restored historical `control_tb4.py` and historical `hospital_slam.yaml` (`0.05 m/cell`) from commit `9f404a9` for regression testing only.  
`control_tb4_mapex.py`: Stage-3 policy implementation prepared from paper/source; no runtime result yet.
