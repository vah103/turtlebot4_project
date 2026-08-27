# STATUS

## Current stage

**Stage 2 — MapEx Nearest baseline (Hospital v2 resolution-change validation before official runs)**

## Done

- Fixed canvas `hospital_canvas_v1`: `0.05 m`, `1504 x 2123`, origin `(-25.6,-60.1)`.
- Frozen ROI `hospital_connected_free_v1`: denominator `215435`, SHA-256 `05d45b7aba66cb6dbb71e0406005f4a3e21875901af3ae72164b17f1d3add8d1`.
- Hospital runtime protocol advanced to `hospital_v2`: SLAM/policy map is now `0.10 m/cell`, while fixed evaluation canvas/ROI remain frozen at `0.05 m/cell`.
- Recorder reprojection updated: axis-aligned runtime maps whose resolution is an integer multiple of `0.05 m` are expanded nearest-neighbour onto `hospital_canvas_v1`; a `0.10 m` SLAM cell becomes a `2 x 2` block on the evaluation canvas.
- `hospital_slam.yaml` and `hospital_slam_no_loop.yaml` both use runtime `resolution: 0.10`; loop-search/correlation resolutions were intentionally not changed because they are matcher parameters, not occupancy-grid resolution.
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
- Added `launch/stock.launch.py` as a reference/debug stack: project Hospital simulation + stock TurtleBot4 Nav2 + near-stock TurtleBot4 SLAM. Debug SLAM uses `config/slam.yaml`, copied from upstream TurtleBot4 with only `max_laser_range: 20.0` changed to match the simulated Hospital LiDAR; this stack is **not** part of the official `hospital_v2` benchmark protocol.
- Added `step.py` as a debug-only wrapper around `control_tb4.py`: one frontier goal at a time, waits for the real Nav2 terminal result, prints compact motion/navigation diagnostics, then pauses for ENTER before the next goal. `control_tb4.py` itself remains unchanged by this debug mode.
- Added `step0.py` as a temporary debug wrapper that sets `min_distance_threshold = 0.0` so the current centroid-selection behavior can be observed without the 0.3 m rejection filter; this is diagnostic only and does not change the benchmark policy.
- Stage-3 preparation: `control_tb4_mapex.py` dùng `MAPEX_RESOLUTION_M = 0.10`; với Hospital v2 source map `0.10`, bước downsample trở thành factor `1`, nên frontier/prediction cùng grid 0.10.

## In progress

- Chưa runtime-validate Hospital v2 sau đổi SLAM resolution `0.05 -> 0.10`.
- Cần kiểm `/map.info.resolution == 0.10`, frontier Nearest hoạt động bình thường, và recorder vẫn tạo fixed canvas `1504 x 2123`/coverage hợp lệ.
- Cần kiểm map geometry dài hạn ở runtime grid mới; nếu vẫn warp, dùng synchronized `hospital_slam_no_loop.yaml` A/B để kiểm false loop closure.
- `control_tb4_mapex.py` vẫn là Stage-3 preparation only: chưa runtime-validate, chưa phải official MapEx benchmark runner và không thay đổi current stage.

## Next actions

1. Pull repo và rebuild `frontier_exploration` để installed `hospital_slam.yaml` nhận `resolution: 0.10`.
2. Khởi động Hospital stack và xác nhận `ros2 topic echo /map --once` cho `info.resolution: 0.1`.
3. Chạy một Nearest pilot mới dưới `protocol_version=hospital_v2`; không tái sử dụng pilot cũ làm evidence cho v2.
4. Trong pilot, kiểm frontier candidate row/col chạy trực tiếp trên grid `0.10` và exact frontier x/y vẫn được gửi Nav2.
5. Kiểm recorder raw snapshot giữ `resolution=0.10`, còn `fixed_canvas` vẫn shape `(2123,1504)` và coverage không NaN khi ROI local tồn tại.
6. Quan sát map geometry dài hạn. Nếu map vẫn warp rõ, chưa chạy official; chạy A/B no-loop ở cùng runtime resolution 0.10.
7. Chỉ khi Hospital v2 runtime + map geometry + logging đều ổn mới khóa code và bắt đầu official Nearest runs.
8. Stage 3 chỉ bắt đầu sau khi Stage 2 ổn định: chuẩn bị official MapEx LaMa runtime + ensemble weights, smoke-test `control_tb4_mapex.py`, rồi ghép execution adapter/logging giống Nearest.

## Important decisions

- Hospital implementation là MapEx frontier/ranking port + ROS/Nav2 execution adapter, không gọi là MapEx nguyên xi.
- Hospital v2 runtime map/policy resolution = `0.10 m/cell`; Nearest và MapEx phải dùng cùng runtime resolution.
- Evaluation vẫn dùng frozen `hospital_canvas_v1`/`hospital_connected_free_v1` ở `0.05 m/cell`; runtime 0.10 được reproject 2x trước metric calculation.
- Đổi runtime resolution là protocol change; mọi official comparison Nearest/MapEx phải dùng `hospital_v2`. Pilot `hospital_v1` chỉ là lịch sử/diagnostic.
- Nearest, full MapEx và proposed method phải dùng cùng Hospital adaptations.
- Exact frontier **x/y** là execution position; frontier không có terminal-yaw objective.
- Validation path chỉ chứng minh planner reachability; NavigateToPose có thể replan nên executed distance lấy từ odometry.
- Planner no-path trong ROS online là transient evidence và phải được revalidate trước completion.
- Controller execution failure không được biến planner-reachable frontier thành permanent unreachable.
- Hospital planner tolerance `0.0 m`; mapping speed `0.45 m/s`; SLAM keyframe spacing `0.10 m / 0.10 rad`.
- Official run phải clean git và lưu effective runtime provenance.
- Simulator seed intentionally uncontrolled; repeated runs xử lý variability.
- Pilot/debug không tính vào official benchmark.

## Latest result

`nearest_pilot_005`: diagnostic 1 m deadlock (`hospital_v1`).  
`nearest_pilot_007`: diagnostic tolerance-snapped execution-goal bug (`hospital_v1`).  
`nearest_pilot_008`: exact-frontier execution PASS nhưng map warp (`hospital_v1`).  
`nearest_pilot_009`: phát hiện near-zero controller command và các execution-semantics gaps (`hospital_v1`).  
Hospital v2 code/config transition: runtime SLAM/policy grid switched to `0.10 m/cell`; frozen evaluation canvas/ROI retained at `0.05 m/cell`; runtime validation pending.  
`control_tb4_mapex.py`: Stage-3 policy implementation prepared from paper/source; no runtime result yet.