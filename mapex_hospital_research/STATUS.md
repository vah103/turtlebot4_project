# STATUS

## Current stage

**Stage 2 — MapEx Nearest baseline (corrected execution pilot before official runs)**

## Done

- Fixed canvas `hospital_canvas_v1`: `0.05 m`, `1504 x 2123`, origin `(-25.6,-60.1)`.
- Frozen ROI `hospital_connected_free_v1`: denominator `215435`, SHA-256 `05d45b7aba66cb6dbb71e0406005f4a3e21875901af3ae72164b17f1d3add8d1`.
- Official MapEx source pinned: `53636bd1c79153acc3c74a532837d78c926bae5e`.
- Nearest frontier semantics ported: 8-neighbour frontier, 8-connected regions, region `>10`, representative gần arithmetic mean nhất, Euclidean ranking.
- Hospital bỏ original MapEx `<1 m` rejection vì `nearest_pilot_005` chứng minh startup deadlock; `below_1m` chỉ còn diagnostic.
- `nearest_pilot_007` phát hiện planner endpoint tolerance-snapped bị dùng sai làm NavigateToPose goal; đã sửa exact frontier x/y execution và `GridBased.tolerance=0.0`.
- `nearest_pilot_008` xác nhận exact-frontier logging nhưng lộ long-run map warping; mapping profile đổi sang `0.45 m/s` + SLAM `0.10 m / 0.10 rad`.
- `nearest_pilot_009` lộ thêm runtime/controller problem: planner replan liên tục nhưng `/cmd_vel_nav`, `/cmd_vel`, `/odom` gần zero.
- Rà soát code sau pilot_009 phát hiện và đã sửa các vấn đề:
  - frontier policy là position-only nhưng code từng ép quaternion yaw `0`; Hospital giờ bỏ final-yaw objective (`yaw_goal_tolerance=pi`, `GoalAngleCritic=false`);
  - `ComputePathToPose` chỉ được accept khi ROS action `STATUS_SUCCEEDED` **và** path non-empty;
  - stable non-empty no-path set phải planner revalidation mỗi `2 s`, không cache một lần rồi completion;
  - planner-valid goal execution failure chỉ suppress `0.25 m / 30 s`, không permanent;
  - `selected_path_length_m` được định nghĩa rõ là validation-path length, không phải executed trajectory;
  - recorder archive/hash cả base Nav2 params và `runtime_nav2_merged.yaml` effective config;
  - validator kiểm effective Nav2 config, action status, terminal revalidation và guard `SUCCEEDED` nhưng odometry gần như không di chuyển.
- `hospital_slam_no_loop.yaml` đã đồng bộ `0.10 m / 0.10 rad` với active SLAM profile để A/B loop-closure không bị confound.
- Preflight parse YAML thực, kiểm source + installed configs thay vì chỉ tìm chuỗi text.

## In progress

- `nearest_pilot_010`: corrected execution/revalidation/provenance pilot.
- Kiểm robot không còn đứng yên do artificial final-yaw requirement.
- Kiểm map geometry dài hạn; nếu vẫn warp, dùng synchronized `hospital_slam_no_loop.yaml` A/B để kiểm false loop closure.

## Next actions

1. Dừng pilot_009; không dùng làm benchmark.
2. Pull repo và rebuild `frontier_exploration`.
3. Chạy `scripts/preflight_nearest.py`; phải PASS toàn bộ runtime config checks.
4. Chạy default `hospital_nearest.launch.py` → `nearest_pilot_010`.
5. Sau vài goal chạy validator `--run-id nearest_pilot_010 --allow-running`.
6. Kiểm robot thực sự di chuyển và `/cmd_vel_nav` không còn near-zero kéo dài tại một frontier.
7. Quan sát map dài hạn. Nếu map vẫn warp rõ, chưa chạy official; chạy A/B no-loop để xác định loop-closure.
8. Chỉ khi pilot_010 runtime + map geometry + validator đều ổn mới khóa code và bắt đầu `nearest_001 ... nearest_005`.

## Important decisions

- Hospital implementation là MapEx frontier/ranking port + ROS/Nav2 execution adapter, không gọi là MapEx nguyên xi.
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

`nearest_pilot_005`: diagnostic 1 m deadlock.  
`nearest_pilot_007`: diagnostic tolerance-snapped execution-goal bug.  
`nearest_pilot_008`: exact-frontier execution PASS nhưng map warp.  
`nearest_pilot_009`: phát hiện near-zero controller command và các execution-semantics gaps.  
`nearest_pilot_010`: pilot kế tiếp sau khi sửa toàn bộ các gap trên.
