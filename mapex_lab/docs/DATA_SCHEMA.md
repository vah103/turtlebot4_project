# Data Schema

## Run layout

```text
experiments/<method>/<run_id>/
├── metadata.json
├── runtime_nav2_merged.yaml
├── metrics.csv
├── trajectory.csv
├── decisions.csv
├── policy_decisions.csv
├── candidates.csv
├── snapshots.csv
├── maps/
├── predictions/
├── variance/
├── visibility/
├── decisions/
│   └── policy_decision_000001/
│       ├── decision.json
│       ├── candidates.csv
│       ├── observed_map_raw.npz
│       └── observed_map_canvas.npz
└── logs/
```

## Canonical grids

- Runtime SLAM/policy grid for `hospital_v2`: `0.10 m/cell`.
- Fixed evaluation canvas: `hospital_canvas_v1`, `0.05 m`, `1504 x 2123`, origin `(-25.6,-60.1)`.
- Evaluation ROI: `hospital_connected_free_v1`, denominator `215435`.
- ROI SHA-256: `05d45b7aba66cb6dbb71e0406005f4a3e21875901af3ae72164b17f1d3add8d1`.

Runtime resolution và evaluation resolution là độc lập. Với map axis-aligned `0.10 m`, mỗi runtime cell được expand nearest-neighbour thành khối `2 x 2` trên fixed canvas `0.05 m` trước khi tính `known_fraction`/`coverage`. Raw map luôn giữ nguyên resolution gốc trong NPZ.

`known_fraction` dùng full fixed canvas. `coverage` dùng frozen ROI. Không crop động và không normalize final state riêng từng run.

## Benchmark clock

`time_s=0` lấy từ `/frontier_exploration_start`, được publish **trước candidate computation của policy decision đầu tiên**.

```text
exploration_start_sim_s
exploration_start_source = first_policy_decision_before_compute
```

## Exact policy decision map

Mỗi policy decision giữ OccupancyGrid thực sự được freeze để tạo/rank candidate:

```text
observed_map_raw.npz
observed_map_canvas.npz
```

Raw NPZ tối thiểu:

```text
data
resolution
width
height
origin_x
origin_y
origin_yaw
frame_id
source_stamp_s
```

Canvas NPZ dùng canonical canvas để evaluation/alignment. `observed_map_canvas.npz` không được hiểu là policy input nếu policy đang chạy ở `0.10 m`; nó là bản reproject phục vụ logging/evaluation.

## Exact exhausted state và planner revalidation

Khi không có ranked candidate:

```text
outcome = exhausted_no_ranked_candidate
candidate_count = 0
terminal_reason = zero_ranked_candidates
```

Nếu có candidates nhưng một planner sweep không tìm được candidate reachable:

```text
outcome = no_nav2_reachable_ranked_candidate
terminal_reason = all_ranked_candidates_failed_nav2_path_validation
nav2_path_success_count = 0
nav2_checked_count = candidate_count
nav2_no_path_count + nav2_rejected_count + nav2_error_count = nav2_checked_count
planner_revalidation_period_s = 2.0
```

Một non-empty no-path state không được cache vĩnh viễn. Trước completion, stable candidate set phải được revalidate bằng Nav2 nhiều sweep theo completion protocol.

## metrics.csv

```text
time_s,distance_m,known_fraction,coverage,occupied_iou,tu
```

`distance_m` là cumulative odometry distance. `occupied_iou`/`tu` có thể để trống online nếu raw snapshots được giữ để tính offline.

## trajectory.csv

```text
time_s,x,y,yaw,cumulative_distance_m
```

Đây là odometry trajectory. Exact policy replay dùng map-frame robot pose trong policy decision.

## decisions.csv

Mỗi selected navigation goal:

```text
decision_id,
policy_decision_id,
time_s,
known_fraction,
coverage,
num_candidates,
selected_candidate_id,
selected_distance_m,
selected_path_length_m,
frontier_x,frontier_y,
planner_endpoint_x,planner_endpoint_y,
planner_endpoint_to_frontier_m,
goal_x,goal_y,
goal_source,
goal_yaw_semantics,
robot_map_x,robot_map_y,robot_map_yaw,
result,
navigation_detail,
failure_reason
```

Semantics bắt buộc:

```text
frontier_x/frontier_y          = exact MapEx frontier center
planner_endpoint_x/y           = last pose returned by validation ComputePathToPose
planner_endpoint_to_frontier_m = distance(planner endpoint, exact frontier)
goal_x/goal_y                  = actual NavigateToPose position
goal_source                    = exact_frontier_center
goal_yaw_semantics             = ignored_by_hospital_goal_checker
```

Official invariant:

```text
distance((goal_x,goal_y),(frontier_x,frontier_y)) <= 0.001 m
goal_source = exact_frontier_center
goal_yaw_semantics = ignored_by_hospital_goal_checker
```

`selected_path_length_m` có semantics:

```text
planner_validation_path_length_not_executed_trajectory
```

Nó **không** phải quãng đường robot thực sự chạy, vì `NavigateToPose` có thể replan. Quãng đường thực lấy từ `trajectory.csv` / `metrics.csv.distance_m`.

Khóa join chính:

```text
policy_decision_id + selected_candidate_id
```

## policy_decisions.csv

```text
policy_decision_id,
sim_time_s,
map_stamp_s,
robot_x,robot_y,robot_yaw,
candidate_compute_ms,
candidate_count,
below_1m_count,
hospital_1m_rule_enforced,
nav2_checked_count,
nav2_path_success_count,
nav2_no_path_count,
nav2_rejected_count,
nav2_error_count,
planner_revalidation_period_s,
selected_candidate_id,
selected_rank,
selected_x,selected_y,
selected_distance_m,
selected_path_length_m,
outcome,
terminal_reason
```

`robot_x/y/yaw` là map-frame pose dùng cho ranking.

## candidates.csv

Mỗi candidate:

```text
policy_decision_id,
candidate_id,
raw_rank,
policy_rank,
row,col,
x,y,
distance_cells,distance_m,
below_1m,
execution_suppressed,
status,
nav2_action_status,
planner_check_ms,
selected,
selected_path_length_m,
execution_result
```

`row,col,distance_cells` là trên runtime policy grid `0.10 m` trong `hospital_v2`; `x,y,distance_m` là metric coordinates và là trường dùng để so sánh vật lý giữa methods.

`nav2_action_status` là ROS action status của `ComputePathToPose`. Candidate chỉ được `selected=true` khi:

```text
nav2_action_status == GoalStatus.STATUS_SUCCEEDED (4)
AND path non-empty
```

### Hospital `<1m` semantics

`below_1m is diagnostic only`. Candidate `<1m` không bị reject chỉ vì khoảng cách.

Status hợp lệ có thể gồm:

```text
ranked
execution_suppressed
checking_nav2
checking_nav2_below_1m_allowed
nav2_rejected
nav2_request_error
nav2_result_error
nav2_action_failed
nav2_no_path
selected
```

`rejected_lt_1m` là status không hợp lệ cho Hospital official runs.

### Execution-failure cooldown

Planner-valid frontier mà `NavigateToPose` fail chỉ bị execution suppression tạm thời:

```text
radius = 0.25 m
cooldown = 30 s
```

Không có permanent suppression do controller failure. Một navigation success xóa temporary cooldowns cũ.

## snapshots.csv

```text
snapshot_id,event,time_s,coverage,known_fraction,distance_m,
robot_map_x,robot_map_y,robot_map_yaw,
raw_map_file,canvas_map_file,source_map_stamp_s,canvas_id
```

Terminated run phải có final snapshot.

## metadata.json

Tối thiểu cho official run:

```json
{
  "run_id": "nearest_001",
  "method": "nearest",
  "git_commit": "...",
  "git_dirty_at_recorder_start": false,
  "protocol_version": "hospital_v2",
  "runtime_map_resolution_m": 0.10,
  "fixed_canvas_id": "hospital_canvas_v1",
  "fixed_canvas_resolution_m": 0.05,
  "evaluation_roi_id": "hospital_connected_free_v1",
  "exploration_start_sim_s": 0.0,
  "exploration_start_source": "first_policy_decision_before_compute",
  "execution_goal_semantics": "exact_frontier_center_xy",
  "goal_yaw_semantics": "ignored_by_hospital_goal_checker",
  "planner_path_semantics": "reachability_evidence_only",
  "selected_path_length_semantics": "planner_validation_path_length_not_executed_trajectory",
  "runtime_nav2_merged_file": "runtime_nav2_merged.yaml",
  "sim_seed": null,
  "sim_seed_policy": "intentionally_uncontrolled_gazebo_default_multiple_run_statistics",
  "config_sha256": {},
  "termination_reason": null
}
```

`config_sha256` phải gồm source research code, installed project runtime files, và thêm:

```text
installed_nav2_base_params
runtime_nav2_merged
```

`runtime_nav2_merged.yaml` là effective base Nav2 YAML + Hospital override của run đó và phải được giữ trong run directory.

## Position-only frontier goal runtime config

Effective merged Nav2 config phải xác nhận:

```text
planner_server.GridBased.tolerance = 0.0
controller_server.general_goal_checker.yaw_goal_tolerance = pi
controller_server.FollowPath.GoalAngleCritic.enabled = false
controller_server.FollowPath.vx_max = 0.45
```

Final yaw không phải frontier-policy objective.

## SLAM A/B diagnostic

`hospital_slam_no_loop.yaml` phải giữ cùng:

```text
resolution = 0.10
minimum_travel_distance = 0.10
minimum_travel_heading = 0.10
minimum_time_interval = 0.15
```

với active `hospital_slam.yaml`; khác biệt chính dùng cho A/B là `do_loop_closing=false`.

## Validation invariants

Post-run validator kiểm tra **data/protocol integrity + experiment health**:

- protocol là `hospital_v2` và raw runtime map resolution là `0.10 m/cell`;
- fixed evaluation canvas vẫn là `hospital_canvas_v1` ở `0.05 m/cell`;
- benchmark clock đúng;
- metrics time/distance monotonic, coverage bounds đúng;
- exact decision/snapshot NPZ tồn tại/readable;
- selected candidate join bằng ID;
- không có `rejected_lt_1m`;
- selected planner candidate phải có `nav2_action_status=4`;
- execution goal `x/y` trùng exact frontier;
- `goal_yaw_semantics=ignored_by_hospital_goal_checker`;
- planner endpoint audit tự nhất quán;
- effective merged Nav2 YAML được archive/hash và có đúng tolerance/yaw/speed semantics;
- terminated run không còn `PENDING`, failed goal có failure reason;
- non-empty completion có repeated stable planner sweeps, không dùng cached no-path một lần;
- no-path terminal counts cộng đúng và success count bằng 0;
- official run clean git;
- guard phát hiện pathology `SUCCEEDED` nhưng odometry gần như không di chuyển dù selected frontier ở xa.

## Full MapEx decision directory sau này

MapEx dùng lại toàn bộ exact observed-map/candidate/execution schema trên và thêm:

```text
g1.png
g2.png
g3.png
mean.npy
variance.npy
visibility/
```

Candidate table MapEx thêm:

```text
predicted_visible_cells,ig,score,rank
```

Nếu có structural GT:

```text
gt_visible_cells,gt_gain,gt_gain_per_m,gt_rank,prediction_error_summary
```

## Lightweight GitHub results

Raw `.npz`/logs giữ local hoặc external storage. Chỉ commit summary/figure/table nhỏ dưới `results/`.
