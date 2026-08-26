# Data Schema

## Run layout

```text
experiments/<method>/<run_id>/
├── metadata.json
├── metrics.csv
├── trajectory.csv
├── decisions.csv
├── policy_decisions.csv
├── candidates.csv
├── snapshots.csv
├── maps/
│   ├── periodic_*_raw.npz
│   ├── periodic_*_canvas.npz
│   └── final_*_raw/canvas.npz
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

- Fixed canvas: `hospital_canvas_v1`, `0.05 m`, `1504 x 2123`, origin `(-25.6,-60.1)`.
- Evaluation ROI: `hospital_connected_free_v1`, denominator `215435`.
- ROI SHA-256: `05d45b7aba66cb6dbb71e0406005f4a3e21875901af3ae72164b17f1d3add8d1`.

`known_fraction` dùng full fixed canvas. `coverage` dùng frozen ROI. Không crop động và không normalize final state riêng từng run.

## Benchmark clock

`time_s=0` lấy từ `/frontier_exploration_start`, được publish **trước candidate computation của policy decision đầu tiên**.

Recorder phải lưu:

```text
exploration_start_sim_s
exploration_start_source = first_policy_decision_before_compute
```

Nhờ vậy computation của decision đầu tiên được tính vào Coverage-vs-time cho cả Nearest và MapEx.

## Exact policy decision map

Đây là OccupancyGrid thực sự được freeze để tạo/rank candidate, không phải `latest_map` muộn hơn.

Mỗi policy decision có:

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

Canvas NPZ dùng canonical canvas để evaluation/alignment.

## Exact exhausted state

Ngay cả khi không có ranked candidate, policy vẫn phải tạo một `policy_decision_*` với exact map/pose và:

```text
outcome = exhausted_no_ranked_candidate
candidate_count = 0
terminal_reason = zero_ranked_candidates
```

Nếu có candidates nhưng không candidate nào Nav2-reachable, terminal decision phải được ghi **trước completion window**:

```text
outcome = no_nav2_reachable_ranked_candidate
terminal_reason = all_ranked_candidates_failed_nav2_path_validation
nav2_path_success_count = 0
nav2_checked_count = candidate_count
nav2_no_path_count + nav2_rejected_count + nav2_error_count = nav2_checked_count
```

Điều này cho phép replay termination chính xác mà không suy từ periodic snapshot.

## metrics.csv

```text
time_s,distance_m,known_fraction,coverage,occupied_iou,tu
```

`occupied_iou` và `tu` có thể để trống khi chạy nếu periodic/final raw maps được lưu đầy đủ để tính offline sau khi evaluator được chốt.

## trajectory.csv

```text
time_s,x,y,yaw,cumulative_distance_m
```

Đây là odometry trajectory. Exact policy replay phải dùng map-frame robot pose trong `policy_decisions.csv`/`decision.json`.

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
robot_map_x,robot_map_y,robot_map_yaw,
result,
navigation_detail,
failure_reason
```

Semantics bắt buộc cho `hospital_v1` official runs:

```text
frontier_x/frontier_y          = exact MapEx frontier center
planner_endpoint_x/y           = last pose returned by ComputePathToPose
planner_endpoint_to_frontier_m = distance(planner endpoint, exact frontier)
goal_x/goal_y                  = actual NavigateToPose goal
goal_source                    = exact_frontier_center
```

`ComputePathToPose` có planner tolerance nên `planner_endpoint_x/y` có thể khác frontier. Endpoint đó **không được dùng làm execution goal**. Validator phải xác nhận:

```text
distance((goal_x,goal_y),(frontier_x,frontier_y)) <= 0.001 m
goal_source = exact_frontier_center
```

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
selected_candidate_id,
selected_rank,
selected_x,selected_y,
selected_distance_m,
selected_path_length_m,
outcome,
terminal_reason
```

`robot_x/y/yaw` là map-frame pose dùng cho ranking.

Các field `nav2_*_count` là audit của planner trên exact frozen decision. Với terminal `no_nav2_reachable_ranked_candidate`, validator phải xác nhận đã kiểm tra đủ toàn bộ ranked candidate và không có path thành công.

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
planner_check_ms,
selected,
selected_path_length_m,
execution_result
```

`candidate_id` ổn định trong từng `policy_decision_id`, ví dụ `candidate_0001`.

### Hospital `<1m` semantics

`below_1m is diagnostic only` trong Hospital benchmark. Candidate `<1m` **không bị reject chỉ vì khoảng cách**; nó giữ rank và được gửi sang Nav2 như candidate khác.

Status hợp lệ có thể gồm:

```text
ranked
execution_suppressed
checking_nav2
checking_nav2_below_1m_allowed
nav2_rejected
nav2_request_error
nav2_result_error
nav2_no_path
selected
```

`rejected_lt_1m` là status không hợp lệ cho Hospital official runs và validator phải FAIL nếu xuất hiện.

## snapshots.csv

Recorder snapshots định kỳ/final:

```text
snapshot_id,event,time_s,coverage,known_fraction,distance_m,
robot_map_x,robot_map_y,robot_map_yaw,
raw_map_file,canvas_map_file,source_map_stamp_s,canvas_id
```

Mỗi referenced NPZ phải tồn tại và readable. Terminated run phải có final snapshot.

## metadata.json

Tối thiểu:

```json
{
  "run_id": "nearest_001",
  "method": "nearest",
  "git_commit": "...",
  "git_dirty_at_recorder_start": false,
  "protocol_version": "hospital_v1",
  "fixed_canvas_id": "hospital_canvas_v1",
  "evaluation_roi_id": "hospital_connected_free_v1",
  "exploration_start_sim_s": 0.0,
  "exploration_start_source": "first_policy_decision_before_compute",
  "execution_goal_semantics": "exact_frontier_center",
  "planner_path_semantics": "reachability_evidence_only",
  "sim_seed": null,
  "sim_seed_policy": "intentionally_uncontrolled_gazebo_default_multiple_run_statistics",
  "config_sha256": {},
  "termination_reason": null
}
```

`config_sha256` phải gồm cả source research code và installed runtime files thực tế (`frontier_exploration` package share), đặc biệt world/SLAM/Nav2/frontier config/launch.

Official runs yêu cầu clean git worktree.

## Periodic/final maps and offline correctness metrics

Periodic raw + fixed-canvas map mặc định mỗi `10 s` và final map được giữ để tính offline:

- occupied IoU/AUC;
- TU;
- các correctness curves khác nếu evaluator được chốt sau pilot.

Việc evaluator chưa tồn tại lúc chạy không được làm mất raw map cần thiết.

## Full MapEx decision directory sau này

MapEx dùng lại toàn bộ exact observed-map/candidate schema trên và thêm:

```text
decisions/policy_decision_000042/
├── decision.json
├── observed_map_raw.npz
├── observed_map_canvas.npz
├── g1.png
├── g2.png
├── g3.png
├── mean.npy
├── variance.npy
├── candidates.csv
└── visibility/
```

Candidate table của MapEx thêm:

```text
predicted_visible_cells,ig,score,rank
```

Nếu có structural GT:

```text
gt_visible_cells,gt_gain,gt_gain_per_m,gt_rank,prediction_error_summary
```

## Validation invariants

Post-run validator phải kiểm tra ít nhất:

- benchmark clock source đúng;
- metrics time và cumulative distance không giảm;
- coverage/known_fraction trong `[0,1]`;
- exact decision NPZ tồn tại/readable;
- snapshot CSV references tồn tại/readable;
- terminated goal không còn `PENDING`;
- failed goal có `failure_reason`;
- selected candidate join được bằng ID;
- không có `rejected_lt_1m`;
- `goal_source=exact_frontier_center` và execution goal trùng exact frontier center;
- planner endpoint được log riêng và offset audit tự nhất quán;
- terminated official run có explicit terminal policy state;
- với `no_nav2_reachable_ranked_candidate`: `candidate_count == nav2_checked_count`, planner-outcome counts cộng đúng, `nav2_path_success_count == 0`, và `terminal_reason` đúng;
- provenance hashes đầy đủ;
- official run bắt đầu với clean git.

## Lightweight GitHub results

Raw `.npz`/logs giữ local hoặc external storage. Chỉ commit summary/figure/table nhỏ dưới `results/`.
