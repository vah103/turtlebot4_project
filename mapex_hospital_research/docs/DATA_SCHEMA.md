# Data Schema

## Run layout

Mỗi run phải theo cấu trúc:

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

`method` ban đầu là `nearest` hoặc `mapex`.

## Canonical grids / masks

Trước benchmark phải chốt và giữ nguyên giữa mọi method/run:

### Fixed logging canvas `C_log`

Một grid cố định có cùng resolution, width/height, origin và frame/alignment. Nó dùng để tính `known_fraction`, coverage-aligned snapshot và các metric offline nhất quán.

### Canonical evaluation ROI `R_eval`

Một mask cố định trên Hospital chỉ chứa các cell được tính vào metric exploration chính. ROI và denominator không được đổi giữa Nearest và MapEx. Nếu ROI thay đổi thì protocol version phải đổi và baseline liên quan phải chạy lại.

## Hai loại map snapshot khác nhau

### Exact policy decision map

Đây là map **thực sự đã được freeze và dùng để tạo/rank candidate**. Nó phải do policy process lưu, không được lấy một `latest_map` muộn hơn rồi gọi là decision snapshot.

Mỗi policy decision lưu cả:

```text
observed_map_raw.npz
observed_map_canvas.npz
```

`observed_map_raw.npz` tối thiểu chứa:

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

Raw map dùng để replay exact frontier generation/ranking. Canvas map dùng cho alignment/evaluation.

### Periodic/final recorder snapshot

Recorder lưu raw + fixed-canvas map định kỳ (mặc định 10 s) và final map. Các snapshot này dùng để tính metric offline như occupied IoU/TU/AUC và không được coi là exact decision state nếu timestamp khác decision map.

## Metric definitions

Ký hiệu `K_t` là tập cell đã biết tại thời điểm `t` sau khi map được đưa về grid chuẩn.

### `known_fraction`

```text
known_fraction(t) = |K_t ∩ C_log| / |C_log|
```

Denominator luôn là fixed logging canvas, không crop động và không normalize final state của từng run thành 100%.

### `coverage`

```text
coverage(t) = |K_t ∩ R_eval| / |R_eval|
```

`R_eval` giống hệt giữa mọi run/phương pháp. Coverage chỉ đo phần không gian đã được quan sát/biết, không nói occupancy classification đúng hay sai.

### `occupied_iou`

Khi triển khai, occupied IoU phải so occupied cells với GT trên cùng evaluation mask/alignment. Threshold/công thức phải được ghi trong protocol trước khi báo cáo. Periodic/final raw maps được lưu để có thể tính metric này offline mà không rerun robot.

### `tu`

Topological Understanding được tính offline sau khi định nghĩa/evaluator được chốt. Periodic/final maps phải đủ để tính lại mà không cần chạy lại exploration.

## Exploration-stage analysis

Primary stage axis dùng absolute `coverage` trên `R_eval` hoặc absolute `known_fraction` trên cùng `C_log`; không kéo giãn final progress riêng của từng run thành 100%.

Tại cùng stage nên so:
- time-to-stage;
- distance-to-stage;
- goals attempted/succeeded/failed;
- success rate;
- computation cost;
- với MapEx: prediction, uncertainty, visibility, IG và ranking metrics.

Hiệu quả tổng thể tính riêng từ Coverage-vs-time, Coverage-vs-distance, AUC và final coverage dưới fixed common budget.

## metadata.json

Field tối thiểu:

```json
{
  "run_id": "nearest_001",
  "method": "nearest",
  "start_time": "ISO-8601",
  "git_commit": "...",
  "git_dirty_at_recorder_start": false,
  "protocol_version": "hospital_v1",
  "fixed_canvas_id": "hospital_canvas_v1",
  "evaluation_roi_id": "hospital_connected_free_v1",
  "world": "hospital_flat",
  "spawn": {"x": 0.0, "y": 12.0, "yaw": -1.57},
  "config_sha256": {},
  "termination_reason": null,
  "notes": ""
}
```

`config_sha256` phải chứa hash của code/config quan trọng để phát hiện run dùng code local khác nhau.

## metrics.csv

```text
time_s,distance_m,known_fraction,coverage,occupied_iou,tu
```

Có thể để trống metric chưa triển khai ở runtime nếu raw snapshots đủ để tính offline.

## trajectory.csv

```text
time_s,x,y,yaw,cumulative_distance_m
```

Đây là odometry trajectory. Không dùng odom pose thay cho map-frame pose trong exact policy replay.

## decisions.csv

Nearest runtime hiện lưu:

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
goal_x,goal_y,
robot_map_x,robot_map_y,robot_map_yaw,
result,
navigation_detail,
failure_reason
```

`policy_decision_id` là khóa nối sang exact policy state dưới `decisions/policy_decision_*`.

## policy_decisions.csv

Mỗi policy decision tối thiểu lưu:

```text
policy_decision_id,
sim_time_s,
map_stamp_s,
robot_x,robot_y,robot_yaw,
candidate_compute_ms,
candidate_count,
valid_ge_1m_count,
selected_rank,
selected_x,selected_y,
selected_distance_m,
selected_path_length_m,
outcome
```

Map-frame `(robot_x, robot_y)` phải là pose dùng khi tính ranking.

## candidates.csv

Mỗi candidate của mỗi policy decision lưu:

```text
policy_decision_id,
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

Các `status` điển hình:

```text
ranked
execution_suppressed
rejected_lt_1m
checking_nav2
nav2_rejected
nav2_request_error
nav2_result_error
nav2_no_path
selected
```

Rule 1 m phải được diễn giải đúng thứ tự MapEx:

```text
rank toàn bộ frontier
→ xét candidate thấp cost nhất
→ nếu distance < 1 m: reject candidate đó
→ thử rank tiếp theo
→ Nav2 path validation
```

Không áp dụng 1 m như frontier-detection filter trước ranking.

## snapshots.csv

Liên kết các periodic/final map với:

```text
snapshot_id,event,time_s,coverage,known_fraction,distance_m,
robot_map_x,robot_map_y,robot_map_yaw,
raw_map_file,canvas_map_file,source_map_stamp_s,canvas_id
```

## MapEx decision directory sau này

MapEx full giữ cùng exact observed-map/candidate schema ở trên và thêm:

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

Nếu có structural GT, thêm:

```text
gt_visible_cells,gt_gain,gt_gain_per_m,gt_rank,prediction_error_summary
```

## Lightweight summary committed to GitHub

Sau mỗi run, chỉ commit summary/figure nhỏ dưới `results/`. Raw `.npz`, prediction arrays, large logs và hàng nghìn frame giữ local hoặc external storage.
