# Data Schema

Schema này mô tả recorder hiện dùng chung cho Nearest Frontier và MapEx. Mục tiêu là đủ dữ liệu để audit/replay policy decision mà không làm thay đổi thuật toán exploration.

## Run layout

### Shared layout

```text
experiments/<method>/<run_id>/
├── metadata.json
├── runtime_nav2_merged.yaml
├── metrics.csv
├── trajectory.csv
├── policy_decisions.csv
├── candidates.csv
├── snapshots.csv
├── goals.csv
├── plans.csv
├── maps/
│   ├── snapshot_..._raw.npz
│   ├── snapshot_..._canvas.npz
│   └── ... final snapshot ...
└── decisions/
    └── policy_decision_000001/
        ├── decision.json
        ├── candidates.csv
        ├── observed_map_raw.npz
        └── observed_map_canvas.npz
```

### MapEx additions

```text
experiments/mapex/<run_id>/
├── runtime_mapex.yaml
├── decisions.csv              # MapEx-specific compatibility/detail table
├── decision_maps/             # legacy flat decision maps for evaluator compatibility
└── predictions/               # when --save-predictions is enabled
    ├── decision_000001_g1.npz
    ├── decision_000001_g2.npz
    ├── decision_000001_g3.npz
    ├── decision_000001_mean.npz
    └── decision_000001_variance.npz
```

## Canonical grids

Đối với `hospital_v2`:

- runtime SLAM/policy grid: `0.10 m/cell`;
- fixed evaluation canvas: `hospital_canvas_v1`, `0.05 m/cell`;
- canvas size: `1504 x 2123`;
- canvas origin: `(-25.6,-60.1)`;
- evaluation ROI: `hospital_connected_free_v1`;
- denominator: `215435`;
- ROI SHA-256: `05d45b7aba66cb6dbb71e0406005f4a3e21875901af3ae72164b17f1d3add8d1`.

Runtime resolution và evaluation resolution độc lập. Với map axis-aligned `0.10 m`, mỗi runtime cell được expand nearest-neighbour thành `2 x 2` cells trên canvas `0.05 m`.

`known_fraction` dùng full fixed canvas. `coverage` dùng frozen ROI. Không dynamic crop và không normalize riêng từng final map.

## Benchmark clock

```text
exploration_start_source = first_policy_decision_before_compute
```

`time_s=0` được recorder freeze ngay trước candidate computation của policy decision đầu tiên, sau khi map/TF/Nav2 đã ready. Không yêu cầu một ROS topic riêng làm source of truth.

Điều này làm cho:

- Nearest tính cả frontier generation + Euclidean ranking đầu tiên;
- MapEx tính cả LaMa prediction + visibility + IG/scoring đầu tiên.

## Exact policy decision map

Mỗi decision giữ đúng OccupancyGrid dùng để tạo/rank frontier:

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

Canvas NPZ chứa canonical evaluation reprojection và `canvas_id`. Canvas chỉ dùng cho alignment/evaluation; policy vẫn dùng raw runtime grid.

## Shared 1 m semantics

Nearest và MapEx dùng chung:

```text
if any raw representative distance >= 1.0 m:
    distance_eligible = candidates >= 1.0 m
    near_frontier_fallback = 0
else:
    distance_eligible = all current representatives
    near_frontier_fallback = 1
```

Sau bước này mới áp dụng execution/planner suppression.

Ý nghĩa field:

- `below_1m`: candidate có `distance_m < 1.0`;
- `distance_eligible`: candidate có được vào ordinary policy selection ở decision đó hay không;
- `near_frontier_fallback`: toàn bộ raw representatives của decision đều `<1 m`, nên nhóm gần được phép xét;
- `distance_deferred_lt_1m`: candidate gần bị tạm bỏ vì decision vẫn còn ít nhất một candidate `>=1 m`.

`below_1m` **không còn là diagnostic-only**. Nó tham gia eligibility, nhưng `all frontiers <1m` không phải completion condition.

## metrics.csv

Shared fields hiện tại:

```text
time_s,
distance_m,
known_fraction,
coverage,
occupied_iou,
tu,
frontiers_selected,
main_attempts,
main_succeeded,
main_failed,
main_interrupted,
abandoned_206,
abandoned_208,
planner_blocked_abandoned_total,
main_success_rate,
subgoal_attempts,
subgoal_succeeded,
subgoal_failed,
subgoal_interrupted
```

- `distance_m`: cumulative odometry distance;
- `occupied_iou` / `tu`: có thể là `nan` online và được tính/backfill offline khi evaluator/ground truth phù hợp;
- MapEx hiện chạy offline evaluation sau khi recorder đóng file.

## trajectory.csv

```text
time_s,x,y,yaw,cumulative_distance_m
```

Đây là odometry trajectory. Policy replay dùng map-frame robot pose trong `policy_decisions.csv`, không lấy pose ranking từ trajectory này.

## policy_decisions.csv

Shared columns:

```text
policy_decision_id,
sim_time_s,
map_stamp_s,
map_generation,
robot_x,
robot_y,
robot_yaw,
candidate_compute_ms,
frontier_region_count,
candidate_count,
distance_eligible_count,
selectable_count,
suppressed_count,
below_1m_count,
preferred_min_distance_m,
near_frontier_fallback,
selected_candidate_id,
selected_rank,
selected_x,
selected_y,
selected_distance_m,
planner_revalidation_started,
outcome,
raw_map,
canvas_map
```

`robot_x/y` là map-frame position dùng cho ranking. `robot_yaw` là audit pose; frontier policy hiện position-based.

Typical outcomes:

```text
selected
exhausted_no_frontier_region
no_normally_selectable_candidate
no_selection
```

`selected_candidate_id` phải join với candidate của cùng `policy_decision_id`.

### MapEx extension

MapEx thêm vào `policy_decisions.csv`:

```text
mapex_policy_decision_id,
ensemble_prediction_ms,
all_frontier_scoring_ms,
selected_information_gain,
selected_score,
selected_visible_unknown_cells
```

## candidates.csv

Shared candidate columns:

```text
policy_decision_id,
candidate_id,
raw_rank,
policy_rank,
row,
col,
x,
y,
distance_cells,
distance_m,
region_size,
below_1m,
distance_eligible,
near_frontier_fallback,
execution_suppressed,
planner_suppressed,
selectable,
status,
selected,
execution_result
```

Stable join key:

```text
policy_decision_id + candidate_id
```

For Nearest, `candidate_id` được tạo ổn định theo runtime grid representative (`r<row>_c<col>`). Với MapEx, `candidate_id` là ID của frontier evaluation trong decision và luôn đi cùng `policy_decision_id`.

Possible shared statuses:

```text
ranked
selected
distance_deferred_lt_1m
execution_suppressed
planner_suppressed
not_selectable
```

`execution_result` hiện là reserved candidate-level field; execution truth nằm ở `goals.csv`, join qua `decision_id/policy_decision_id`. Không suy execution success từ candidate row một mình.

### Nearest ranking

```text
raw_rank    = Euclidean distance order over raw representatives
policy_rank = Euclidean distance order over selectable representatives
```

Selected candidate phải là `policy_rank = 1`.

### MapEx candidate extension

MapEx giữ toàn bộ shared fields và thêm:

```text
decision_id,
mapex_policy_decision_id,
rank_all,
rank_selectable,
information_gain,
score,
visible_unknown_cells,
ensemble_prediction_ms,
all_frontier_scoring_ms,
suppressed_planner_blocked
```

Semantics:

```text
score = information_gain / distance_m
rank_all = score-descending rank over all evaluated frontiers
rank_selectable = score-descending rank after 1m eligibility + suppression
```

Selected MapEx candidate phải có score lớn nhất trong selectable set.

## goals.csv

```text
goal_id,
decision_id,
mode,
start_time_s,
end_time_s,
target_x,
target_y,
result,
status,
error_code,
error_msg
```

`mode` là `main` hoặc `subgoal` ở row bắt đầu. Completion row của cùng `goal_id` chứa result/status/error.

Execution invariant:

- main `target_x/y` là exact selected frontier coordinates;
- recovery subgoal là temporary Nav2 target, không phải policy frontier mới;
- retry exact main frontier không được tính như một policy decision mới.

## plans.csv

```text
time_s,
goal_id,
decision_id,
poses,
path_length_m,
endpoint_x,
endpoint_y,
frontier_x,
frontier_y,
endpoint_error_m,
usable
```

`/plan` là Nav2 navigation diagnostic và path-guided recovery evidence.

- `path_length_m` **không** phải executed trajectory distance;
- `usable=1` cho recovery khi path endpoint đủ gần exact main frontier theo shared tolerance;
- total executed distance luôn lấy từ odometry.

Ordinary policy selection hiện không bắt buộc `ComputePathToPose` pre-check cho từng candidate.

## Planner suppression / terminal revalidation

Shared execution layer phân biệt:

- `105 FAILED_TO_MAKE_PROGRESS`: execution failure, bounded path-guided recovery rồi temporary cooldown;
- `206 GOAL_OCCUPIED` và `208 NO_VALID_PATH`: planner-blocking failure, ordinary selection suppression;
- khi normally selectable set cạn vì planner-blocking state, `ComputePathToPose` được dùng để terminal-revalidate candidates;
- candidate reachable trở lại được bỏ planner suppression;
- completion cần repeated stable exhausted evidence, không dùng một cached no-path result duy nhất.

`planner_revalidation_started` trong `policy_decisions.csv` cho biết decision đã đi vào branch revalidation. Candidate CSV không được hiểu là log đầy đủ mọi asynchronous `ComputePathToPose` callback.

## snapshots.csv

```text
snapshot_id,
event,
time_s,
coverage,
known_fraction,
distance_m,
robot_map_x,
robot_map_y,
robot_map_yaw,
raw_map_file,
canvas_map_file,
source_map_stamp_s,
canvas_id
```

Recorder lưu periodic snapshot khoảng mỗi `10 s` và final snapshot. Snapshot lưu cả raw runtime map và fixed canvas.

## metadata.json

Shared minimum fields quan trọng:

```json
{
  "run_id": "nearest_001",
  "method": "nearest_frontier_euclidean",
  "git_commit": "...",
  "git_dirty_at_recorder_start": false,
  "protocol_version": "...",
  "runtime_profile_name": "submap",
  "runtime_map_resolution_m": 0.10,
  "fixed_canvas_id": "hospital_canvas_v1",
  "fixed_canvas_resolution_m": 0.05,
  "evaluation_roi_id": "hospital_connected_free_v1",
  "exploration_start_sim_s": 0.0,
  "exploration_start_source": "first_policy_decision_before_compute",
  "execution_goal_semantics": "exact_frontier_center_xy",
  "preferred_min_frontier_distance_m": 1.0,
  "near_frontier_fallback": "all_frontiers_below_threshold",
  "frontier_region_min_cells_strictly_greater_than": 10,
  "distance_metric": "euclidean",
  "runtime_nav2_merged_file": "runtime_nav2_merged.yaml",
  "sim_seed": null,
  "sim_seed_policy": "intentionally_uncontrolled_gazebo_default_multiple_run_statistics",
  "config_sha256": {},
  "termination_reason": null
}
```

`config_sha256` phải hash source/runtime files thực sự tồn tại và liên quan tới selected runtime profile. Wrapper runner đã xóa không được ghi vào provenance.

`RUNTIME_PROFILES` được quản lý tập trung trong `nf_run.py` và `mapex_run.py` import cùng source of truth.

## summary.json

Hai method đều phải có tối thiểu:

```text
final_coverage
final_known_fraction
total_distance_m
total_time_s
frontiers_selected
policy_decisions
near_frontier_fallback_count
near_frontier_fallback_fraction
main_attempts / succeeded / failed
subgoal_attempts / succeeded / failed
abandoned_206 / abandoned_208
main_success_rate
termination_reason
```

Nearest thêm `candidate_compute_ms_mean/std`.

MapEx thêm:

```text
decision_computation_ms_mean/std
ensemble_prediction_ms_mean/std
all_frontier_scoring_ms_mean/std
selected_information_gain_mean
selected_score_mean
selected_distance_m_mean
selection_verification_failures
```

## MapEx prediction retention

Khi `--save-predictions` bật, mỗi decision có thể lưu:

```text
G1
G2
G3
ensemble mean
ensemble variance
```

Candidate table luôn giữ `information_gain`, `score` và `visible_unknown_cells`. Visibility mask riêng có thể bổ sung khi cần debug chuyên sâu, nhưng không bắt buộc cho mọi official run nếu prediction + candidate diagnostics đã đủ cho mục tiêu phân tích.

## Validation invariants

Post-run validation nên kiểm tra:

- raw runtime resolution đúng protocol;
- fixed canvas ID/resolution không đổi;
- benchmark clock bắt đầu ở first policy computation;
- metrics time/distance monotonic;
- coverage trong bounds hợp lệ;
- decision/snapshot NPZ tồn tại/readable;
- selected candidate join được bằng decision + candidate ID;
- shared 1m rule đúng: nếu có candidate `>=1m`, candidate `<1m` không `distance_eligible`; nếu tất cả `<1m`, fallback phải bật;
- selected candidate phải `distance_eligible=1`, `selectable=1`;
- Nearest selected phải là nearest selectable;
- MapEx selected phải là max-score selectable;
- main goal target phải trùng exact selected frontier x/y;
- `206/208` không bị double-count;
- `frontiers_selected` không bị double-count do recorder hook;
- terminated run có final snapshot;
- official run dùng clean git worktree;
- guard phát hiện pathology `SUCCEEDED` nhưng odometry gần như không di chuyển dù frontier ở xa.

## Lightweight GitHub results

Raw `.npz` và log lớn có thể giữ local/external storage. Chỉ commit summary/figure/table cần thiết dưới `results/` khi muốn repo nhẹ.
