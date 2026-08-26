# Data Schema

## Run layout

Mỗi run phải theo cấu trúc:

```text
experiments/<method>/<run_id>/
├── metadata.json
├── metrics.csv
├── trajectory.csv
├── decisions.csv
├── maps/
├── predictions/
├── variance/
├── visibility/
├── decisions/
└── logs/
```

`method` ban đầu là `nearest` hoặc `mapex`.

## Canonical grids / masks

Trước khi chạy benchmark phải chốt hai đối tượng và giữ nguyên giữa mọi method/run:

### Fixed logging canvas `C_log`

Một grid cố định có cùng:
- resolution;
- width/height;
- origin;
- frame/alignment.

Nó dùng để lưu snapshot và tính `known_fraction` nhất quán. Không được dùng bounding box động của từng snapshot làm denominator.

### Canonical evaluation ROI `R_eval`

Một mask cố định trên Hospital, chỉ chứa các cell được tính vào metric exploration chính. ROI phải loại padding/outside-of-benchmark/invalid cells theo một quy tắc được chốt ở Stage 1.

ROI và denominator không được thay đổi giữa Nearest và MapEx. Nếu định nghĩa ROI thay đổi thì protocol version phải đổi và baseline liên quan phải chạy lại.

## Metric definitions

Ký hiệu `K_t` là tập cell mà occupancy map tại thời điểm `t` đã biết, tức cell không còn giá trị `unknown` sau khi map được đưa về grid chuẩn.

### `known_fraction`

Progress/debugging proxy trên fixed logging canvas:

```text
known_fraction(t) = |K_t ∩ C_log| / |C_log|
```

Quy tắc:
- denominator luôn là toàn bộ fixed logging canvas đã chốt;
- không crop theo map hiện tại;
- không chuẩn hóa final value của từng run thành 1.0;
- chỉ so trực tiếp giữa các run khi canvas/resolution/origin giống nhau.

`known_fraction` hữu ích cho snapshot indexing/debugging, nhưng không phải metric correctness của bản đồ.

### `coverage`

Metric exploration chính trên canonical evaluation ROI:

```text
coverage(t) = |K_t ∩ R_eval| / |R_eval|
```

Quy tắc:
- `R_eval` giống hệt giữa mọi run/phương pháp;
- denominator cố định;
- một cell được tính là covered khi nó đã được quan sát/biết trong current SLAM map;
- coverage không nói cell đó được phân loại đúng hay sai; correctness được đánh giá riêng bằng IoU/precision/recall.

### `occupied_iou`

Nếu triển khai, occupied IoU phải so occupied cells của current map với GT trên cùng evaluation mask/alignment. Công thức/threshold chính xác phải được ghi trong protocol trước khi dùng để báo cáo.

## Exploration-stage analysis

Stage chính không được lấy final progress riêng của từng run rồi kéo giãn thành 100%.

Primary stage axis:
- dùng absolute `coverage` trên `R_eval`;
- hoặc absolute `known_fraction` nếu cần, với cùng `C_log` cho mọi run.

Nếu một run kết thúc trước một stage thì run đó không đóng góp sample cho stage chưa đạt.

### Same-stage comparison semantics

Khi hai run/method được căn chỉnh tại cùng một coverage stage, `coverage` chỉ là biến alignment.

Tại mỗi stage nên tổng hợp các đại lượng như:
- `time_to_stage_s`;
- `distance_to_stage_m`;
- goals attempted/succeeded/failed đến stage;
- success rate đến stage;
- computation cost nếu có;
- với MapEx: prediction, uncertainty, visibility, IG và ranking metrics tại stage.

Không tạo kết luận kiểu "method A có coverage cao hơn method B tại cùng coverage stage".

Các metric hiệu quả exploration tổng thể được tính riêng từ curve/raw run data:
- Coverage vs time;
- Coverage vs distance;
- Coverage AUC theo time/distance;
- final coverage dưới fixed time/distance budget.

Có thể phân tích phụ theo fixed resource budget:

```text
time_progress = time_s / fixed_time_budget_s
distance_progress = cumulative_distance_m / fixed_distance_budget_m
```

Budget phải giống nhau giữa các phương pháp. Các giá trị này có thể tính offline từ raw logs, không bắt buộc lưu thành cột riêng.

## metadata.json

Các field tối thiểu:

```json
{
  "run_id": "nearest_001",
  "method": "nearest",
  "start_time": "ISO-8601",
  "git_commit": "optional",
  "protocol_version": "TODO",
  "fixed_canvas_id": "TODO",
  "evaluation_roi_id": "TODO",
  "world": "hospital",
  "spawn": {"x": null, "y": null, "yaw": null},
  "termination_reason": null,
  "notes": ""
}
```

## metrics.csv

Mỗi dòng là một timestamp/sample:

```text
time_s,distance_m,known_fraction,coverage,occupied_iou,tu
```

Có thể để trống metric chưa triển khai nhưng không đổi tên cột giữa các run.

## trajectory.csv

```text
time_s,x,y,yaw,cumulative_distance_m
```

## decisions.csv

Nearest tối thiểu:

```text
decision_id,time_s,known_fraction,num_candidates,selected_candidate_id,selected_distance_m,result
```

MapEx thêm:

```text
selected_ig,selected_score,selected_rank
```

## MapEx decision directory

```text
decisions/decision_000042/
├── decision.json
├── observed_map.png
├── g1.png
├── g2.png
├── g3.png
├── mean.npy
├── variance.npy
├── frontiers.csv
└── visibility/
```

`frontiers.csv` nên có:

```text
candidate_id,x,y,distance_m,predicted_visible_cells,ig,score,rank,selected
```

Nếu có structural GT, thêm:

```text
gt_visible_cells,gt_gain,gt_gain_per_m,gt_rank,prediction_error_summary
```

## Lightweight summary committed to GitHub

Sau mỗi run, cập nhật các file nhỏ dưới `results/summary/`. Dữ liệu nặng chỉ lưu local hoặc storage ngoài GitHub.
