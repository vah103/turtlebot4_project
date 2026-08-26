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

## metadata.json

Các field tối thiểu:

```json
{
  "run_id": "nearest_001",
  "method": "nearest",
  "start_time": "ISO-8601",
  "git_commit": "optional",
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
