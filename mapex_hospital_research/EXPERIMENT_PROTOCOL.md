# Experiment Protocol

Mọi phương pháp so sánh phải dùng cùng protocol, trừ khi thay đổi đó chính là biến thí nghiệm và được ghi rõ.

## Protocol identity

- Protocol version: TODO — chỉ gán `hospital_v1` sau khi ROI mask được generate và denominator được freeze.
- Fixed canvas ID: `hospital_canvas_v1`
- Evaluation ROI ID: `hospital_connected_free_v1`

## Environment

- World: `Hospital flat` (`hospital_aws_flat.sdf`)
- Simulator: Gazebo / TurtleBot4 simulation stack hiện có trong `frontier_exploration`
- Robot: TurtleBot4
- Spawn x: `0.0 m`
- Spawn y: `12.0 m`
- Spawn yaw: `-1.57 rad`

## Mapping / SLAM

- SLAM package/config: `slam_toolbox` với `ros2_ws/src/frontier_exploration/config/hospital_slam.yaml`
- Map resolution: `0.05 m/cell`
- Map frame: `map`; mọi snapshot được đưa về fixed SLAM-start canvas để logging/evaluation.

### Fixed logging canvas

- ID: `hospital_canvas_v1`
- Resolution: `0.05 m/cell`
- Width: `1504 cells`
- Height: `2123 cells`
- Origin: `(-25.6, -60.1) m`
- Alignment rule: reproject mọi SLAM snapshot lên đúng grid này; không crop theo bounding box động.

`known_fraction` được tính trên canvas cố định này. Không crop theo bounding box động và không chuẩn hóa final known fraction của từng run thành 100%.

### Canonical evaluation ROI

- ROI ID: `hospital_connected_free_v1`
- ROI specification: `ground_truth/hospital/roi_v1.yaml`
- ROI alignment: cùng resolution `0.05 m`, origin, kích thước và SLAM-start frame của `hospital_canvas_v1`.
- Structural source: Hospital wall collision mesh + flat-world elevator blockers; wall slice `z=0.30 m`, wall raster thickness `2 cells`, sau đó dilate `1 cell` để đóng raster cracks.
- Valid-cell rule: trong fixed Hospital bounds `x=[-0.572445, 24.588833]`, `y=[-35.091079, 21.044604]` ở SLAM-start frame, lấy các free cells thuộc **8-connected component chứa robot start `(0,0)`** sau khi structural obstacles được rasterize.
- Excluded cells: obstacle cells, disconnected free-space pockets, vùng ngoài Hospital bounds và toàn bộ safety padding của fixed canvas.
- Total denominator cells: TODO — generate mask đúng một lần trước `nearest_001`, ghi số cell vào `roi_v1.yaml` và file này, sau đó freeze.

`coverage` được tính:

```text
coverage(t) = known cells inside hospital_connected_free_v1 / total cells in hospital_connected_free_v1
```

ROI phải giống hệt giữa Nearest và MapEx. Thay đổi ROI yêu cầu ROI ID mới và chạy lại baseline.

## Navigation

- Nav2 config: `hospital_nav2.launch.py` + `nav2_hospital_override.yaml`
- Goal timeout: TODO
- Recovery behavior: TODO
- Goal acceptance radius: TODO

## Sensor

- LiDAR topic: `/scan`
- Max range: `20.0 m`
- Sensor update rate: TODO

## Exploration

- Frontier detector: TODO
- Stopping condition: TODO
- No-frontier timeout: TODO
- Minimum frontier size: TODO
- Random seed policy: TODO

### Fixed resource budgets for secondary stage analysis

- Time budget: TODO
- Distance budget: TODO

Nếu dùng normalized resource progress:

```text
time_progress = time_s / fixed_time_budget_s
distance_progress = distance_m / fixed_distance_budget_m
```

Budget phải giống nhau giữa các phương pháp. Đây là trục phân tích phụ; stage chính vẫn dựa trên absolute exploration state (`coverage` hoặc `known_fraction` với denominator cố định).

## Repetition

- Nearest target runs: 10
- MapEx target runs: 10
- Minimum acceptable runs before preliminary analysis: 5 per method

## Metrics required per run

- absolute `known_fraction` vs time/distance
- `coverage` vs time
- `coverage` vs distance
- total distance
- total exploration time
- number of frontier goals
- successful goals
- failed goals
- success rate
- termination reason

MapEx additionally logs all decision-level data defined in `docs/DATA_SCHEMA.md`.

## Exploration-stage comparison rule

- Không kéo giãn final state của từng run thành 100% progress.
- Primary stage axis dùng absolute `coverage` trên cùng `R_eval`; có thể dùng absolute `known_fraction` nếu fixed canvas giống hệt.
- Stage threshold phải được chốt một lần sau pilot và không đổi giữa method.
- Run kết thúc trước một stage không được giả lập/normalize để có sample ở stage đó.
- Tại cùng một coverage stage, coverage chỉ là biến căn chỉnh trạng thái; không so coverage với coverage tại chính mốc đó.
- Tại cùng stage, so các đại lượng như time-to-stage, distance-to-stage, goal failures/success, computation cost và các metric chẩn đoán pipeline tương ứng.
- Hiệu quả exploration tổng thể phải báo bằng Coverage-vs-time, Coverage-vs-distance, Coverage AUC và final coverage dưới cùng fixed budget.
- Có thể báo thêm time-progress hoặc distance-progress theo fixed common budget.

## Fair-comparison rule

Không được thay spawn, Nav2, SLAM, sensor, timeout, stopping condition, fixed canvas, evaluation ROI hoặc resource budget giữa Nearest và MapEx mà không ghi rõ lý do và chạy lại baseline tương ứng.
