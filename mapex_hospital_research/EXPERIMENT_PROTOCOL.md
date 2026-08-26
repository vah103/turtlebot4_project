# Experiment Protocol

Mọi phương pháp so sánh phải dùng cùng protocol, trừ khi thay đổi đó chính là biến thí nghiệm và được ghi rõ.

## Protocol identity

- Protocol version: TODO
- Fixed canvas ID: TODO
- Evaluation ROI ID: TODO

## Environment

- World: `Hospital`
- Simulator: TODO
- Robot: TurtleBot4
- Spawn x: TODO
- Spawn y: TODO
- Spawn yaw: TODO

## Mapping / SLAM

- SLAM package/config: TODO
- Map resolution: TODO
- Map frame: TODO

### Fixed logging canvas

- Resolution: TODO
- Width: TODO
- Height: TODO
- Origin: TODO
- Alignment rule: TODO

`known_fraction` được tính trên canvas cố định này. Không crop theo bounding box động và không chuẩn hóa final known fraction của từng run thành 100%.

### Canonical evaluation ROI

- ROI source/mask: TODO
- ROI alignment: TODO
- Valid-cell rule: TODO
- Excluded cells: TODO
- Total denominator cells: TODO

`coverage` được tính trên ROI cố định này theo định nghĩa trong `docs/DATA_SCHEMA.md`. ROI phải giống hệt giữa Nearest và MapEx.

## Navigation

- Nav2 config: TODO
- Goal timeout: TODO
- Recovery behavior: TODO
- Goal acceptance radius: TODO

## Sensor

- LiDAR topic: TODO
- Max range: TODO
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
- Primary comparison dùng absolute `coverage` trên cùng `R_eval`; có thể dùng absolute `known_fraction` nếu fixed canvas giống hệt.
- Stage threshold phải được chốt một lần sau pilot và không đổi giữa method.
- Run kết thúc trước một stage không được giả lập/normalize để có sample ở stage đó.
- Có thể báo thêm time-progress hoặc distance-progress theo fixed common budget.

## Fair-comparison rule

Không được thay spawn, Nav2, SLAM, sensor, timeout, stopping condition, fixed canvas, evaluation ROI hoặc resource budget giữa Nearest và MapEx mà không ghi rõ lý do và chạy lại baseline tương ứng.
