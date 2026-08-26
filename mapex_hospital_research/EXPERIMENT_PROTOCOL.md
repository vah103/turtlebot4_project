# Experiment Protocol

Mọi phương pháp so sánh phải dùng cùng protocol, trừ khi thay đổi đó chính là biến thí nghiệm và được ghi rõ.

## Protocol identity

- Protocol version: `hospital_v1`
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
- Total denominator cells: `215435`.
- Frozen mask SHA-256: `05d45b7aba66cb6dbb71e0406005f4a3e21875901af3ae72164b17f1d3add8d1`.

`coverage` được tính:

```text
coverage(t) = known cells inside hospital_connected_free_v1 / 215435
```

ROI phải giống hệt giữa Nearest và MapEx. Thay đổi ROI yêu cầu ROI ID mới và chạy lại baseline.

## Navigation

- Nav2 config: `hospital_nav2.launch.py` + `nav2_hospital_override.yaml`
- Maximum speed: `0.75 m/s`
- Per-goal hard timeout: `180 s`
- Stall timeout: `30 s` nếu không có meaningful progress
- Recovery behavior: giữ nguyên Nav2 stack hiện tại; execution failure chỉ suppress frontier tạm thời, không tự coi là permanently unreachable.
- Goal acceptance: dùng cùng Nav2 goal-checker configuration của Hospital stack cho mọi method; research code không override riêng giữa Nearest và MapEx.

## Sensor

- LiDAR topic: `/scan`
- Max range used by SLAM: `20.0 m`
- Sensor model/update rate: giữ nguyên TurtleBot4 RPLidar simulation description; research methods không override sensor parameters.

## Exploration

- Frontier detector: WFD-style reachable-space BFS trên raw SLAM `/map`, sau đó 8-connected frontier clustering.
- Minimum frontier cluster size: `5 cells`
- Minimum split segment size: `5 cells`
- Minimum selection distance: `0.60 m`
- Nearest rule: candidate frontier được sắp theo Euclidean distance từ robot tới frontier representative; goal phải qua costmap safety + Nav2 `ComputePathToPose` reachability check trước khi được chọn.
- Stopping condition: `/exploration_complete` khi không còn reachable frontier, trạng thái này ổn định `5` cycles, check mỗi `2 s`, và idle ít nhất `10 s` (startup grace `20 s`).
- Random seed policy: Nearest không có random frontier selection; repeated runs vẫn được dùng để đo simulator/SLAM/navigation variability.

### Resource budgets for secondary analysis

`hospital_v1` không dùng fixed time/distance budget làm controller termination. Primary run termination là exploration completion ở trên. Sau pilot có thể chọn một common offline analysis window/budget từ raw logs để báo thêm final coverage hoặc normalized resource progress; việc chọn cutoff phân tích không được thay đổi controller behavior hoặc ROI.

Nếu dùng normalized resource progress:

```text
time_progress = time_s / fixed_time_budget_s
distance_progress = distance_m / fixed_distance_budget_m
```

Khi đã chọn budget để báo cáo, budget đó phải giống nhau giữa mọi phương pháp.

## Repetition

- Nearest target runs: 10
- MapEx target runs: 10
- Minimum acceptable runs before preliminary analysis: 5 per method
- Trước official runs: chạy `nearest_pilot_001` để xác nhận logger, alignment và termination.

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
- Hiệu quả exploration tổng thể phải báo bằng Coverage-vs-time, Coverage-vs-distance, Coverage AUC và final coverage dưới cùng fixed budget nếu budget được sử dụng.
- Có thể báo thêm time-progress hoặc distance-progress theo fixed common budget.

## Fair-comparison rule

Không được thay spawn, Nav2, SLAM, sensor, timeout, stopping condition, fixed canvas hoặc evaluation ROI giữa Nearest và MapEx mà không ghi rõ lý do và chạy lại baseline tương ứng. Nếu một resource budget được dùng như termination condition trong tương lai, việc thay budget cũng yêu cầu protocol mới và baseline rerun.
