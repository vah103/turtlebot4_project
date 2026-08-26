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
- Map frame: `map`; mọi evaluation snapshot được đưa về fixed SLAM-start canvas.

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
- Valid-cell rule: trong fixed Hospital bounds `x=[-0.572445, 24.588833]`, `y=[-35.091079,21.044604]` ở SLAM-start frame, lấy các free cells thuộc **8-connected component chứa robot start `(0,0)`** sau khi structural obstacles được rasterize.
- Excluded cells: obstacle cells, disconnected free-space pockets, vùng ngoài Hospital bounds và toàn bộ safety padding của fixed canvas.
- Total denominator cells: `215435`.
- Frozen mask SHA-256: `05d45b7aba66cb6dbb71e0406005f4a3e21875901af3ae72164b17f1d3add8d1`.

```text
coverage(t) = known cells inside hospital_connected_free_v1 / 215435
```

ROI phải giống hệt giữa Nearest và MapEx. Thay đổi ROI yêu cầu ROI ID mới và chạy lại baseline.

## Navigation

- Nav2 config: `hospital_nav2.launch.py` + `nav2_hospital_override.yaml`
- Maximum speed: `0.75 m/s`
- Per-goal hard timeout: `180 s`
- Stall timeout: `30 s` nếu không có meaningful progress
- Goal acceptance: dùng cùng Nav2 goal-checker configuration của Hospital stack cho mọi method.

## Sensor

- LiDAR topic: `/scan`
- Max range used by SLAM: `20.0 m`
- Sensor model/update rate: giữ nguyên TurtleBot4 RPLidar simulation description.

## Frontier policy shared by Nearest and MapEx

Nearest và MapEx Hospital dùng cùng frontier-generation semantics từ `castacks/MapEx` commit `53636bd1c79153acc3c74a532837d78c926bae5e`:

- Observed free: ROS occupancy value `0`.
- Unknown: ROS occupancy value `< 0`.
- Frontier cell: free cell có ít nhất một unknown cell trong **8-neighbourhood**.
- Frontier-region connectivity: **8-connected**.
- Region-size threshold: giữ region khi `size > 10`.
- Frontier representative: arithmetic mean của `(row,col)`, sau đó lấy frontier cell gần mean nhất.
- Không dùng WFD reachable-BFS, segment split, standoff goal hoặc nearest-safe-cell substitution.

### Hospital adaptation of MapEx `cur_pose_dist_threshold_m = 1.0`

MapEx gốc có rule hậu-ranking: candidate đã chọn nếu cách robot `<1.0 m` thì bị loại và reselect candidate tiếp theo.

`nearest_pilot_005` trên Hospital xác nhận rule này gây startup deadlock: có `534` frontier cells, `1` large region, `1` representative duy nhất ở `0.469 m`; candidate duy nhất bị reject trước khi Nav2 được hỏi path. Vì đây là incompatibility giữa simulator policy và Hospital/SLAM frontier geometry, `hospital_v1` **không enforce rule 1 m**.

Hospital control flow chính thức từ pilot_006 trở đi:

```text
1. detect toàn bộ MapEx frontier regions
2. lấy representative của từng region >10
3. score/rank toàn bộ candidate theo method hiện tại
4. lấy candidate cost thấp nhất
5. KHÔNG reject chỉ vì distance <1.0 m
6. gửi candidate sang Nav2 ComputePathToPose
7. nếu no path/rejected/error -> thử candidate có rank tiếp theo
8. candidate có path -> NavigateToPose và lock tới goal outcome
```

Distance vẫn được log và mỗi candidate có cờ `below_1m` để audit/offline analysis.

**Fairness rule:** adaptation này phải dùng giống hệt cho Nearest, MapEx original Hospital và proposed method. Không được bật lại 1 m cho một method riêng lẻ.

### Nearest scoring

```text
cost(frontier_i) = EuclideanDistance(current_pose, frontier_center_i)
selected = argmin(cost)
```

Nếu Nav2 không tìm được path tới candidate hiện tại thì bỏ candidate đó và thử candidate có cost thấp tiếp theo.

### ROS/TurtleBot4 execution adapter

MapEx gốc dùng `pyastar2d.astar_path(..., allow_diagonal=False)`. Hospital dùng:

```text
MapEx frontier generation + scoring
        ↓
Hospital adaptation: allow candidate <1 m
        ↓
Nav2 ComputePathToPose
        ↓
Nav2 NavigateToPose
```

Goal vẫn là chính MapEx frontier-center cell; adapter không shift goal hoặc tạo standoff.

Runtime:
- MapEx reference adapter: `scripts/mapex_nearest_ros.py`
- Hospital audit/visualization: `scripts/mapex_nearest_ros_hospital.py`
- exact research logging: `scripts/mapex_nearest_ros_research.py`
- Hospital 1 m adaptation used by launch: `scripts/mapex_nearest_ros_hospital_adapted.py`
- navigation detail wrapper: `scripts/exploration_manager_research.py`
- recorder: `scripts/research_recorder_safe.py`

File `frontier_detector*.py` cũ không được dùng trong Nearest research launch.

## Required replay logging before official runs

Mọi official Nearest/MapEx run phải lưu đủ dữ liệu để analysis/oracle/offline metric không cần rerun robot chỉ vì thiếu log.

### Exact policy decision state

Tại mỗi decision lưu chính OccupancyGrid đã freeze để tạo/rank candidate:
- raw OccupancyGrid values;
- width/height/resolution/origin/frame/map timestamp;
- exact fixed-canvas reprojection;
- map-frame robot pose dùng cho ranking;
- candidate-generation/ranking computation time.

### Candidate-level logging

Mỗi decision lưu toàn bộ candidate set:
- row/col và x/y;
- Euclidean distance;
- raw/policy rank;
- `below_1m` flag (diagnostic only; không reject trong Hospital);
- execution-suppression state;
- Nav2 path validation result;
- planner check time;
- selected flag/path length;
- execution success/failure.

### Navigation result detail

Ngoài `SUCCEEDED/FAILED`, giữ detail khi có thể: rejected, timeout, stall/no-progress, Nav2 status/result error.

### Periodic/final map retention

Recorder lưu raw + fixed-canvas snapshot định kỳ mặc định `10 s` và final map để có thể tính occupied IoU/TU và AUC offline.

### Provenance

`metadata.json` phải giữ git commit, git dirty state, protocol/canvas/ROI IDs, SHA-256 của code/config chính, world/spawn và termination reason.

## Exploration termination

Primary termination:
- không còn planner-reachable frontier theo Hospital policy ở trên;
- stable `5` checks;
- check period `2 s`;
- idle ít nhất `10 s`;
- startup grace `20 s`.

Nếu trước khi mission thật sự start không tồn tại candidate Nav2-reachable thì ghi **policy failure**, không gán nhầm successful completion.

## Resource budgets for secondary analysis

`hospital_v1` không dùng fixed time/distance budget làm controller termination. Sau pilot có thể chọn common offline analysis window/budget từ raw logs.

```text
time_progress = time_s / fixed_time_budget_s
distance_progress = distance_m / fixed_distance_budget_m
```

Budget báo cáo phải giống nhau giữa mọi phương pháp.

## Repetition

- Nearest target runs: `10`
- MapEx target runs: `10`
- Minimum acceptable runs before preliminary analysis: `5` per method
- `nearest_pilot_005`: invalid for benchmark; demonstrated 1 m startup deadlock.
- Trước official runs: chạy `nearest_pilot_006` để xác nhận motion + logging + termination với Hospital adaptation.
- Pilot/debug không được tính vào official benchmark.

## Metrics required per run

- absolute `known_fraction` vs time/distance
- `coverage` vs time
- `coverage` vs distance
- total distance/time
- number of frontier goals
- successful/failed goals + success rate
- termination reason
- computation timing
- raw/periodic maps đủ để tính occupied IoU/TU offline

MapEx additionally logs all prediction/variance/visibility/IG data defined trong `docs/DATA_SCHEMA.md`.

## Exploration-stage comparison rule

- Không kéo giãn final state của từng run thành 100% progress.
- Primary stage axis dùng absolute `coverage` trên cùng `R_eval`.
- Stage threshold chốt một lần sau pilot và giữ nguyên giữa methods.
- Run kết thúc trước một stage không được normalize để tạo sample giả.
- Tại cùng stage so time-to-stage, distance-to-stage, goal outcome, computation và diagnostic metrics; coverage chỉ là biến alignment.
- Hiệu quả tổng thể báo bằng Coverage-vs-time, Coverage-vs-distance, Coverage AUC và final coverage dưới cùng fixed budget.

## Fair-comparison rule

Không được thay spawn, Nav2, SLAM, sensor, timeout, stopping condition, fixed canvas, evaluation ROI, frontier-generation semantics, Hospital below-1m adaptation hoặc resource budget giữa Nearest và MapEx mà không ghi rõ lý do và chạy lại baseline tương ứng.
