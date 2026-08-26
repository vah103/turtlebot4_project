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
- Goal acceptance: dùng cùng Nav2 goal-checker configuration của Hospital stack cho mọi method; research code không override riêng giữa Nearest và MapEx.

## Sensor

- LiDAR topic: `/scan`
- Max range used by SLAM: `20.0 m`
- Sensor model/update rate: giữ nguyên TurtleBot4 RPLidar simulation description; research methods không override sensor parameters.

## Frontier policy shared by Nearest and MapEx

Để so đúng với implementation được tác giả MapEx công bố, Nearest và MapEx Hospital phải dùng cùng frontier-generation semantics từ `castacks/MapEx` commit `53636bd1c79153acc3c74a532837d78c926bae5e`, chủ yếu từ `scripts/sim_utils.py` và control flow trong `scripts/explore.py`.

- Observed free: ROS occupancy value `0`.
- Unknown: ROS occupancy value `< 0`.
- Frontier cell: free cell có ít nhất một unknown cell trong **8-neighbourhood**.
- Frontier-region connectivity: **8-connected**.
- Region-size threshold: MapEx gốc dùng `region_size_threshold = 10` và giữ region khi `size > 10`.
- Frontier representative: tính arithmetic mean của `(row, col)` trong region, sau đó lấy frontier cell có Euclidean distance nhỏ nhất tới mean.
- Không dùng WFD reachable-BFS, segment split, standoff goal hoặc nearest-safe-cell substitution để tạo/chuyển frontier trong baseline MapEx-nearest này.

### Exact `cur_pose_dist_threshold_m = 1.0` semantics

Rule 1 m **không phải frontier-detection filter và không được áp dụng trước ranking**.

Control flow phải là:

```text
1. detect toàn bộ MapEx frontier regions
2. lấy representative của từng region >10
3. score/rank toàn bộ candidate theo method hiện tại
4. lấy candidate cost thấp nhất làm locked candidate
5. nếu locked candidate cách robot <1.0 m -> candidate invalid -> remove/reselect
6. thử candidate có rank tiếp theo
7. candidate hợp lệ mới đi qua local-planner/Nav2 path validation
8. nếu no path -> remove/reselect candidate tiếp theo
```

Điều này bám theo `is_locked_frontier_center_valid()` + `reselect_frontier_from_frontier_region_centers()` trong code MapEx gốc. Không được chuyển rule này thành `if distance < 1m: continue` ở bước tạo frontier trước ranking.

### Nearest scoring

Nearest dùng đúng score mode của MapEx:

```text
cost(frontier_i) = EuclideanDistance(current_pose, frontier_center_i)
selected = argmin(cost)
```

Frontier hợp lệ đã chọn được lock cho tới khi đạt goal/outcome. Nếu local planner không tìm được path tới frontier-center đó thì bỏ candidate hiện tại và thử candidate có cost thấp tiếp theo.

### ROS/TurtleBot4 execution adapter

MapEx gốc chạy grid simulator và dùng `pyastar2d.astar_path(..., allow_diagonal=False)`. Hospital benchmark không giả lập robot bằng pixel steps; vì vậy **chỉ execution layer được thay**:

```text
MapEx frontier generation + scoring
        ↓
post-ranking locked-frontier validity (<1 m -> reject/reselect)
        ↓
Nav2 ComputePathToPose  (thay pyastar2d A* reachability/local planning)
        ↓
Nav2 NavigateToPose     (TurtleBot4 execution)
```

Goal gửi sang Nav2 vẫn là chính MapEx frontier-center cell; adapter không tự dịch goal vào sâu trong free space. Nếu `ComputePathToPose` không có path, adapter thử frontier tiếp theo giống vòng reselect của MapEx. Navigation execution failure là hiện tượng riêng của robot/ROS mà simulator MapEx gốc không mô hình hóa; adapter tạm bỏ vùng goal vừa fail và reselect, đồng thời logger phải ghi failure này.

Implementation runtime hiện tại:
- base policy: `scripts/mapex_nearest_ros.py`
- Hospital audit/visualization: `scripts/mapex_nearest_ros_hospital.py`
- exact research logging: `scripts/mapex_nearest_ros_research.py`
- navigation detail wrapper: `scripts/exploration_manager_research.py`
- recorder: `scripts/research_recorder_safe.py`

File `ros2_ws/src/frontier_exploration/frontier_exploration/frontier_detector*.py` cũ không được dùng trong Nearest research launch.

## Required replay logging before official runs

Mọi official Nearest/MapEx run phải lưu đủ dữ liệu để analysis/oracle/offline metric không cần rerun robot chỉ vì thiếu log.

### Exact policy decision state

Tại mỗi policy decision phải lưu **chính OccupancyGrid đã freeze để tạo/rank candidate**, gồm:
- raw OccupancyGrid values;
- width/height/resolution/origin/frame/map timestamp;
- exact fixed-canvas reprojection;
- map-frame robot pose dùng cho ranking;
- candidate-generation/ranking computation time.

Không được lấy một `latest_map` muộn hơn sau `ComputePathToPose` rồi gọi đó là decision snapshot.

### Candidate-level logging

Mỗi decision phải lưu toàn bộ candidate set tối thiểu:
- row/col và x/y;
- Euclidean distance;
- raw/policy rank;
- `<1m` validity rejection;
- execution-suppression state;
- Nav2 path validation result;
- planner check time;
- selected flag/path length;
- execution success/failure.

### Navigation result detail

Ngoài `SUCCEEDED/FAILED`, phải giữ detail khi có thể: rejected, timeout, stall/no-progress, Nav2 status/result error.

### Periodic/final map retention

Recorder lưu raw + fixed-canvas snapshot định kỳ mặc định `10 s` và final map. Mục đích là cho phép tính occupied IoU/TU và các AUC offline sau khi evaluator được chốt.

### Provenance

`metadata.json` phải giữ:
- git commit;
- git dirty state;
- protocol/canvas/ROI IDs;
- SHA-256 của code/config chính;
- world/spawn;
- termination reason.

## Exploration termination

MapEx gốc kết thúc/fail một trial khi không còn frontier region hợp lệ/candidate A* reachable. Trong ROS, map/TF/Nav2 cập nhật bất đồng bộ nên `hospital_v1` dùng một stabilization wrapper chung cho Nearest và MapEx trước khi tuyên bố completion:

- không còn planner-reachable frontier theo policy ở trên;
- stable `5` checks;
- check period `2 s`;
- idle ít nhất `10 s`;
- startup grace `20 s`.

Nếu trước khi mission thật sự start không tồn tại usable candidate thì phải ghi **policy failure**, không được gán nhầm successful exploration completion.

Wrapper này là protocol-level ROS adaptation và phải giống nhau giữa Nearest và MapEx; nó không thay frontier score/ranking.

## Resource budgets for secondary analysis

`hospital_v1` không dùng fixed time/distance budget làm controller termination. Primary run termination là exploration completion ở trên. Sau pilot có thể chọn một common offline analysis window/budget từ raw logs để báo thêm final coverage hoặc normalized resource progress; việc chọn cutoff phân tích không được thay đổi controller behavior hoặc ROI.

Nếu dùng normalized resource progress:

```text
time_progress = time_s / fixed_time_budget_s
distance_progress = distance_m / fixed_distance_budget_m
```

Khi đã chọn budget để báo cáo, budget đó phải giống nhau giữa mọi phương pháp.

## Repetition

- Nearest target runs: `10`
- MapEx target runs: `10`
- Minimum acceptable runs before preliminary analysis: `5` per method
- Trước official runs: chạy `nearest_pilot_005` để xác nhận robot motion, logger, alignment, exact decision snapshots, candidate logs và termination.
- Pilot/debug không được tính vào official benchmark.

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
- computation timing
- raw/periodic maps đủ để tính occupied IoU/TU offline

MapEx additionally logs all decision-level prediction/variance/visibility/IG data defined in `docs/DATA_SCHEMA.md`.

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

Không được thay spawn, Nav2, SLAM, sensor, timeout, stopping condition, fixed canvas, evaluation ROI, frontier-generation semantics, post-ranking 1 m validity rule hoặc resource budget giữa Nearest và MapEx mà không ghi rõ lý do và chạy lại baseline tương ứng.
