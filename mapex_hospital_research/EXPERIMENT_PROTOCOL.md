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
- Spawn: `x=0.0 m`, `y=12.0 m`, `yaw=-1.57 rad`

### Simulator seed policy

Gazebo launch hiện tại không expose một seed cố định. Vì vậy `hospital_v1` chọn rõ policy:

- simulator seed: **intentionally uncontrolled** (Gazebo default);
- không giả vờ rằng các run có cùng random seed;
- variability được xử lý bằng repeated runs (`5` tối thiểu, mục tiêu `10` mỗi method);
- metadata của mỗi run phải ghi `sim_seed_policy=intentionally_uncontrolled_gazebo_default_multiple_run_statistics`.

Nếu sau này thêm fixed seed thì đó là thay đổi protocol và baseline liên quan phải được xem xét chạy lại.

## Mapping / SLAM

- SLAM: `slam_toolbox` với `hospital_slam.yaml`
- Resolution: `0.05 m/cell`
- Map frame: `map`

### Fixed logging canvas

- ID: `hospital_canvas_v1`
- Resolution: `0.05 m/cell`
- Width: `1504`
- Height: `2123`
- Origin: `(-25.6, -60.1) m`

Không crop theo bounding box động và không normalize final known fraction riêng từng run thành 100%.

### Canonical evaluation ROI

- ROI: `hospital_connected_free_v1`
- Denominator: `215435` cells
- Mask SHA-256: `05d45b7aba66cb6dbb71e0406005f4a3e21875901af3ae72164b17f1d3add8d1`
- Bounds SLAM-start: `x=[-0.572445,24.588833]`, `y=[-35.091079,21.044604]`
- Structural source: Hospital wall collision mesh + flat elevator blockers
- Connectivity: 8-connected free component containing start `(0,0)`

```text
coverage(t) = known cells inside hospital_connected_free_v1 / 215435
```

ROI/canvas không được đổi giữa Nearest, MapEx và proposed method trong `hospital_v1`.

## Navigation

- Nav2: `hospital_nav2.launch.py` + `nav2_hospital_override.yaml`
- Maximum speed: `0.75 m/s`
- Hard timeout per goal: `180 s`
- Stall timeout: `30 s`
- Goal checker/sensor/SLAM/Nav2 params giống nhau giữa methods
- LiDAR topic: `/scan`
- SLAM max range: `20.0 m`

## Frontier policy shared by Nearest and MapEx

Frontier generation bám theo `castacks/MapEx` commit `53636bd1c79153acc3c74a532837d78c926bae5e`:

- free: occupancy `0`;
- unknown: occupancy `<0`;
- frontier cell: free cell kề unknown trong 8-neighbourhood;
- frontier regions: 8-connected;
- giữ region khi `size > 10`;
- representative: frontier cell gần arithmetic mean `(row,col)` của region nhất;
- không dùng custom WFD reachable-BFS, segment split, standoff hoặc nearest-safe-cell substitution.

### Hospital adaptation of MapEx 1 m rule

MapEx gốc reject selected frontier nếu `<1.0 m` sau ranking. `nearest_pilot_005` cho thấy rule này deadlock ngay startup Hospital: `534` frontier cells, `1` large region, `1` representative ở `0.469 m`, candidate duy nhất bị reject trước Nav2.

Vì vậy từ pilot_006 trở đi, Hospital **không enforce 1 m rejection**:

```text
detect MapEx frontier regions
→ representative
→ score/rank
→ giữ candidate kể cả <1m
→ Nav2 ComputePathToPose
→ no path/rejected/error: thử rank tiếp theo
→ valid path: giữ exact frontier center làm execution goal
→ NavigateToPose(exact frontier center)
```

`below_1m` vẫn được log để audit. Adaptation này phải giống nhau giữa Nearest, full MapEx và proposed method.

### Nearest scoring

```text
cost(frontier_i) = EuclideanDistance(current_pose, frontier_center_i)
selected = argmin(cost)
```

## ROS/TurtleBot4 execution adapter

MapEx simulator A* (`pyastar2d`) được thay bằng:

```text
MapEx frontier generation + method-specific scoring
→ Hospital below-1m adaptation
→ Nav2 ComputePathToPose
→ path tồn tại? dùng làm reachability evidence
→ Nav2 NavigateToPose(exact frontier center)
```

**Execution-goal invariant:** endpoint cuối của `ComputePathToPose` không được dùng thay cho frontier center. Planner có tolerance nên endpoint có thể cách requested frontier đáng kể. `hospital_v1` chỉ dùng path để xác nhận candidate có planner path; goal thực thi phải lấy từ `/frontier_selected` và có `x/y` trùng exact MapEx frontier center.

Recorder phải giữ riêng:

```text
frontier_x/frontier_y              = exact MapEx frontier center
goal_x/goal_y                      = actual NavigateToPose goal
planner_endpoint_x/y               = last pose of ComputePathToPose path
planner_endpoint_to_frontier_m     = audit only
goal_source                        = exact_frontier_center
```

`goal_x/y` phải trùng `frontier_x/y` trong official runs. Path endpoint chỉ là diagnostic/reachability evidence, không phải nearest-safe-cell substitution hay standoff goal.

`nearest_pilot_007` đã phát hiện bug adapter cũ: planner endpoint bị snap khoảng `0.5 m` theo tolerance và manager gửi endpoint đó thay vì exact frontier, làm Nav2 báo success khi robot gần như không di chuyển. Các run dùng semantics cũ không được tính là official benchmark.

## Benchmark clock

`time_s = 0` phải bắt đầu tại **first policy decision before computation**: sau khi map/TF/Nav2 đã ready nhưng ngay trước lần đầu chạy candidate computation.

Điều này đảm bảo:

- Nearest tính cả frontier-generation/ranking cost của decision đầu;
- MapEx sau này tính cả LaMa ensemble, mean/variance, visibility, IG và ranking của decision đầu;
- không bias Coverage-vs-time bằng cách bắt đầu clock sau khi path đã được chọn.

Policy publish timestamp chính xác qua `/frontier_exploration_start`; recorder dùng timestamp đó làm `exploration_start_sim_s`.

## Required replay logging before official runs

### Exact policy decision state

Mỗi policy decision phải lưu chính frozen OccupancyGrid dùng để tính/rank candidate:

- raw OccupancyGrid + width/height/resolution/origin/frame/timestamp;
- fixed-canvas reprojection;
- map-frame robot pose dùng cho ranking;
- candidate-generation computation time;
- full candidate set/ranking/status.

### Exact exhausted state

Khi ranked candidate set rỗng, vẫn phải lưu một exact policy decision có map + pose + candidate audit và:

```text
outcome = exhausted_no_ranked_candidate
```

Không được chỉ dựa vào periodic/final recorder snapshot để suy ra termination state.

Nếu có candidates nhưng tất cả fail Nav2 path validation, policy decision phải kết thúc bằng:

```text
outcome = no_nav2_reachable_ranked_candidate
```

### Candidate-level schema

Mỗi candidate có khóa ổn định trong policy decision:

```text
policy_decision_id + candidate_id
```

và tối thiểu:

- raw/policy rank;
- row/col, x/y;
- distance;
- `below_1m` (diagnostic only);
- execution-suppressed flag;
- Nav2 path status/timing;
- selected flag/path length;
- execution result.

`decisions.csv.selected_candidate_id` phải join trực tiếp tới `candidates.csv`, không suy ra từ tọa độ.

### Navigation result detail

Ngoài `SUCCEEDED/FAILED`, giữ detail khi có thể: rejected, timeout, stall/no-progress, Nav2 status/result error. Detail phải gắn với exact frontier execution goal, không với planner endpoint.

### Periodic/final map retention

Recorder lưu raw + fixed-canvas snapshot mỗi `10 s` và final map. Các map này dùng để tính offline occupied IoU/TU/AUC khi evaluator được chốt.

### Provenance

`metadata.json` phải giữ:

- git commit + git dirty state;
- protocol/canvas/ROI IDs;
- source code/config SHA-256;
- SHA-256 của **installed runtime files thực tế** dưới package share;
- world/spawn;
- simulator seed policy;
- benchmark-clock source;
- termination reason.

Official run (`nearest_001...`) phải bắt đầu từ clean git worktree.

## Exploration termination

Primary termination:

- không còn planner-reachable frontier theo Hospital policy;
- stable `5` checks;
- check period `2 s`;
- idle ít nhất `10 s`;
- startup grace `20 s`.

Nếu mission chưa thật sự start và không tồn tại Nav2-reachable candidate thì ghi policy failure, không gán successful completion.

## Metrics required per run

- `known_fraction` vs time/distance;
- `coverage` vs time/distance;
- total distance/time;
- frontier goals attempted/succeeded/failed;
- success rate;
- termination reason;
- computation timing;
- raw maps đủ để tính occupied IoU/TU offline.

MapEx thêm prediction/variance/visibility/IG/ranking data theo `docs/DATA_SCHEMA.md`.

## Repetition

- Nearest target: `10` runs
- MapEx target: `10` runs
- Minimum preliminary: `5` per method
- `nearest_pilot_005`: diagnostic deadlock pilot, không benchmark
- `nearest_pilot_007`: pilot cuối để validate motion + final logging schema
- Pilot/debug không tính vào official benchmark

## Exploration-stage comparison

- Primary stage axis: absolute `coverage` trên cùng ROI.
- Không kéo giãn final state từng run thành 100%.
- Stage threshold chốt một lần sau pilot.
- Tại cùng coverage stage so time-to-stage, distance-to-stage, goal outcomes, computation và diagnostic metrics.
- Hiệu quả tổng thể báo bằng Coverage-vs-time, Coverage-vs-distance, Coverage AUC và final coverage dưới cùng fixed budget.

## Fair-comparison rule

Không đổi spawn, sensor, SLAM, Nav2, timeout, stopping condition, canvas, ROI, frontier-generation semantics, Hospital below-1m adaptation, exact-frontier execution-goal semantics, benchmark-clock definition hoặc resource budget giữa Nearest và MapEx mà không ghi rõ lý do và đánh giá lại baseline.
