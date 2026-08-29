# Experiment Protocol

Mọi phương pháp so sánh phải dùng cùng protocol, trừ khi thay đổi đó chính là biến thí nghiệm và được ghi rõ.

## Protocol identity

- Protocol version: `hospital_v2`
- Fixed canvas ID: `hospital_canvas_v1`
- Evaluation ROI ID: `hospital_connected_free_v1`

`hospital_v2` thay đổi runtime SLAM grid từ `0.05` sang `0.10 m/cell` để Nearest frontier, MapEx frontier và MapEx prediction cùng làm việc trên grid `0.10 m/cell`. Fixed logging canvas và canonical ROI **không đổi**: evaluation vẫn diễn ra trên `hospital_canvas_v1` ở `0.05 m/cell` để giữ nguyên thước đo coverage đã freeze.

## Environment

- World: `Hospital flat` (`hospital_aws_flat.sdf`)
- Simulator: Gazebo / TurtleBot4 simulation stack hiện có trong `frontier_exploration`
- Robot: TurtleBot4
- Spawn: `x=0.0 m`, `y=12.0 m`, `yaw=-1.57 rad`

### Simulator seed policy

Gazebo launch hiện tại không expose một seed cố định. Vì vậy `hospital_v2` chọn rõ policy:

- simulator seed: **intentionally uncontrolled** (Gazebo default);
- không giả vờ rằng các run có cùng random seed;
- variability được xử lý bằng repeated runs (`5` tối thiểu, mục tiêu `10` mỗi method);
- metadata của mỗi run phải ghi `sim_seed_policy=intentionally_uncontrolled_gazebo_default_multiple_run_statistics`.

Nếu sau này thêm fixed seed thì đó là thay đổi protocol và baseline liên quan phải được xem xét chạy lại.

## Mapping / SLAM

- SLAM: `slam_toolbox` với `hospital_slam.yaml`
- Runtime resolution: `0.10 m/cell`
- Map frame: `map`
- Mapping speed cap: `0.45 m/s`
- `minimum_travel_distance = 0.10 m`
- `minimum_travel_heading = 0.10 rad`
- `minimum_time_interval = 0.15 s`
- LiDAR max range dùng bởi SLAM: `20 m`

`hospital_slam_no_loop.yaml` phải giống active profile về runtime resolution, scan/keyframe parameters và chỉ khác ở `do_loop_closing=false` + debug logging, để A/B test loop closure không bị confound.

### Fixed logging canvas

- ID: `hospital_canvas_v1`
- Resolution: `0.05 m/cell`
- Width: `1504`
- Height: `2123`
- Origin: `(-25.6, -60.1) m`

Runtime SLAM resolution và evaluation canvas resolution là hai khái niệm tách biệt. Với `hospital_v2`, mỗi cell axis-aligned `0.10 m` của `/map` được reproject bằng nearest-neighbour area preservation thành khối `2 x 2` cell trên fixed canvas `0.05 m`. Raw OccupancyGrid vẫn được lưu nguyên resolution để replay/audit.

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

ROI/canvas không được đổi giữa Nearest, MapEx và proposed method trong `hospital_v2`. Thay đổi runtime SLAM resolution **không** làm thay đổi denominator hoặc SHA của ROI vì ROI được định nghĩa trên fixed evaluation canvas.

## Navigation

- Nav2: `hospital_nav2.launch.py` + `nav2_hospital_override.yaml`
- Maximum speed: `0.45 m/s`
- `GridBased` planner tolerance: `0.0 m`
- Hard timeout per goal: `180 s`
- Stall timeout: `30 s`
- LiDAR topic: `/scan`

### Position-only frontier goal semantics

MapEx frontier là một **position-only** target `(x,y)`. Frontier policy không định nghĩa terminal yaw.

Vì vậy Hospital execution phải đảm bảo:

- exact frontier `x/y` là execution position;
- không ép frontier thành `(x,y,yaw=0)`;
- `general_goal_checker.yaw_goal_tolerance = pi`;
- `FollowPath.GoalAngleCritic.enabled = false`;
- quaternion trong request chỉ là neutral orientation seed, không phải policy objective.

`xy_goal_tolerance` của Nav2 vẫn được giữ theo base Nav2 config; chỉ final yaw constraint bị vô hiệu hóa.

## Frontier policy shared by Nearest and MapEx

Frontier generation bám theo `castacks/MapEx` commit `53636bd1c79153acc3c74a532837d78c926bae5e`:

- runtime policy grid: `0.10 m/cell`;
- free: occupancy `0`;
- unknown: occupancy `<0`;
- frontier cell: free cell kề unknown trong 8-neighbourhood;
- frontier regions: 8-connected;
- giữ region khi `size > 10`;
- representative: frontier cell gần arithmetic mean `(row,col)` của region nhất;
- không dùng custom WFD reachable-BFS, segment split, standoff hoặc nearest-safe-cell substitution.

### Hospital adaptation of MapEx 1 m rule

MapEx gốc reject selected frontier nếu `<1.0 m` sau ranking. `nearest_pilot_005` cho thấy rule này deadlock ngay startup Hospital: `534` frontier cells, `1` large region, `1` representative ở `0.469 m`, candidate duy nhất bị reject trước Nav2.

Vì vậy Hospital **không enforce 1 m rejection**:

```text
detect MapEx frontier regions
→ representative
→ score/rank
→ giữ candidate kể cả <1m
→ Nav2 ComputePathToPose
→ chỉ accept nếu action status == SUCCEEDED và path non-empty
→ valid path: giữ exact frontier x/y làm execution position
→ NavigateToPose(exact frontier x/y, yaw unconstrained)
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
→ Nav2 ComputePathToPose(tolerance=0.0 m)
→ require action STATUS_SUCCEEDED + non-empty path
→ path dùng như reachability evidence
→ Nav2 NavigateToPose(exact frontier x/y; final yaw ignored)
```

**Execution-goal invariant:** endpoint cuối của `ComputePathToPose` không được dùng thay cho frontier center. Goal thực thi phải có `x/y` trùng exact MapEx frontier center. Path endpoint chỉ là diagnostic/reachability evidence.

Recorder phải giữ riêng:

```text
frontier_x/frontier_y              = exact MapEx frontier center
goal_x/goal_y                      = actual NavigateToPose position
planner_endpoint_x/y               = last pose of validation path
planner_endpoint_to_frontier_m     = audit only
goal_source                        = exact_frontier_center
goal_yaw_semantics                 = ignored_by_hospital_goal_checker
```

`selected_path_length_m` là **planner validation path length**, không phải quãng đường robot thực thi thực tế. Quãng đường thực tế lấy từ odometry/trajectory.

## Planner revalidation and navigation failures

Một lần planner no-path/rejected/error không được coi là bằng chứng vĩnh viễn trong hệ ROS online.

- `NavigateToPose` main-goal error `206 = GOAL_OCCUPIED` và `208 = NO_VALID_PATH` được coi là **planner-blocking failures**. Frontier đó và candidate trong bán kính `0.10 m` bị suppress khỏi ordinary selection để node chuyển sang frontier khác thay vì retry vô hạn.
- Suppression `206/208` không phải blacklist vĩnh viễn. Khi không còn normally selectable candidate, full eligible representative set phải được `ComputePathToPose` revalidate lại; candidate reachable trở lại phải được bỏ suppression và exploration tiếp tục.
- Stable non-empty ranked candidate set phải chạy **planner revalidation** mỗi `2 s` trước khi tăng completion streak; completion cần `5` validated exhausted sweeps trên stable set.
- Nếu map/frontier set thay đổi, completion window reset.
- Các failure khác của main goal (ví dụ controller `105 FAILED_TO_MAKE_PROGRESS`) vẫn đi qua path-guided recovery nếu có main-goal `/plan` hợp lệ; chúng không tự động trở thành permanent planner-unreachable evidence.
- Một failed temporary subgoal không tự blacklist main frontier.

## Benchmark clock

`time_s = 0` bắt đầu tại **first policy decision before computation**: sau khi map/TF/Nav2 ready nhưng ngay trước lần đầu candidate computation.

Điều này đảm bảo Nearest tính cả frontier-generation/ranking cost đầu tiên và MapEx sau này tính cả LaMa/visibility/IG cost đầu tiên.

Policy publish timestamp chính xác qua `/frontier_exploration_start`; recorder dùng timestamp đó làm `exploration_start_sim_s`.

## Required replay logging before official runs

### Exact policy decision state

Mỗi policy decision phải lưu:

- raw OccupancyGrid + width/height/resolution/origin/frame/timestamp;
- fixed-canvas reprojection;
- map-frame robot pose dùng cho ranking;
- candidate-generation computation time;
- full candidate set/ranking/status;
- Nav2 action status cho mỗi planner check.

### Exact exhausted state

Khi ranked candidate set rỗng:

```text
outcome = exhausted_no_ranked_candidate
```

Nếu có candidates nhưng cả planner sweep fail:

```text
outcome = no_nav2_reachable_ranked_candidate
```

Non-empty completion phải có repeated terminal decisions từ planner revalidation, không chỉ lặp một cached no-path signature.

### Candidate-level schema

Mỗi candidate có khóa ổn định:

```text
policy_decision_id + candidate_id
```

và tối thiểu:

- raw/policy rank;
- row/col, x/y;
- distance;
- `below_1m`;
- execution-suppressed flag;
- Nav2 action status + path status/timing;
- selected flag/path length;
- execution result.

`decisions.csv.selected_candidate_id` phải join trực tiếp tới `candidates.csv`.

### Periodic/final map retention

Recorder lưu raw + fixed-canvas snapshot mỗi `10 s` và final map.

### Effective runtime provenance

Mỗi run phải hash source/installed runtime files và ayrıca lưu:

- base `/opt/ros/.../nav2_bringup/params/nav2_params.yaml` hash;
- `runtime_nav2_merged.yaml` là effective base+Hospital override dùng cho run;
- hash của merged YAML;
- world/spawn, seed policy, benchmark clock source, termination reason.

Official run (`nearest_001...`) phải bắt đầu từ clean git worktree.

## Exploration termination

Primary termination:

- không còn planner-reachable frontier theo Hospital policy;
- non-empty stable set được planner revalidate mỗi `2 s`;
- stable `5` exhausted sweeps;
- idle ít nhất `10 s`;
- startup grace `20 s`.

Nếu mission chưa thật sự start và không tồn tại Nav2-reachable candidate thì ghi policy failure, không gán successful completion.

## Metrics required per run

- `known_fraction` vs time/distance;
- `coverage` vs time/distance;
- total odometry distance/time;
- frontier goals attempted/succeeded/failed;
- success rate;
- termination reason;
- computation timing;
- raw maps đủ để tính occupied IoU/TU offline.

Validator phải tách **data/protocol integrity** khỏi **experiment health** và phát hiện tối thiểu pathology "goal SUCCEEDED nhưng robot gần như không di chuyển".

## Repetition

- Nearest target: `10` runs
- MapEx target: `10` runs
- Minimum preliminary: `5` per method
- `nearest_pilot_005`: diagnostic 1 m deadlock (`hospital_v1` history)
- `nearest_pilot_007`: diagnostic tolerance-snapped execution-goal bug (`hospital_v1` history)
- `nearest_pilot_008`: exact-frontier execution check; map warping observed (`hospital_v1` history)
- `nearest_pilot_009`: mapping-profile/controller diagnostic; revealed artificial yaw/runtime robustness issues (`hospital_v1` history)
- next corrected pilot after the resolution change must record `protocol_version=hospital_v2`
- Pilot/debug không tính vào official benchmark

## Fair-comparison rule

Không đổi spawn, sensor, SLAM runtime resolution, Nav2, planner tolerance, timeout, stopping condition, canvas, ROI, frontier-generation semantics, Hospital below-1m adaptation, position-only goal semantics, planner-blocking `206/208` suppression/revalidation, benchmark-clock definition hoặc resource budget giữa Nearest và MapEx mà không ghi rõ lý do và đánh giá lại baseline.
