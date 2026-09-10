# Experiment Protocol

Mọi phương pháp so sánh phải dùng cùng protocol, trừ khi thay đổi đó chính là biến thí nghiệm và được ghi rõ.

## Protocol identity

- Protocol version: `hospital_v2`
- Fixed canvas ID: `hospital_canvas_v1`
- Evaluation ROI ID: `hospital_connected_free_v1`

`hospital_v2` dùng runtime SLAM/policy grid `0.10 m/cell` để Nearest Frontier, MapEx frontier và MapEx prediction làm việc trên cùng độ phân giải. Fixed logging canvas vẫn ở `0.05 m/cell`; runtime grid và evaluation canvas là hai khái niệm độc lập.

## Environment

- World: `Hospital flat` (`hospital_aws_flat.sdf`)
- Simulator: Gazebo / TurtleBot4 simulation stack
- Robot: TurtleBot4
- Spawn: `x=0.0 m`, `y=12.0 m`, `yaw=-1.57 rad`

### Simulator seed policy

Gazebo launch hiện không expose một seed cố định. Vì vậy:

- simulator seed: **intentionally uncontrolled**;
- không giả vờ các run có cùng random seed;
- variability được xử lý bằng repeated runs (`5` tối thiểu, mục tiêu `10` mỗi method);
- metadata ghi `sim_seed_policy=intentionally_uncontrolled_gazebo_default_multiple_run_statistics`.

Nếu sau này thêm fixed seed thì đó là thay đổi protocol và baseline liên quan phải được xem xét chạy lại.

## Mapping / SLAM

- SLAM: `slam_toolbox`
- Runtime resolution: `0.10 m/cell` đối với `hospital_v2`
- Map frame: `map`
- Mapping speed cap mục tiêu: `0.45 m/s`
- `minimum_travel_distance = 0.10 m`
- `minimum_travel_heading = 0.10 rad`
- `minimum_time_interval = 0.15 s`
- LiDAR max range dùng bởi SLAM: `20 m`

Khi A/B một biến SLAM, các tham số còn lại phải được giữ giống nhau để tránh confound.

### Fixed logging canvas

- ID: `hospital_canvas_v1`
- Resolution: `0.05 m/cell`
- Width: `1504`
- Height: `2123`
- Origin: `(-25.6, -60.1) m`

Với map axis-aligned `0.10 m`, mỗi runtime cell được reproject bằng nearest-neighbour area preservation thành khối `2 x 2` cell trên fixed canvas `0.05 m`. Raw OccupancyGrid vẫn được lưu nguyên resolution để replay/audit.

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

ROI/canvas không được đổi giữa Nearest, MapEx và proposed method trong cùng protocol.

## Navigation

Nav2 là execution layer chung cho Nearest và MapEx. Các method phải dùng cùng Nav2 configuration khi so sánh trực tiếp.

### Position-only frontier goal semantics

Frontier là target vị trí `(x,y)`; policy không tối ưu terminal yaw.

Yêu cầu:

- exact frontier `x/y` là execution position;
- không thay frontier bằng nearest-safe-cell hoặc planner endpoint;
- quaternion trong `NavigateToPose` chỉ là neutral orientation seed;
- effective Nav2 goal checker phải không biến terminal yaw thành biến policy khác giữa các method.

Quãng đường thực tế lấy từ odometry/trajectory, không suy từ một global path duy nhất vì Nav2 có thể replan.

## Frontier policy shared by Nearest and MapEx

Frontier generation bám theo public MapEx implementation (`castacks/MapEx`, reference commit `53636bd1c79153acc3c74a532837d78c926bae5e`):

- runtime policy grid: `0.10 m/cell` trong `hospital_v2`;
- free: occupancy `0`;
- unknown: occupancy `<0`;
- frontier cell: free cell kề unknown trong 8-neighbourhood;
- frontier regions: 8-connected;
- giữ region khi `size > 10`;
- representative: frontier cell gần arithmetic mean `(row,col)` của region nhất;
- không dùng custom WFD reachable-BFS, segment split, standoff hoặc nearest-safe-cell substitution.

### Shared 1 m frontier-distance preference + near-frontier fallback

Public MapEx có `cur_pose_dist_threshold_m = 1`. Trong ROS/TurtleBot4 adaptation, hard rejection toàn bộ candidate `<1 m` có thể làm exploration đứng ngay khi frontier hiện tại chỉ có các representative gần robot. Vì vậy Nearest và MapEx dùng **cùng một luật eligibility**:

```text
raw frontier representatives
→ nếu tồn tại ít nhất 1 representative có distance >= 1.0 m:
     chỉ nhóm >= 1.0 m được đưa vào ordinary policy selection
→ nếu tất cả representative hiện tại đều < 1.0 m:
     bật near-frontier fallback và cho phép xét toàn bộ nhóm gần
→ sau đó mới áp dụng execution/planner suppression
→ policy chọn candidate tốt nhất trong tập còn selectable
```

Các điểm cần giữ đúng:

- `1.0 m` là **preference threshold**, không phải điều kiện completion;
- `all frontiers <1m` **không** đồng nghĩa exploration complete;
- fallback chỉ bật khi toàn bộ raw representative hiện tại đều dưới ngưỡng, không phải vì các candidate xa đang bị suppression;
- luật này phải giống nhau giữa Nearest, MapEx và proposed method nếu muốn so sánh policy công bằng;
- recorder phải log `below_1m`, `distance_eligible` và `near_frontier_fallback` để đo ảnh hưởng thực tế của adaptation.

Đây là ROS2 execution adaptation; không mô tả là byte-for-byte reproduction của public simulator.

### Nearest scoring

```text
cost(frontier_i) = EuclideanDistance(current_pose, frontier_i)
selected = argmin(cost) trong tập selectable
```

### MapEx scoring

MapEx giữ nguyên core scoring:

```text
P1,P2,P3 = LaMa ensemble predictions
mean_map = mean(P1,P2,P3)
variance_map = variance(P1,P2,P3)
visibility(f) = probabilistic predicted visibility
IG(f) = sum variance over predicted-visible AND currently-unknown cells
score(f) = IG(f) / EuclideanDistance(robot, f)
selected = argmax(score) trong tập selectable
```

Luật 1 m/fallback chỉ thay **candidate eligibility**; không thay công thức `IG/d`.

## ROS/TurtleBot4 execution adapter

Public MapEx simulator dùng A* (`pyastar2d`). ROS2 implementation hiện dùng Nav2:

```text
frontier generation
→ method-specific ranking/scoring
→ shared 1 m preference / all-near fallback
→ shared suppression rules
→ NavigateToPose(exact frontier x/y)
→ /plan được lưu làm navigation diagnostic và path-guided recovery evidence
```

Không bắt buộc `ComputePathToPose` trước **mọi** ordinary frontier. `ComputePathToPose` được dùng cho terminal planner revalidation khi không còn normally selectable candidate sau planner-blocking suppression.

**Execution-goal invariant:** goal thực thi phải giữ exact frontier `x/y`; planner/path endpoint không được dùng để thay frontier center.

Recorder giữ riêng:

```text
frontier x/y       = exact selected representative
goal target x/y    = actual NavigateToPose position
plan endpoint x/y  = last pose của /plan, diagnostic only
plan endpoint error = distance(endpoint, exact frontier), diagnostic only
```

`plans.csv.path_length_m` là chiều dài path được Nav2 publish tại thời điểm đó, **không** phải total executed trajectory. Quãng đường robot thực lấy từ `trajectory.csv` / `metrics.csv.distance_m`.

## Planner revalidation and navigation failures

Một planner/navigation failure không được coi là bằng chứng vĩnh viễn trong hệ ROS online.

- `NavigateToPose` main-goal error `206 = GOAL_OCCUPIED` và `208 = NO_VALID_PATH` là planner-blocking failures. Frontier và candidate rất gần nó bị suppress khỏi ordinary selection để exploration chuyển sang frontier khác.
- Suppression `206/208` không phải blacklist vĩnh viễn. Khi không còn normally selectable candidate, eligible representative set được `ComputePathToPose` revalidate lại.
- Candidate planner-reachable trở lại phải được bỏ planner suppression và exploration tiếp tục.
- Stable non-empty candidate set cần repeated exhausted revalidation sweeps trước completion.
- Nếu map/frontier set thay đổi, completion verification reset.
- `105 FAILED_TO_MAKE_PROGRESS` là execution/controller failure: cho path-guided recovery có giới hạn rồi temporary cooldown; nó không tự trở thành permanent planner-unreachable evidence.
- Failed temporary subgoal không tự biến main frontier thành permanent blacklist.

## Benchmark clock

`time_s = 0` bắt đầu tại **first policy decision before computation**: sau khi map/TF/Nav2 ready nhưng ngay trước candidate computation đầu tiên.

- Nearest vì vậy tính cả frontier-generation/ranking cost đầu tiên.
- MapEx tính cả LaMa prediction + visibility + IG/scoring đầu tiên.
- Recorder lưu timestamp này trong `metadata.json` dưới `exploration_start_sim_s`.

Không yêu cầu một topic riêng để định nghĩa clock; source of truth là timestamp được recorder freeze khi decision đầu tiên bắt đầu.

## Required replay logging before official runs

### Exact policy decision state

Mỗi policy decision phải lưu đủ để replay lựa chọn:

- raw OccupancyGrid + width/height/resolution/origin/frame/timestamp;
- fixed-canvas reprojection;
- map-frame robot pose dùng cho ranking;
- candidate computation time;
- full frontier representative set;
- raw rank / policy rank;
- `below_1m`, `distance_eligible`, `near_frontier_fallback`;
- execution/planner suppression state;
- selected candidate và decision outcome.

Khóa join chung:

```text
policy_decision_id + candidate_id
```

### Nearest replay invariant

Từ `observed_map_raw.npz` + robot pose phải có thể tái tạo:

```text
frontier cells
→ regions >10
→ representatives
→ distance
→ 1m preference/fallback
→ suppression
→ nearest selectable candidate
```

### MapEx replay extension

MapEx lưu thêm:

- G1/G2/G3 predictions (khi prediction retention bật);
- ensemble mean / variance;
- Information Gain;
- score `IG/d`;
- visible-unknown cell count;
- prediction/scoring timing.

### Periodic/final map retention

Recorder lưu raw + fixed-canvas snapshot khoảng mỗi `10 s` và một final snapshot; `snapshots.csv` là index chung cho hai method.

### Effective runtime provenance

Mỗi run phải lưu/hash tối thiểu:

- policy + recorder source;
- active launch/profile;
- relevant SLAM/frontend source;
- base installed Nav2 params;
- project Nav2 override;
- `runtime_nav2_merged.yaml`;
- world/ground-truth source khi áp dụng;
- simulator seed policy;
- benchmark clock source;
- termination reason.

Official runs nên bắt đầu từ clean git worktree.

## Exploration termination

Completion của shared execution layer phải bảo thủ:

- không complete khi main/subgoal còn active;
- không complete chỉ vì remaining frontiers `<1 m`;
- zero frontier regions cần stable repeated evidence;
- nếu regions còn nhưng normally selectable set bị planner-blocking suppression, dùng repeated planner revalidation;
- stable `5` exhausted sweeps;
- sweep spacing tối thiểu `2 s`;
- idle tối thiểu `10 s`;
- startup grace `20 s`;
- execution cooldown do `105` không phải terminal evidence.

## Metrics required per run

- `known_fraction` vs time/distance;
- `coverage` vs time/distance;
- total odometry distance/time;
- frontier goals attempted/succeeded/failed;
- success rate;
- termination reason;
- policy computation timing;
- fallback count/fraction;
- raw maps đủ để tính occupied IoU/TU offline.

Paper-level metrics cần nhớ: Coverage, occupied-class IoU và Topological Understanding (TU).

Validator nên tách **data/protocol integrity** khỏi **experiment health** và phát hiện pathology kiểu "goal SUCCEEDED nhưng robot gần như không di chuyển".

## Repetition

- Nearest target: `10` runs
- MapEx target: `10` runs
- Minimum preliminary: `5` per method
- pilot/debug không tính vào official benchmark.

Historical Hospital pilots trước shared 1 m fallback có thể giữ làm diagnostic nhưng không trộn với official runs của protocol mới mà không ghi rõ version.

## Fair-comparison rule

Không đổi giữa Nearest và MapEx mà không ghi rõ lý do: spawn, sensor, SLAM runtime resolution/profile, Nav2, timeout, stopping condition, fixed canvas, ROI, frontier-generation semantics, **1 m preference + all-near fallback**, position-only goal semantics, suppression/revalidation, benchmark-clock definition hoặc resource budget.

Khác biệt policy chủ yếu cần giữ đúng là:

```text
Nearest: chọn min Euclidean distance
MapEx:   chọn max IG / Euclidean distance
```

Execution adapter và recorder phải được chia sẻ tối đa để khác biệt kết quả không đến từ hạ tầng thí nghiệm.
