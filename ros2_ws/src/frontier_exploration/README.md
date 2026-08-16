# Frontier Exploration Baseline

ROS 2 frontier-based exploration baseline cho TurtleBot4.

## Pipeline hiện tại

```text
SLAM /map
  -> detector-only /frontier_map preprocessing
  -> WFD-style reachable free-space BFS
  -> free/unknown frontier cells
  -> 8-connected frontier clusters
  -> local segmentation for very long connected boundaries
  -> representative frontier point per segment
  -> accessible free goal cells beside the frontier
  -> global/local Nav2 costmap filtering
  -> nearest safe frontier ordering
  -> Nav2 ComputePathToPose reachability check
  -> NavigateToPose
  -> goal result -> map settle -> frontier refresh
```

Điểm quan trọng của bản refactor hiện tại là robot **không còn được gửi thẳng tới frontier representative**. Representative vẫn là điểm dùng để mô tả/chọn frontier, còn navigation goal được materialize từ một ô `free` lân cận và phải qua Nav2 costmap filter trước khi planner kiểm tra path.

## Frontier extraction

Core ROS-independent nằm trong `frontier_exploration/frontier_core.py`.

1. Từ pose robot, tìm free seed gần nhất nếu cần.
2. BFS 4-connected chỉ qua vùng `free` robot tiếp cận được.
3. Trong reachable free component, một cell là frontier nếu nó nằm ở phía `free` và chạm `unknown` trong lân cận 8 hướng.
4. Frontier cells được cluster bằng 8-connectivity.
5. Nếu một connected boundary quá dài, cluster được chia thành các local segment theo `segment_radius_m` để tạo nhiều candidate thay vì chỉ một centroid cho toàn bộ biên.
6. Mỗi segment lấy actual frontier cell gần centroid nhất làm representative.

Cấu trúc này giữ baseline dễ giải thích nhưng thay phần quét toàn map trước đây bằng reachable-space BFS kiểu WFD.

## Safe navigation goal

Với mỗi frontier segment, detector tìm các ô `free` ngay phía explored-side của frontier và loại:

- frontier cell itself;
- goal quá gần robot;
- goal bị global Nav2 costmap đánh giá occupied/inflated quá ngưỡng;
- goal nằm trong vùng blocked của local costmap, nếu local costmap đang phủ vị trí đó.

Trong các free goal còn lại, detector chọn điểm gần centroid của segment nhất rồi dùng `ComputePathToPose` kiểm tra reachability. Candidate vẫn được xếp theo khoảng cách Euclid từ robot tới frontier representative, nên baseline vẫn là **Nearest Reachable Frontier** chứ không phải MRTSP/MapEx scoring.

## Marker RViz

`/frontier_markers` gồm:

- `frontier_cells`: toàn bộ reachable frontier cells;
- `frontier_segment_centers`: tâm hình học của local segment;
- `frontier_representatives`: representative frontier point;
- `reachable_frontier_selected`: frontier được chọn;
- `frontier_navigation_goal`: free/costmap-safe goal thực tế gửi qua Nav2;
- `planner_candidate_checking`: goal đang được `ComputePathToPose` kiểm tra.

`/frontier_selected_path` là path đã được Nav2 planner xác nhận.

## Navigation manager

`exploration_manager` chỉ thực thi path đã được detector xác nhận. Node không publish `cmd_vel` trực tiếp.

Navigation mặc định tắt:

```yaml
enable_navigation: false
```

Sau `SUCCEEDED`, manager publish `/frontier_completed_goal`; detector reset selection, chờ `post_goal_settle_sec`, rồi rebuild WFD candidate từ map mới nhất. Sau failure/stall, manager publish `/frontier_failed_goal`; detector tạm suppress frontier region đó và chọn candidate khác.

Progress watchdog hiện dùng timeout dài hơn và epsilon nhỏ để tránh false-stall khi robot đang xoay/chỉnh local path:

```yaml
stall_timeout_sec: 30.0
stall_progress_epsilon_m: 0.02
navigation_timeout_sec: 180.0
```

## Tham số chính

```yaml
frontier_detector:
  map_topic: /frontier_map
  global_costmap_topic: /global_costmap/costmap
  local_costmap_topic: /local_costmap/costmap
  segment_radius_m: 0.75
  min_selection_distance_m: 0.60
  costmap_occ_threshold: 65
  failed_goal_radius_m: 0.40
  failed_goal_cooldown_sec: 60.0
  post_goal_settle_sec: 1.0

exploration_manager:
  enable_navigation: false
  stall_timeout_sec: 30.0
  stall_progress_epsilon_m: 0.02
```

## Build và test

```bash
cd ~/turtlebot4_project/ros2_ws
colcon build --packages-select frontier_exploration
source install/setup.bash
```

Pure frontier core có unit tests:

```bash
colcon test --packages-select frontier_exploration
colcon test-result --verbose
```

## Launch dry-run

Simulation + SLAM + Nav2 phải chạy trước.

```bash
ros2 launch frontier_exploration frontier_autonomy.launch.py \
  use_sim_time:=true \
  enable_navigation:=false \
  enable_sim_twist_adapter:=false
```

Khi đúng sẽ thấy log dạng:

```text
Frontier extraction: WFD-style reachable-space BFS + 8-connected frontier clustering
Frontier cells: ... | connected clusters: ... | segments: ... | representatives: ... | safe eligible: ...
Checking Nav2 path to frontier-safe free goal: frontier=(...), goal=(...)
Reachable frontier selected: frontier=(...), goal=(...), euclidean=..., path=...
```

## Autonomous simulation

Chỉ bật khi simulation, SLAM và Nav2 đang ổn định:

```bash
ros2 launch frontier_exploration frontier_autonomy.launch.py \
  use_sim_time:=true \
  enable_navigation:=true \
  enable_sim_twist_adapter:=true
```

## Nguồn thuật toán tham khảo

Bản này là Python implementation riêng cho TurtleBot4, **không vendor/copy nguyên package bên ngoài**. Kiến trúc được refactor sau khi đối chiếu các implementation frontier ROS 2 trưởng thành:

- `robo-friends/m-explore-ros2` (`explore_lite` ROS 2 port, BSD): WFD/frontier BFS structure, nearest-frontier loop, progress timeout và blacklist pattern.
- `mertgulerx/frontier_exploration_ros2` (Apache-2.0): costmap-aware frontier validation, lựa chọn accessible free goal gần frontier, post-goal settle và blocked/failed frontier handling.

Repo tham khảo:

- https://github.com/robo-friends/m-explore-ros2
- https://github.com/mertgulerx/frontier_exploration_ros2

Các phần MRTSP, bounded-horizon DP, information-gain scoring và visible-reveal preemption của repo tham khảo **không được đưa vào baseline này**, để baseline vẫn giữ đúng vai trò Frontier cơ bản trước khi so sánh với MapEx/hướng đề xuất.
