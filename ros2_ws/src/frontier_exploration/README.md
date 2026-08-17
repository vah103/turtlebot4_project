# Frontier Exploration Baseline

ROS 2 frontier-based exploration baseline cho TurtleBot4.

## Pipeline hiện tại

```text
SLAM /map
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

Baseline chạy trực tiếp trên raw SLAM `/map`. Node `frontier_map_preprocessor` vẫn được giữ trong package cho thí nghiệm riêng nhưng **không còn nằm trong launch baseline mặc định**, vì WFD đã tự giới hạn tìm kiếm trong reachable free-space và việc sửa occupancy map trước detector sẽ làm baseline kém thuần hơn.

Điểm quan trọng: robot **không được gửi thẳng tới frontier representative**. Representative dùng để mô tả/xếp frontier; navigation goal được materialize từ một ô `free` ở phía explored-side và phải qua Nav2 costmap filter trước khi planner kiểm tra path.

## Frontier extraction

Core ROS-independent nằm trong `frontier_exploration/frontier_core.py`.

1. Từ pose robot, tìm free seed gần nhất nếu cần.
2. BFS 4-connected chỉ qua vùng `free` robot tiếp cận được.
3. Trong reachable free component, một cell là frontier nếu nó nằm ở phía `free` và chạm `unknown` trong lân cận 8 hướng.
4. Frontier cells được cluster bằng 8-connectivity.
5. Nếu connected boundary quá dài, cluster được chia thành local segment theo `segment_radius_m` để tạo nhiều candidate thay vì một centroid cho toàn bộ biên.
6. Mỗi segment lấy actual frontier cell gần centroid nhất làm representative.

Đây là WFD-style reachable search; cách đánh dấu frontier dùng **free-side boundary** thay vì unknown-side boundary như một số implementation khác. Hai cách đều biểu diễn cùng biên free/unknown nhưng cần mô tả đúng khi viết báo cáo.

## Safe navigation goal

Với mỗi frontier segment, detector tìm các ô `free` lân cận ở phía explored-side và loại các goal bị global/local Nav2 costmap đánh giá blocked. Trong số còn lại, detector ưu tiên free goal vừa đủ xa robot và gần centroid nhất; nếu frontier representative đã đủ xa nhưng free goal phía trong chỉ hơi gần hơn ngưỡng, detector dùng safe fallback gần centroid thay vì loại cả frontier.

Candidate được xếp theo khoảng cách Euclid từ robot tới frontier representative, sau đó `ComputePathToPose` kiểm tra lần lượt. Vì vậy baseline vẫn là **Nearest Reachable Frontier**, không phải MRTSP/MapEx scoring.

## Goal lifecycle

Khi planner đang kiểm tra candidate hoặc robot đang thực thi một frontier goal, detector giữ nguyên map snapshot/candidate identity. Map mới từ SLAM chỉ được giữ lại dưới dạng deferred snapshot. Điều này tránh lỗi grid index bị hiểu theo map geometry mới nếu SLAM resize hoặc shift origin trong lúc robot đang chạy.

Sau `SUCCEEDED`:

1. manager publish `/frontier_completed_goal`;
2. detector ghi nhận frontier vừa thăm trong vùng cooldown ngắn để tránh gửi lại ngay cùng frontier;
3. detector lấy map mới nhất đã deferred;
4. chờ `post_goal_settle_sec`;
5. chạy WFD và chọn frontier mới.

Sau failure/stall, frontier đúng của goal đang thực thi được suppress tạm thời rồi detector chuyển sang candidate khác. Completed-frontier suppression chỉ ngắn hạn để chống immediate loop; failed-frontier suppression dài hơn.

## Stopping condition

Các frontier cell nhỏ hoặc nằm sát vùng unknown có thể còn tồn tại dù Nav2 không
thể đi tới. Vì vậy baseline không chờ số frontier cell thô bằng 0. Detector chỉ
báo hoàn thành sau khi toàn bộ candidate an toàn đã được planner kiểm tra mà
không còn candidate reachable trong nhiều lần kiểm tra ổn định. Khi đó node log
`Exploration complete` và publish `true` trên `/exploration_complete`.

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

`exploration_manager` chỉ thực thi path đã được detector xác nhận. Node không publish `cmd_vel` trực tiếp. Navigation mặc định tắt:

```yaml
enable_navigation: false
```

Progress watchdog:

```yaml
stall_timeout_sec: 30.0
stall_progress_epsilon_m: 0.02
navigation_timeout_sec: 180.0
```

Timeout 30 s cho phép Nav2 có thời gian xoay/recovery; timer chỉ reset khi `distance_remaining` cải thiện có ý nghĩa.

## Tham số chính

```yaml
frontier_detector:
  map_topic: /map
  global_costmap_topic: /global_costmap/costmap
  local_costmap_topic: /local_costmap/costmap
  segment_radius_m: 0.75
  min_selection_distance_m: 0.60
  costmap_occ_threshold: 65
  failed_goal_radius_m: 0.40
  failed_goal_cooldown_sec: 60.0
  completed_frontier_radius_m: 0.40
  completed_frontier_cooldown_sec: 15.0
  post_goal_settle_sec: 1.0
  completion_stable_cycles: 5
  completion_min_idle_sec: 10.0
  completion_check_period_sec: 2.0

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

Log mong đợi:

```text
Frontier extraction: WFD-style reachable-space BFS + 8-connected frontier clustering
Frontier cells: ... | connected clusters: ... | segments: ... | representatives: ... | safe eligible: ...
Checking Nav2 path to frontier-safe free goal: frontier=(...), goal=(...)
Reachable frontier selected: frontier=(...), goal=(...), euclidean=..., path=...
```

## Autonomous simulation

Chỉ bật sau khi build/test và dry-run đều PASS:

```bash
ros2 launch frontier_exploration frontier_autonomy.launch.py \
  use_sim_time:=true \
  enable_navigation:=true \
  enable_sim_twist_adapter:=true
```

## Nguồn thuật toán tham khảo

Bản này là Python implementation riêng cho TurtleBot4, không vendor/copy nguyên package bên ngoài. Kiến trúc được đối chiếu với:

- `robo-friends/m-explore-ros2` (BSD): frontier BFS/nearest-frontier loop, progress timeout và blacklist pattern.
- `mertgulerx/frontier_exploration_ros2` (Apache-2.0): costmap-aware frontier validation, accessible free goal gần frontier, best-far/best-any goal selection, post-goal settle và suppression/goal-lifecycle ideas.

Repo tham khảo:

- https://github.com/robo-friends/m-explore-ros2
- https://github.com/mertgulerx/frontier_exploration_ros2

MRTSP, bounded-horizon DP, information-gain scoring, decision-map optimization và visible-reveal preemption **không được đưa vào baseline**, để giữ vai trò Frontier cơ bản trước khi so sánh với MapEx/hướng đề xuất.
