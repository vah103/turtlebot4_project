# Frontier Exploration Baseline

ROS 2 package riêng cho Stage 5 của đồ án TurtleBot4.

Pipeline hiện tại:

1. Đọc `nav_msgs/OccupancyGrid` từ SLAM.
2. Phát hiện frontier cell tại biên `free`–`unknown`.
3. Gom frontier bằng connected components.
4. Chia connected cluster dài thành nhiều local frontier segment.
5. Chọn representative point trên mỗi segment.
6. Lọc candidate quá gần robot.
7. Sắp xếp candidate theo khoảng cách Euclid.
8. Dùng Nav2 `ComputePathToPose` để kiểm tra reachability.
9. Chọn candidate gần nhất có path hợp lệ.
10. Tùy chọn gửi candidate đó qua `NavigateToPose` bằng `exploration_manager`.

## Hai node

### `frontier_detector`

Node detector + selector. Node này không điều khiển robot. Nó publish:

- `/frontier_markers`
- `/frontier_selected_path`

Các marker gồm:

- `frontier_cells`
- `frontier_segment_centers`
- `frontier_representatives`
- `planner_candidate_checking`
- `reachable_frontier_selected`

### `exploration_manager`

Node thực thi frontier goal qua Nav2 `NavigateToPose`.

**Mặc định `enable_navigation: false`**, vì vậy package vẫn an toàn khi launch bình thường. Chỉ khi người vận hành chủ động bật `enable_navigation:=true` thì manager mới gửi goal chuyển động.

Manager chỉ dùng path đã được `frontier_detector` xác nhận bằng `ComputePathToPose`; node không publish `cmd_vel` trực tiếp.

## Tham số chính

```yaml
frontier_detector:
  robot_frame: base_link
  planner_action: /compute_path_to_pose
  segment_radius_m: 0.75
  min_selection_distance_m: 0.60

exploration_manager:
  navigate_action: /navigate_to_pose
  enable_navigation: false
  goal_repeat_tolerance_m: 0.20
```

## Build

```bash
cd ros2_ws
colcon build --packages-select frontier_exploration --symlink-install
source install/setup.bash
```

## Chỉ test detector + planner

```bash
ros2 launch frontier_exploration frontier_detector.launch.py
```

Cần có Nav2 planner:

```bash
ros2 action list | grep compute_path_to_pose
```

Khi chạy đúng sẽ thấy dạng:

```text
Frontier cells: 533 | connected clusters: 1 | segments: 9 | representatives: 9 | eligible: 8
Checking Nav2 path to candidate: x=..., y=..., euclidean=... m
Reachable frontier selected: x=..., y=..., euclidean=... m, path=... m
```

## Launch full baseline nhưng chưa cho robot chạy

```bash
ros2 launch frontier_exploration frontier_autonomy.launch.py
```

Manager sẽ báo:

```text
Autonomous navigation is disabled
```

## Bật autonomous navigation trong simulation

Chỉ bật khi simulation, SLAM và Nav2 đều đang ổn định và người vận hành chủ động cho phép robot mô phỏng di chuyển:

```bash
ros2 launch frontier_exploration frontier_autonomy.launch.py enable_navigation:=true
```

Khi bật, chu trình là:

```text
map -> frontier -> candidate -> path check -> NavigateToPose -> map update -> frontier mới
```

Sau một navigation goal thành công, manager chờ detector cập nhật map/frontier và nhận path mới. Goal vừa hoàn thành không được gửi lặp lại trong bán kính `goal_repeat_tolerance_m`.

Nếu navigation thất bại, goal đó không được tự gửi lại ngay. Blacklist/recovery nâng cao sẽ được bổ sung ở bước tiếp theo.

## Bước tiếp theo

- Chạy autonomous frontier loop trong Depot trước.
- Quan sát robot có đi tới nhiều frontier liên tiếp hay không.
- Bổ sung blacklist cho goal thất bại.
- Thêm stopping condition khi không còn frontier reachable.
- Thêm benchmark logging: coverage, thời gian, quãng đường, failed goal.

MapEx chưa nằm trong package này; MapEx sẽ được xây trên frontier baseline sau khi Stage 5 ổn định.
