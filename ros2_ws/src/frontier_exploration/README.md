# Frontier Exploration Baseline

ROS 2 package riêng cho Stage 5 của đồ án TurtleBot4.

Mục tiêu hiện tại:

1. Đọc `nav_msgs/OccupancyGrid` từ SLAM.
2. Phát hiện frontier cell tại biên `free`–`unknown`.
3. Gom frontier bằng connected components.
4. Lọc các cụm quá nhỏ.
5. Publish RViz markers để kiểm tra trực quan frontier cell và tâm frontier cluster.
6. Sau khi detector ổn định mới bổ sung representative goal, scoring và Nav2.

## Trạng thái hiện tại

`frontier_detector` vẫn là node **read-only** đối với robot: node chỉ subscribe map, phát hiện frontier và publish marker trực quan. Node chưa publish navigation goal và chưa điều khiển robot.

Các marker hiện có:

- `/frontier_markers` → `frontier_cells`: toàn bộ frontier cells.
- `/frontier_markers` → `frontier_cluster_centers`: tâm hình học của các cluster hợp lệ.

## Build

```bash
cd ros2_ws
colcon build --packages-select frontier_exploration --symlink-install
source install/setup.bash
```

## Chạy detector

```bash
ros2 launch frontier_exploration frontier_detector.launch.py
```

Mặc định node đọc `/map` và publish `/frontier_markers`. Có thể đổi trong `config/frontier.yaml` để phù hợp namespace hoặc simulation setup.

## Hiển thị trên RViz

1. Mở RViz đang dùng cùng simulation/SLAM.
2. Chọn **Add**.
3. Chọn display type **MarkerArray**.
4. Đặt topic thành `/frontier_markers`.
5. Kiểm tra các điểm frontier nằm trên ranh giới free–unknown và cluster center nằm gần giữa mỗi cụm.

## Các bước tiếp theo

- Chọn representative point an toàn cho mỗi frontier cluster.
- Thêm nearest-frontier baseline.
- Kiểm tra costmap / khả năng lập đường.
- Tích hợp Nav2 trong simulation.
- Thêm blacklist, stopping condition và benchmark logging.

MapEx chưa nằm trong package này; MapEx sẽ được xây trên frontier baseline sau khi Stage 5 chạy ổn định.
