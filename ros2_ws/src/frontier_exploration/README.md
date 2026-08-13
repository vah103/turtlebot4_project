# Frontier Exploration Baseline

ROS 2 package riêng cho Stage 5 của đồ án TurtleBot4.

Mục tiêu hiện tại:

1. Đọc `nav_msgs/OccupancyGrid` từ SLAM.
2. Phát hiện frontier cell tại biên `free`–`unknown`.
3. Gom frontier bằng connected components.
4. Lọc các cụm quá nhỏ.
5. Publish RViz markers để kiểm tra trực quan frontier cell, centroid và representative point.
6. Sau khi representative point ổn định mới bổ sung scoring và Nav2.

## Trạng thái hiện tại

`frontier_detector` vẫn là node **read-only** đối với robot: node chỉ subscribe map, phát hiện frontier và publish marker trực quan. Node chưa publish navigation goal và chưa điều khiển robot.

Với mỗi cluster, representative point được chọn bằng cách tính centroid trong grid rồi lấy **frontier cell gần centroid nhất**. Vì representative vẫn là một frontier cell nên nó nằm trên ô `free` của occupancy grid thay vì rơi trực tiếp vào vùng `unknown`. Đây mới là kiểm tra an toàn ở mức occupancy map; costmap/footprint/Nav2 feasibility sẽ được kiểm tra ở bước sau.

Các marker hiện có trên `/frontier_markers`:

- `frontier_cells`: toàn bộ frontier cells.
- `frontier_cluster_centroids`: centroid hình học của các cluster hợp lệ.
- `frontier_representatives`: representative point được chọn trên chính frontier cell.

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
5. Kiểm tra frontier cell nằm trên ranh giới free–unknown.
6. Kiểm tra representative point nằm trên một frontier cell hợp lệ.

## Các bước tiếp theo

- Kiểm tra representative point trong Depot/Warehouse.
- Tách/điều chỉnh cluster nếu một frontier liên tục quá lớn tạo ra quá ít candidate goal.
- Thêm nearest-frontier baseline.
- Kiểm tra costmap / footprint / khả năng lập đường.
- Tích hợp Nav2 trong simulation.
- Thêm blacklist, stopping condition và benchmark logging.

MapEx chưa nằm trong package này; MapEx sẽ được xây trên frontier baseline sau khi Stage 5 chạy ổn định.
