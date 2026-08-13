# Frontier Exploration Baseline

ROS 2 package riêng cho Stage 5 của đồ án TurtleBot4.

Mục tiêu hiện tại:

1. Đọc `nav_msgs/OccupancyGrid` từ SLAM.
2. Phát hiện frontier cell tại biên `free`–`unknown`.
3. Gom frontier bằng connected components.
4. Chia connected cluster dài thành nhiều frontier segment cục bộ.
5. Chọn một representative point trên mỗi segment.
6. Lấy pose robot từ TF và chọn representative gần nhất theo khoảng cách Euclid.
7. Publish RViz markers để kiểm tra trực quan trước khi tích hợp Nav2.

## Trạng thái hiện tại

`frontier_detector` vẫn là node **read-only** đối với robot: node chỉ subscribe map/TF, phát hiện frontier, chọn candidate gần nhất và publish marker trực quan. Node chưa publish navigation goal và chưa điều khiển robot.

Một connected frontier có thể kéo dài quanh gần toàn bộ vùng đã khám phá. Nếu lấy một centroid cho cả connected cluster thì chỉ sinh ra một candidate goal và representative dễ bị kéo về gần trung tâm bản đồ. Vì vậy detector hiện chia mỗi connected cluster thành nhiều **local frontier segment**.

Segmentation dùng region-growing trên chính frontier graph. Mỗi segment chỉ nhận các frontier cell liên thông nằm trong bán kính cấu hình `segment_radius_m` tính từ seed của segment. Các mảnh quá nhỏ được loại bằng `min_segment_size`.

Với mỗi segment, representative point được chọn bằng cách tính centroid trong grid rồi lấy **frontier cell gần centroid nhất**. Vì representative vẫn là một frontier cell nên nó nằm trên ô `free` của occupancy grid thay vì rơi trực tiếp vào vùng `unknown`.

Nearest-frontier baseline lấy pose robot từ TF `map -> base_link`, tính khoảng cách Euclid từ robot tới từng representative và chọn candidate có khoảng cách nhỏ nhất. Đây mới là bước **selection/scoring**; costmap, footprint, path feasibility và navigation goal vẫn chưa được dùng.

Các marker hiện có trên `/frontier_markers`:

- `frontier_cells`: toàn bộ frontier cells.
- `frontier_segment_centers`: tâm hình học của các segment.
- `frontier_representatives`: candidate representative point trên từng segment.
- `nearest_frontier_selected`: representative được nearest-frontier baseline chọn, hiển thị bằng marker đỏ lớn.

## Tham số hiện tại

```yaml
robot_frame: base_link
min_cluster_size: 5
segment_radius_m: 0.75
min_segment_size: 5
```

`segment_radius_m` là tham số chính để điều chỉnh mật độ candidate:

- giảm giá trị → nhiều segment / representative hơn;
- tăng giá trị → ít segment / representative hơn.

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

Mặc định node đọc `/map`, dùng frame robot `base_link` và publish `/frontier_markers`. Có thể đổi trong `config/frontier.yaml` để phù hợp namespace hoặc simulation setup.

Terminal sẽ báo dạng:

```text
Frontier cells: 533 | connected clusters: 1 | segments: 9 | representatives: 9 | selected: 1
Nearest frontier: x=..., y=..., distance=... m
```

## Hiển thị trên RViz

1. Mở RViz đang dùng cùng simulation/SLAM.
2. Chọn **Add**.
3. Chọn display type **MarkerArray**.
4. Đặt topic thành `/frontier_markers`.
5. Kiểm tra các representative màu xanh lá được phân bố dọc theo frontier.
6. Kiểm tra marker đỏ nằm trên representative gần robot nhất.

## Các bước tiếp theo

- Xác minh nearest-frontier selection trong Depot/Warehouse.
- Kiểm tra candidate bằng costmap / footprint / khả năng lập đường của Nav2.
- Chỉ sau khi feasibility ổn mới gửi navigation goal trong simulation.
- Thêm blacklist, stopping condition và benchmark logging.

MapEx chưa nằm trong package này; MapEx sẽ được xây trên frontier baseline sau khi Stage 5 chạy ổn định.
