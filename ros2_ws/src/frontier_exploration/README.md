# Frontier Exploration Baseline

ROS 2 package riêng cho Stage 5 của đồ án TurtleBot4.

Mục tiêu hiện tại:

1. Đọc `nav_msgs/OccupancyGrid` từ SLAM.
2. Phát hiện frontier cell tại biên `free`–`unknown`.
3. Gom frontier bằng connected components.
4. Chia connected cluster dài thành nhiều frontier segment cục bộ.
5. Chọn một representative point trên mỗi segment.
6. Publish RViz markers để kiểm tra trực quan trước khi bổ sung scoring và Nav2.

## Trạng thái hiện tại

`frontier_detector` vẫn là node **read-only** đối với robot: node chỉ subscribe map, phát hiện frontier và publish marker trực quan. Node chưa publish navigation goal và chưa điều khiển robot.

Một connected frontier có thể kéo dài quanh gần toàn bộ vùng đã khám phá. Nếu lấy một centroid cho cả connected cluster thì chỉ sinh ra một candidate goal và representative dễ bị kéo về gần trung tâm bản đồ. Vì vậy detector hiện chia mỗi connected cluster thành nhiều **local frontier segment**.

Segmentation dùng region-growing trên chính frontier graph. Mỗi segment chỉ nhận các frontier cell liên thông nằm trong bán kính cấu hình `segment_radius_m` tính từ seed của segment. Các mảnh quá nhỏ được loại bằng `min_segment_size`.

Với mỗi segment, representative point được chọn bằng cách tính centroid trong grid rồi lấy **frontier cell gần centroid nhất**. Vì representative vẫn là một frontier cell nên nó nằm trên ô `free` của occupancy grid thay vì rơi trực tiếp vào vùng `unknown`. Costmap, footprint và khả năng lập đường của Nav2 sẽ được kiểm tra ở bước sau.

Các marker hiện có trên `/frontier_markers`:

- `frontier_cells`: toàn bộ frontier cells.
- `frontier_segment_centers`: tâm hình học của các segment.
- `frontier_representatives`: candidate representative point trên từng segment.

## Tham số hiện tại

```yaml
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

Mặc định node đọc `/map` và publish `/frontier_markers`. Có thể đổi trong `config/frontier.yaml` để phù hợp namespace hoặc simulation setup.

Terminal sẽ báo dạng:

```text
Frontier cells: 533 | connected clusters: 1 | segments: N | representatives: N
```

## Hiển thị trên RViz

1. Mở RViz đang dùng cùng simulation/SLAM.
2. Chọn **Add**.
3. Chọn display type **MarkerArray**.
4. Đặt topic thành `/frontier_markers`.
5. Kiểm tra các representative màu xanh lá được phân bố dọc theo frontier thay vì chỉ tập trung tại một điểm.

## Các bước tiếp theo

- Kiểm tra số lượng và vị trí representative trong Depot/Warehouse.
- Điều chỉnh `segment_radius_m` nếu candidate quá dày hoặc quá thưa.
- Thêm nearest-frontier scoring.
- Kiểm tra costmap / footprint / khả năng lập đường.
- Tích hợp Nav2 trong simulation.
- Thêm blacklist, stopping condition và benchmark logging.

MapEx chưa nằm trong package này; MapEx sẽ được xây trên frontier baseline sau khi Stage 5 chạy ổn định.
