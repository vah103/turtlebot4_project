# Frontier Exploration Baseline

ROS 2 package riêng cho Stage 5 của đồ án TurtleBot4.

Mục tiêu hiện tại:

1. Đọc `nav_msgs/OccupancyGrid` từ SLAM.
2. Phát hiện frontier cell tại biên `free`–`unknown`.
3. Gom frontier bằng connected components.
4. Chia connected cluster dài thành nhiều frontier segment cục bộ.
5. Chọn một representative point trên mỗi segment.
6. Lấy pose robot từ TF và sắp xếp candidate theo khoảng cách Euclid.
7. Dùng Nav2 `ComputePathToPose` để kiểm tra khả năng lập đường.
8. Chọn candidate gần nhất mà Nav2 xác nhận có path.

## Trạng thái hiện tại

`frontier_detector` **chưa điều khiển robot** và không gọi `NavigateToPose`. Node chỉ dùng action planning `ComputePathToPose` để hỏi Nav2 có lập được đường tới candidate hay không.

Một connected frontier có thể kéo dài quanh gần toàn bộ vùng đã khám phá. Nếu lấy một centroid cho cả connected cluster thì chỉ sinh ra một candidate goal và representative dễ bị kéo về gần trung tâm bản đồ. Vì vậy detector chia mỗi connected cluster thành nhiều **local frontier segment**.

Segmentation dùng region-growing trên frontier graph. Mỗi segment chỉ nhận các frontier cell liên thông nằm trong bán kính `segment_radius_m` tính từ seed. Các mảnh quá nhỏ được loại bằng `min_segment_size`.

Với mỗi segment, representative point được chọn bằng cách tính centroid trong grid rồi lấy **frontier cell gần centroid nhất**. Candidate quá gần robot được loại bằng `min_selection_distance_m`.

Các candidate còn lại được sắp xếp theo khoảng cách Euclid. Node hỏi Nav2 planner lần lượt từ candidate gần nhất. Nếu `ComputePathToPose` thất bại thì thử candidate kế tiếp. Chỉ candidate có path hợp lệ mới được đánh dấu là selected.

## Marker và path

Các marker trên `/frontier_markers`:

- `frontier_cells`: toàn bộ frontier cells.
- `frontier_segment_centers`: tâm hình học của các segment.
- `frontier_representatives`: candidate representative point trên từng segment.
- `planner_candidate_checking`: candidate Nav2 đang kiểm tra, màu vàng.
- `reachable_frontier_selected`: candidate gần nhất có path hợp lệ, màu đỏ.

Path được Nav2 trả về được publish tại:

```text
/frontier_selected_path
```

Topic này chỉ dùng để kiểm tra/hiển thị, không làm robot di chuyển.

## Tham số hiện tại

```yaml
robot_frame: base_link
planner_action: /compute_path_to_pose
planner_id: ""
min_cluster_size: 5
segment_radius_m: 0.75
min_segment_size: 5
min_selection_distance_m: 0.60
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

## Kiểm tra Nav2 planner

Trước khi chạy detector, kiểm tra action planning tồn tại:

```bash
ros2 action list | grep compute_path_to_pose
```

Cần thấy:

```text
/compute_path_to_pose
```

## Chạy detector

```bash
ros2 launch frontier_exploration frontier_detector.launch.py
```

Nếu planner chưa chạy, node sẽ chỉ chờ và báo:

```text
Waiting for Nav2 planner action /compute_path_to_pose
```

Khi planner hoạt động, terminal sẽ có dạng:

```text
Frontier cells: 533 | connected clusters: 1 | segments: 9 | representatives: 9 | eligible: 8
Checking Nav2 path to candidate: x=..., y=..., euclidean=... m
Reachable frontier selected: x=..., y=..., euclidean=... m, path=... m
```

Nếu candidate đầu tiên không reachable, node sẽ thử candidate kế tiếp.

## Hiển thị trên RViz

1. Giữ `MarkerArray` topic `/frontier_markers`.
2. Có thể thêm display type **Path**.
3. Đặt Path topic thành `/frontier_selected_path`.
4. Marker đỏ chỉ nên xuất hiện khi Nav2 đã trả về path hợp lệ.

## Các bước tiếp theo

- Xác minh reachability selection trong Depot/Warehouse.
- Nếu frontier cell nằm quá sát vùng unknown/inflated costmap, bổ sung safe goal projection vào vùng free bên trong.
- Sau khi candidate generation + feasibility ổn định mới tích hợp `NavigateToPose` trong simulation.
- Thêm blacklist, stopping condition và benchmark logging.

MapEx chưa nằm trong package này; MapEx sẽ được xây trên frontier baseline sau khi Stage 5 chạy ổn định.
