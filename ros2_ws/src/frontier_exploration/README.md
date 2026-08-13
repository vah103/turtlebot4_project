# Frontier Exploration Baseline

ROS 2 package riêng cho Stage 5 của đồ án TurtleBot4.

Mục tiêu hiện tại:

1. Đọc `nav_msgs/OccupancyGrid` từ SLAM.
2. Phát hiện frontier cell tại biên `free`–`unknown`.
3. Gom frontier bằng connected components.
4. Lọc các cụm quá nhỏ.
5. Sau khi detector ổn định mới bổ sung representative goal, RViz markers, scoring và Nav2.

## Trạng thái hiện tại

`frontier_detector` là node **read-only**: chỉ subscribe map và in số frontier cell / cluster hợp lệ. Node chưa publish goal và chưa điều khiển robot.

## Build

```bash
cd ros2_ws
colcon build --packages-select frontier_exploration
source install/setup.bash
```

## Chạy detector

```bash
ros2 launch frontier_exploration frontier_detector.launch.py
```

Mặc định node đọc `/map`. Có thể đổi trong `config/frontier.yaml` để phù hợp namespace hoặc simulation setup.

## Các bước tiếp theo

- Chọn representative point an toàn cho mỗi frontier cluster.
- Hiển thị candidate frontier trên RViz.
- Thêm nearest-frontier baseline.
- Kiểm tra costmap / khả năng lập đường.
- Tích hợp Nav2 trong simulation.
- Thêm blacklist, stopping condition và benchmark logging.

MapEx chưa nằm trong package này; MapEx sẽ được xây trên frontier baseline sau khi Stage 5 chạy ổn định.
