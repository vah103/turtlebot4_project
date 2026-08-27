# Launch

Launch files mới dành riêng cho lộ trình nghiên cứu sẽ đặt ở đây.

## Stock reference stack

- `hospital_stock_stack.launch.py`
  - dùng Hospital simulation hiện có;
  - dùng launch/config mặc định của package `turtlebot4_navigation` cho SLAM;
  - dùng launch/config mặc định của package `turtlebot4_navigation` cho Nav2;
  - không dùng `hospital_slam.yaml`, `hospital_slam_no_loop.yaml` hay `nav2_hospital_override.yaml`.
  - chỉ dùng làm reference/debug cơ bản, không thay thế protocol benchmark Hospital v2.

Chạy trực tiếp từ repo:

```bash
ros2 launch mapex_hospital_research/launch/hospital_stock_stack.launch.py
```

Sau đó có thể chạy controller frontier riêng, ví dụ `control_tb4.py`.

## Research launches

Planned:
- `hospital_nearest.launch.py`
- `hospital_mapex.launch.py`
- nếu cần: launch riêng recorder/logger.

Không copy mù các launch cũ. Benchmark Nearest và MapEx phải tiếp tục dùng đúng cùng protocol/config đã chốt trong `EXPERIMENT_PROTOCOL.md`.

Existing dependencies có thể tham khảo trong `ros2_ws/src/frontier_exploration/launch/`.
