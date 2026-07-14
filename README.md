# TurtleBot4 Project Tools

Workspace ROS 2 Jazzy tối giản dành cho các công cụ giám sát thụ động của TurtleBot4 Standard. Hiện tại project không chứa SLAM, Navigation hay chức năng điều khiển chuyển động.

## Yêu cầu

- Ubuntu 24.04
- ROS 2 Jazzy đã được cài và source
- `colcon` có sẵn trong môi trường ROS 2

Không cần robot để build, test hoặc chạy thử node. Khi không có message, monitor chỉ ghi log chờ và tiếp tục hoạt động.

## Kiểm tra và build

```bash
./scripts/check_environment.sh
./scripts/build_workspace.sh
./scripts/local_preflight.sh
```

Hoặc thực hiện thủ công:

```bash
source /opt/ros/jazzy/setup.bash
cd ros2_ws
colcon build --symlink-install
source install/setup.bash
colcon test
colcon test-result --verbose
```

## Chạy monitor

Các topic mặc định là tên tương đối, vì vậy chúng tự tuân theo namespace được truyền vào:

```bash
source /opt/ros/jazzy/setup.bash
source ros2_ws/install/setup.bash
ros2 launch tb4_project_tools status_monitor.launch.py
```

Ví dụ cấu hình khi đã xác nhận namespace và topic thực tế:

```bash
ros2 launch tb4_project_tools status_monitor.launch.py \
  namespace:=ROBOT_NAMESPACE \
  battery_topic:=battery_state \
  scan_topic:=scan \
  odom_topic:=odom
```

Không thêm dấu `/` đầu topic nếu muốn topic nằm dưới namespace. Node này chỉ tạo subscription cho `sensor_msgs/msg/BatteryState`, `sensor_msgs/msg/LaserScan` và `nav_msgs/msg/Odometry`; nó không tạo publisher.

