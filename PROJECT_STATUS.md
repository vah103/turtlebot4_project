# Trạng thái project TurtleBot4

*Cập nhật lần cuối: 2026-07-24, sau commit `975d338` bổ sung map và ảnh RViz của phiên lab ngày 2026-07-23.*

## 1. Mục tiêu project

Project cung cấp một workspace ROS 2 Jazzy tối giản để làm việc với TurtleBot4 Standard trong mạng lab. Trọng tâm hiện tại là kiểm tra kết nối và cảm biến, quản lý map, thử nghiệm Localization và Nav2 trên robot thật, sau đó từng bước xây dựng workflow riêng cho đồ án theo quy trình an toàn và có thể tái lập.

Các công cụ giám sát mặc định trong repository phải hoạt động thụ động: chỉ subscribe dữ liệu trạng thái, không publish lệnh chuyển động và không gọi action hoặc service điều khiển robot.

## 2. Môi trường laptop

- Hệ điều hành: Ubuntu 24.04.
- ROS: ROS 2 Jazzy.
- Workspace: `ros2_ws` dùng `colcon` và `ament_python`.
- Laptop Dell chạy SLAM, Localization, Nav2 và RViz bằng các package TurtleBot4/Nav2 đã cài trong hệ thống.

## 3. Môi trường robot

- Robot: TurtleBot4 Standard.
- Namespace đã xác nhận: `/bot1`.
- Laptop và robot giao tiếp qua mạng lab.
- Topic và namespace của công cụ phải cấu hình được; không ghi cứng `/bot1` trong source code.
- Các topic đã xác nhận gồm `/bot1/battery_state`, `/bot1/scan`, `/bot1/odom`, `/bot1/tf`, `/bot1/tf_static`, `/bot1/dock_status` và các topic/action liên quan Nav2.

## 4. Những việc đã hoàn thành

- Kết nối ROS 2 trên laptop với robot qua mạng lab.
- Kiểm tra dữ liệu pin (`BatteryState`).
- Kiểm tra dữ liệu LiDAR (`LaserScan`).
- Kiểm tra odometry và cây TF.
- Khắc phục LiDAR không quay bằng service `/bot1/start_motor`.
- Xác nhận LiDAR phát dữ liệu ổn định khoảng `7.58 Hz`.
- Chạy SLAM và điều khiển robot đi quét khu vực lab.
- Lưu map lab ban đầu thành `maps/map_lab.pgm` và `maps/map_lab.yaml`.
- Tạo map mới ngày 2026-07-23 và đưa vào repository dưới tên `maps/lab_2026_07_23.pgm` và `maps/lab_2026_07_23.yaml`.
- Lưu hai ảnh RViz minh họa SLAM và Nav2 trong `docs/images/2026-07-23/`.
- Chạy Localization bằng AMCL với map đã lưu.
- Đặt initial pose bằng **2D Pose Estimate** và xác nhận dữ liệu LiDAR khớp với map.
- Chạy Nav2, kích hoạt đầy đủ lifecycle và điều hướng robot tới nhiều goal thành công.
- Cho robot rời dock thành công bằng action `/bot1/undock`.
- Tạo Git baseline đầu tiên cho project và duy trì báo cáo tiến độ theo ngày.

Repository hiện chưa chứa package triển khai SLAM, Localization hoặc Navigation riêng. Các phiên thử nghiệm đã sử dụng phần mềm ROS 2/TurtleBot4/Nav2 có sẵn ngoài source riêng của project này.

## 5. Package, script và dữ liệu hiện có

### Package ROS 2

`tb4_project_tools` là package `ament_python` phiên bản `0.1.0`, cung cấp executable `robot_status_monitor`.

Monitor chỉ subscribe các loại dữ liệu sau:

- `sensor_msgs/msg/BatteryState` trên topic tương đối `battery_state`.
- `sensor_msgs/msg/LaserScan` trên topic tương đối `scan`.
- `nav_msgs/msg/Odometry` trên topic tương đối `odom`.

Package gồm:

- `tb4_project_tools/robot_status_monitor.py`: node giám sát thụ động.
- `launch/status_monitor.launch.py`: launch file hỗ trợ namespace, topic, QoS pin và simulated time có thể cấu hình.
- `config/status_monitor.yaml`: cấu hình mặc định cho monitor.
- `test/test_monitor_source.py`: test trạng thái topic, QoS và kiểm tra source không tạo publisher/service/client/action điều khiển.
- `package.xml`, `setup.py`, `setup.cfg` và resource marker: metadata và cấu hình build/install.

### Script

- `scripts/check_environment.sh`: kiểm tra ROS 2 Jazzy, `ros2`, `colcon`, Python và các module cần thiết.
- `scripts/build_workspace.sh`: build workspace bằng `colcon build --symlink-install`.
- `scripts/local_preflight.sh`: kiểm tra môi trường, build, compile-check và chạy test cục bộ mà không cần robot.

### Map trong repository

- `maps/map_lab.pgm`.
- `maps/map_lab.yaml`: độ phân giải `0.050` m/pixel, origin `[-5.538, -10.321, 0]`, mode `trinary`.
- `maps/lab_2026_07_23.pgm`.
- `maps/lab_2026_07_23.yaml`: độ phân giải `0.050` m/pixel, origin `[-9.128, -4.808, 0]`, mode `trinary`.

### Ảnh minh họa trong repository

- `docs/images/2026-07-23/slam-map-rviz.png`.
- `docs/images/2026-07-23/nav2-success-rviz.png`.

Map mới và hai ảnh RViz được thêm trong commit `975d338` ngày 2026-07-24.

## 6. Các sự cố đã gặp

### COMM/BATT từng tắt

- `turtlebot4.service` đã được restart và hệ thống phục hồi.
- Nguyên nhân gốc chưa được xác định chắc chắn.

### Scan từng bị drop khi SLAM khởi động

- Xuất hiện tình trạng queue hoặc TF chưa ổn định.
- Sau khi kiểm tra và remap đúng `/tf` cùng `/tf_static`, SLAM đã tạo được map.

### LiDAR có publisher nhưng motor không quay

- `/bot1/scan` có publisher nhưng không có bản tin thực và LiDAR không quay.
- `/dev/RPLIDAR` vẫn trỏ tới `/dev/ttyUSB0`.
- Gọi `/bot1/start_motor` giúp LiDAR quay lại và phát dữ liệu khoảng `7.58 Hz`.

### Diagnostics còn lỗi ở Camera, Mouse và Joystick

- Battery, Dock, Hazards, IMU, Wheels và Lidar báo `OK`.
- Camera, Mouse và Joystick từng báo không có dữ liệu.
- Các lỗi này chưa ngăn bài thử SLAM bằng LiDAR và Nav2, nhưng chưa được xử lý triệt để.

### Nav2 lifecycle chưa active đầy đủ

Kết quả ban đầu:

```text
planner_server: inactive [2]
controller_server: active [3]
bt_navigator: inactive [2]
```

Sau khi reset và startup lại qua `/bot1/lifecycle_manager_navigation/manage_nodes`, Nav2 đã lập đường và điều khiển robot tới nhiều goal thành công.

### Một goal Nav2 bị abort

- Một goal trả `status: 6` (`ABORTED`).
- Các goal khác ở vùng trống chạy thành công.
- Nguyên nhân của goal riêng lẻ có thể liên quan costmap, vật cản hoặc vị trí đích, nhưng chưa được chẩn đoán chi tiết.

### Dock action trên laptop chưa dùng được

- Khi gọi action dock trên laptop Dell, CLI báo `The passed action type is invalid`.
- Undock đã chạy thành công từ Raspberry Pi của robot.
- Docking tự động cuối phiên ngày 2026-07-23 chưa được xác nhận.

Các sự cố liên quan service hoặc robot chỉ được xử lý bởi người vận hành có thẩm quyền; công cụ tự động trong repository không tự chạy `systemctl`, SSH hoặc lệnh điều khiển robot.

## 7. Trạng thái hiện tại

- Kết nối ROS 2 qua mạng lab hoạt động.
- Dữ liệu battery, LiDAR, odometry và TF đã được kiểm tra.
- LiDAR đã được khôi phục sau sự cố motor không tự quay.
- SLAM đã chạy và map đã được lưu.
- Localization bằng AMCL đã được kiểm tra thành công trên robot thật.
- Transform cần thiết `map -> odom -> base_link` đã hoạt động trong phiên Navigation.
- Nav2 đã được kiểm tra thành công với nhiều goal.
- Map mới và ảnh RViz ngày 2026-07-23 đã có trong repository.
- Git baseline và workflow báo cáo hằng ngày đang hoạt động.
- Package `tb4_project_tools` và các script build/preflight vẫn chỉ phục vụ giám sát thụ động.
- Repository chưa có launch/config riêng để tái lập toàn bộ Localization và Nav2 chỉ bằng source trong project.
- Camera, Mouse, Joystick và docking tự động còn cần kiểm tra.

## 8. Việc tiếp theo

1. Bổ sung metadata quản lý cho `lab_2026_07_23`: khu vực, robot, cảm biến, giới hạn và quy trình tạo.
2. Lặp lại quy trình từ trạng thái khởi động mới để kiểm tra tính tái lập của Localization và Nav2.
3. Điều tra diagnostics của Camera và Mouse.
4. Kiểm tra docking tự động và chuẩn hóa cách gọi action từ laptop.
5. Tạo launch/config riêng cho Localization và Nav2 sau khi quy trình thủ công đã ổn định.
6. Bắt đầu các bước đồ án nâng cao sau khi baseline Navigation ổn định, gồm frontier exploration, semantic layer và logic nhiệm vụ theo phạm vi đã chốt.

## 9. Các quy tắc an toàn quan trọng

- Không tự sử dụng `sudo`.
- Không tự SSH vào robot hoặc máy khác.
- Không tự chạy `systemctl` hoặc sửa file hệ thống.
- Không gửi bất kỳ lệnh chuyển động nào tới robot.
- Không publish vào `cmd_vel` hoặc bất kỳ biến thể có namespace nào nếu chưa có phê duyệt rõ ràng của người vận hành.
- Ưu tiên kiểm tra chỉ đọc đối với ROS graph, topic, QoS, TF và trạng thái hệ thống.
- Namespace và tên topic phải cấu hình được; không ghi cứng namespace robot trong source.
- Công cụ giám sát chỉ được subscribe; không được publish hoặc gọi action/service điều khiển.
- Không commit các thư mục do colcon sinh ra: `ros2_ws/build`, `ros2_ws/install`, `ros2_ws/log`.
- Không chạy SLAM trước khi discovery, namespace, battery, LiDAR, odometry, TF, frame và package liên quan đã được xác nhận.
- Không chạy Navigation hoặc thử chuyển động khi chưa có kế hoạch thử nghiệm, vùng an toàn và sự phê duyệt của người vận hành.
