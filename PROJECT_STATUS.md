# Trạng thái project TurtleBot4

## 1. Mục tiêu project

Project cung cấp một workspace ROS 2 Jazzy tối giản để làm việc với TurtleBot4 Standard trong mạng lab. Trọng tâm hiện tại là kiểm tra kết nối, quan sát trạng thái robot và cảm biến, quản lý map, sau đó từng bước tiến tới localization và Nav2 theo quy trình an toàn.

Các công cụ trong repository phải hoạt động thụ động: chỉ subscribe dữ liệu trạng thái, không publish lệnh chuyển động và không gọi action hoặc service điều khiển robot.

## 2. Môi trường laptop

- Hệ điều hành: Ubuntu 24.04.
- ROS: ROS 2 Jazzy.
- Workspace: `ros2_ws` dùng `colcon` và `ament_python`.

## 3. Môi trường robot

- Robot: TurtleBot4 Standard.
- Namespace đã xác nhận: `/bot1`.
- Laptop và robot giao tiếp qua mạng lab.
- Topic và namespace của công cụ phải cấu hình được; không ghi cứng `/bot1` trong source code.

## 4. Những việc đã hoàn thành

- Kết nối ROS 2 trên laptop với robot qua mạng lab.
- Kiểm tra dữ liệu pin (`BatteryState`).
- Kiểm tra dữ liệu LiDAR (`LaserScan`).
- Kiểm tra odometry và cây TF.
- Chạy SLAM.
- Lưu map lab thành `maps/map_lab.pgm` và `maps/map_lab.yaml`.
- Tạo Git baseline đầu tiên cho project.

Repository hiện không chứa package triển khai SLAM hoặc Navigation; các lần chạy SLAM đã sử dụng phần mềm ROS 2/TurtleBot4 có sẵn ngoài source của project này.

## 5. Package và script hiện có

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

### Map hiện có

- `maps/map_lab.pgm`: occupancy image 159 x 395 pixel.
- `maps/map_lab.yaml`: metadata dùng ảnh `map_lab.pgm`, độ phân giải `0.050` m/pixel, origin `[-5.538, -10.321, 0]`, mode `trinary`.

## 6. Các sự cố đã gặp

- Trạng thái COMM/BATT từng tắt.
- `turtlebot4.service` đã được restart và hệ thống phục hồi.
- Khi SLAM khởi động, scan từng bị drop do TF hoặc queue chưa ổn định.

Các sự cố liên quan service hoặc robot chỉ được xử lý bởi người vận hành có thẩm quyền; công cụ tự động trong repository không tự chạy `systemctl`, SSH hoặc lệnh điều khiển robot.

## 7. Trạng thái hiện tại

- Kết nối ROS 2 qua mạng lab, dữ liệu battery, LiDAR, odometry và TF đã được kiểm tra.
- SLAM đã chạy và `map_lab` đã được lưu trong repository.
- Git baseline đầu tiên đã được tạo.
- Package giám sát thụ động và các script build/preflight cục bộ đã có sẵn.
- Namespace robot thực tế là `/bot1`; monitor vẫn dùng topic tương đối và nhận namespace qua cấu hình.
- Repository chưa chứa workflow localization hoặc Nav2 riêng.
- Map hiện có metadata kỹ thuật cần thiết cho map server nhưng chưa có metadata quản lý về thời gian, khu vực, robot/cảm biến và quy trình tạo map.

## 8. Việc tiếp theo

1. Duy trì Git baseline rõ ràng; đưa tài liệu trạng thái này vào thay đổi kế tiếp sau khi review.
2. Bổ sung metadata quản lý cho `map_lab`, gồm ngày tạo, khu vực, robot/cảm biến, phiên bản và quy trình tạo.
3. Thiết lập và kiểm tra localization bằng `map_lab`, sau khi xác nhận topic, namespace, TF và frame thực tế.
4. Thử Nav2 theo quy trình an toàn, bắt đầu bằng kiểm tra cấu hình và quan sát; chỉ cho phép chuyển động khi người vận hành phê duyệt rõ ràng và khu vực thử nghiệm đã an toàn.

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
