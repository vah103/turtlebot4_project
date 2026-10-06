# Checklist kiểm tra với TurtleBot4

Chỉ thực hiện các bước dưới đây khi laptop và robot đã được kết nối đúng mạng. Bắt đầu bằng các lệnh quan sát; không gửi lệnh chuyển động.

## 1. Môi trường và discovery

- [ ] Xác nhận `ROS_DOMAIN_ID` trên laptop và robot phù hợp.
- [ ] Chạy `ros2 node list` và lưu lại danh sách node quan sát được.
- [ ] Chạy `ros2 topic list -t` và lưu lại topic cùng kiểu message.
- [ ] Xác định namespace thực tế của robot, không giả định trước.

## 2. Dữ liệu cảm biến và trạng thái

- [ ] Tìm topic có kiểu `sensor_msgs/msg/BatteryState` và xem một message bằng `ros2 topic echo --once`.
- [ ] Tìm topic có kiểu `sensor_msgs/msg/LaserScan`; kiểm tra publisher và QoS bằng `ros2 topic info --verbose`.
- [ ] Tìm topic có kiểu `nav_msgs/msg/Odometry`; kiểm tra publisher và QoS.
- [ ] Kiểm tra `/tf` và `/tf_static` tồn tại, có publisher và kiểu message đúng.
- [ ] Xác nhận cây TF có các frame cần thiết: `map`, `odom`, `base_link`; ghi lại tên frame thực tế nếu có prefix/namespace.

## 3. Phần mềm trên robot/laptop

- [ ] Dùng `ros2 pkg list` để xác nhận các package TurtleBot4 đang được cài.
- [ ] Xác nhận riêng các package SLAM dự kiến sử dụng, nhưng chưa khởi chạy.
- [ ] Xác nhận riêng các package Nav2 dự kiến sử dụng, nhưng chưa khởi chạy.

## 4. Monitor thụ động

- [ ] Truyền namespace và tên topic đã xác nhận vào `status_monitor.launch.py`.
- [ ] Kiểm tra monitor nhận BatteryState, LaserScan và Odometry.
- [ ] Kiểm tra timestamp nhận gần nhất được cập nhật và không có publisher điều khiển nào được tạo.

## Điều kiện trước SLAM

Không chạy SLAM trước khi discovery, namespace, ba luồng dữ liệu, TF, frame và package liên quan ở trên đều đạt yêu cầu. Không chạy Navigation hoặc gửi `cmd_vel` trong checklist này.

