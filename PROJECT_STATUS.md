# Trạng thái project TurtleBot4

*Cập nhật lần cuối: 2026-08-03, sau khi hoàn thành Stage 4 – Simulation & Scenarios.*

## 1. Trạng thái tổng quan

- **Stage 1 — Foundation & Inputs: hoàn thành.**
- **Stage 2 — Localization & Nav2: hoàn thành.**
- **Stage 3 — Navigation Benchmark: hoàn thành.**
- **Stage 4 — Simulation & Scenarios: hoàn thành.**
- **Stage 5 — Frontier Exploration Baseline: đang thực hiện.**
- Tiến độ kỹ thuật được ghi nhận trên Joy Dashboard: **42%**.

## 2. Môi trường đã xác nhận

### Robot thật

- Laptop Dell: Ubuntu 24.04, ROS 2 Jazzy, `ROS_DOMAIN_ID=0`.
- Robot: TurtleBot4 Standard.
- Namespace: `/bot1`.
- Localization: AMCL trên map đã lưu.
- Navigation: Nav2 với action `/bot1/navigate_to_pose`.

### Simulation

- Hai world đã xác minh: **Depot** và **Warehouse**.
- Robot model spawn và di chuyển có kiểm soát.
- Interface đã xác minh: LiDAR, RGB-D camera info, odometry và TF.
- RViz: trạng thái OK.
- Restart simulation: lặp lại thành công.
- Gazebo freeze: đã phục hồi thành công sau khi khởi động lại phiên mô phỏng.

## 3. Stage 1 và Stage 2

- OAK-D Pro RGB và RGB-D đã được xác minh, gồm tốc độ, độ trễ và kiểm tra ổn định 20 phút.
- SLAM, map saving, AMCL, TF và Nav2 đã được kiểm tra trên robot thật.
- Fresh-start Localization/Nav2 và docking đã thành công.
- Quy trình chi tiết được ghi trong `report/2026-07-29.md`.

## 4. Stage 3 — Navigation Benchmark

Benchmark chính thức trên robot thật sử dụng bốn goal cố định, chạy ba vòng:

- Tổng số lượt: `12`.
- Thành công: `12/12`.
- Success rate: `100%`.
- Failed trials: `0`.
- Recoveries: `0`.
- Tổng thời gian di chuyển: `101.11 s`.
- Trung bình: `8.43 s/goal`.
- Tổng quãng đường: `20.62 m`.
- Trung bình: `1.72 m/goal`.

Bộ bằng chứng gồm CSV, terminal log, rosbag metadata, map, goal configuration, checksum và summary. Báo cáo chi tiết nằm tại `report/2026-07-30.md`.

## 5. Stage 4 — Simulation & Scenarios

Ngày 2026-08-03, simulation được xác minh trong Depot và Warehouse:

- Robot model xuất hiện đúng và có thể di chuyển có kiểm soát.
- LiDAR, RGB-D camera info, odometry và TF có mặt cho pipeline phát triển.
- RViz và TF đạt yêu cầu của Stage 4.
- Simulation có thể dừng và khởi động lại lặp lại.
- Một lần Gazebo freeze đã được phục hồi thành công.
- Kết quả simulation chỉ dùng cho phát triển và kiểm thử có kiểm soát; không thay thế bằng chứng hiệu năng trên robot thật.

Báo cáo chi tiết nằm tại `report/2026-08-03.md`.

## 6. Trạng thái hiện tại — Stage 5

Project đã chuyển sang **Stage 5 — Frontier Exploration Baseline**. Focus hiện tại:

1. Định nghĩa frontier cell từ `nav_msgs/OccupancyGrid`.
2. Triển khai frontier-cell detection và clustering.
3. Tạo goal đại diện an toàn.
4. Publish RViz markers để kiểm tra detector.
5. Sau đó bổ sung geometric scoring và tích hợp Nav2 goal execution.

Completion gate của Stage 5 là có các lần exploration lặp lại được với coverage, thời gian, quãng đường, failed goal và stopping condition có thể đo.

## 7. Quy tắc an toàn

- Không publish `cmd_vel` hoặc gửi action chuyển động trên robot thật khi chưa có người vận hành tại lab.
- Kiểm tra pin, discovery, TF, LiDAR, odometry và khu vực an toàn trước mỗi phép thử robot thật.
- Dừng thử nghiệm khi localization mất ổn định, costmap bất thường hoặc pin xuống thấp.
- Không dùng kết quả simulation để tuyên bố hiệu năng thực tế khi chưa xác minh trên robot.
- Không commit các thư mục `build/`, `install/` và `log/` do colcon tạo ra.
