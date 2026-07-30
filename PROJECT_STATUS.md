# Trạng thái project TurtleBot4

*Cập nhật lần cuối: 2026-07-30, sau khi hoàn thành Stage 3 Navigation Benchmark trên robot thật.*

## 1. Trạng thái tổng quan

- **Stage 1 — Foundation & Inputs: hoàn thành.**
- **Stage 2 — Localization & Nav2: hoàn thành.**
- **Stage 3 — Navigation Benchmark: hoàn thành.**
- **Stage 4 — Simulation & Scenarios: chưa bắt đầu.**
- Tiến độ kỹ thuật theo checklist có trọng số: **32%**.
- Robot kết thúc phiên ở trạng thái đã dock.

## 2. Môi trường đã xác nhận

- Laptop Dell: Ubuntu 24.04, ROS 2 Jazzy, `ROS_DOMAIN_ID=0`.
- Robot: TurtleBot4 Standard.
- Namespace: `/bot1`.
- Localization: AMCL trên map đã lưu.
- Navigation: Nav2 với action `/bot1/navigate_to_pose`.

## 3. Stage 1 và Stage 2

- OAK-D Pro RGB và RGB-D đã được xác minh, gồm tốc độ, độ trễ và kiểm tra ổn định 20 phút.
- SLAM, map saving, AMCL, TF và Nav2 đã được kiểm tra trên robot thật.
- Fresh-start Localization/Nav2 và docking đã thành công.
- Quy trình chi tiết được ghi trong `report/2026-07-29.md`.

## 4. Stage 3 — Navigation Benchmark

Ngày 2026-07-30, phòng lab được quét và lưu lại thành map mới:

- Kích thước: `168 × 385` pixel.
- Độ phân giải: `0.05 m/pixel`.
- Map: `maps/lab_2026_07_30/lab_map.yaml` và `lab_map.pgm` trên workspace lab.

Benchmark chính thức sử dụng bốn goal cố định, chạy ba vòng:

- Tổng số lượt: `12`.
- Thành công: `12/12`.
- Success rate: `100%`.
- Failed trials: `0`.
- Recoveries: `0`.
- Tổng thời gian di chuyển: `101.11 s`.
- Trung bình: `8.43 s/goal`.
- Tổng quãng đường: `20.62 m`.
- Trung bình: `1.72 m/goal`.

Rosbag chính thức:

- Dung lượng: `28.0 MiB`.
- Thời lượng: `383.514 s`.
- Messages: `43,542`.
- Có AMCL pose, odometry, LiDAR, TF, Nav2 action, velocity, battery và diagnostics.

Bộ bằng chứng gồm CSV, terminal log, rosbag, map, goal configuration, checksum và summary. Báo cáo chi tiết nằm tại `report/2026-07-30.md`.

## 5. Trạng thái hiện tại

Navigation baseline trên robot thật đã được đóng băng và đủ dữ liệu để đối chiếu với các thuật toán khám phá sau này. Stage 3 không còn checkpoint kỹ thuật đang chờ.

Stage tiếp theo là **Stage 4 — Simulation & Scenarios**, chủ yếu thực hiện tại nhà:

1. Chuẩn bị TurtleBot4 simulation và robot model.
2. Xác minh LiDAR, RGB-D, odometry và TF mô phỏng.
3. Tạo các scenario exploration/risk có thể lặp lại.
4. Ghi rõ khác biệt simulation-to-real.

Sau Stage 4, project chuyển sang **Stage 5 — Frontier Exploration Baseline**.

## 6. Quy tắc an toàn

- Không publish `cmd_vel` hoặc gửi action chuyển động khi chưa có người vận hành tại lab.
- Kiểm tra pin, discovery, TF, LiDAR, odometry và khu vực an toàn trước mỗi phép thử robot thật.
- Dừng thử nghiệm khi localization mất ổn định, costmap bất thường hoặc pin xuống thấp.
- Không commit các thư mục `build/`, `install/` và `log/` do colcon tạo ra.
