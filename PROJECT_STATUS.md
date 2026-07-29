# Trạng thái project TurtleBot4

*Cập nhật lần cuối: 2026-07-29, sau phiên lab hoàn tất Stage 1 và Stage 2 và chuẩn bị baseline cho Stage 3.*

## 1. Trạng thái tổng quan

- **Stage 1 — Foundation & Inputs: hoàn thành.**
- **Stage 2 — Localization & Nav2: hoàn thành.**
- **Stage 3 — Navigation Benchmark: đang thực hiện.**
- Tiến độ kỹ thuật theo checklist có trọng số: **24%**.
- Robot kết thúc phiên ở trạng thái đã dock và đang sạc.

Phần đóng gói launch/config Localization–Nav2 thành package riêng được giữ dưới dạng maintenance follow-up; nó không còn chặn trạng thái hoàn thành của Stage 2 vì workflow đã được xác minh trên robot thật và được ghi lại trong report.

## 2. Môi trường đã xác nhận

- Laptop Dell: Ubuntu 24.04, ROS 2 Jazzy, `ROS_DOMAIN_ID=0`.
- Robot: TurtleBot4 Standard, Raspberry Pi Ubuntu 24.04.
- Robot IP trong phiên lab: `10.11.103.148`.
- Namespace: `/bot1`.
- LiDAR: `/bot1/scan`, khoảng `7.57 Hz`.
- Odometry: `/bot1/odom`, khoảng `19.87 Hz`.

## 3. Stage 1 — Kết quả hoàn thành

- Khôi phục OAK-D sau khi topic RGB từng có `Publisher count: 0`.
- Xác nhận OAK-D Pro kết nối ở USB SUPER và pipeline RGB hoạt động.
- Tạo cấu hình `oakd_pro_rgbd.yaml` và chạy pipeline `RGBD`.
- Xác nhận RGB:
  - khoảng `30 Hz`;
  - frame `oakd_rgb_camera_optical_frame`;
  - delay nền khoảng `30–32 ms`.
- Xác nhận depth:
  - topic `/bot1/oakd/stereo/image_raw`;
  - encoding `16UC1`;
  - độ phân giải `1280 × 720`;
  - khoảng `28.7 Hz`;
  - delay khoảng `95–101 ms`.
- Hoàn thành stability observation 20 phút:
  - RGB cuối bài khoảng `30.002 Hz`;
  - depth cuối bài khoảng `28.666 Hz`;
  - không ghi nhận node camera crash.

## 4. Stage 2 — Kết quả hoàn thành

- Chạy SLAM và lưu map mới:
  - kích thước `206 × 363`;
  - resolution `0.05 m/pixel`;
  - file `lab_map.pgm` và `lab_map.yaml`.
- Chạy Localization bằng AMCL và đặt 2D Pose Estimate.
- Xác nhận particle cloud hội tụ và LaserScan khớp map.
- Kích hoạt Navigation và Localization thành công.
- Thực hiện nhiều Nav2 goal thành công trong cùng phiên.
- Thực hiện fresh-start và ghi nhận:
  - `Begin navigating`;
  - `Reached the goal!`;
  - `Goal succeeded`.
- Dock robot thành công với `is_docked: true`.

Các cảnh báo `Control loop missed its desired rate` xuất hiện ngắn hạn nhưng không làm các goal được ghi nhận thất bại.

## 5. Stage 3 — Phần đã chuẩn bị

- Chọn bốn goal cố định trong frame `map`.
- Thiết lập `3 rounds × 4 goals`.
- Thiết lập settle time `2 s` và timeout `180 s`.
- Lưu checksum cho map, goal config và benchmark runner.
- Dry-run của `navigation_benchmark.py` thành công.
- Xác nhận `ros2 bag record` hỗ trợ output, regex và hidden topics.

Benchmark vật lý, CSV, rosbag và phần tính metrics chưa chạy do pin thấp.

## 6. Việc tiếp theo

1. Sạc robot lên ít nhất 60%.
2. Chạy một pilot ngắn để kiểm tra goal, orientation và logging.
3. Chạy đủ ba vòng với bốn goal cố định.
4. Lưu CSV, rosbag, terminal log và ảnh RViz.
5. Tính success rate, travel time, path length, recovery count và failure classification.
6. Freeze cấu hình Nav2 baseline chính thức bằng checksum và commit SHA.

## 7. Bằng chứng

- Report Git: `report/2026-07-29.md`.
- Bản Google Docs đầy đủ được liên kết trong report Git.
- Evidence cục bộ gồm stability log RGB/depth, cấu hình RGB-D, ảnh RGB/depth, map mới và log fresh-start Nav2.

## 8. Quy tắc an toàn

- Không tự publish `cmd_vel` hoặc gửi action chuyển động khi chưa có người vận hành tại lab.
- Kiểm tra pin, discovery, TF, LiDAR, odometry và khu vực an toàn trước mỗi phép thử.
- Dừng benchmark khi localization mất ổn định, costmap bất thường hoặc pin xuống thấp.
- Không commit các thư mục `build/`, `install/` và `log/` do colcon tạo ra.
