# Stage 4 evidence — 2026-08-03

Thư mục này là bộ evidence có kiểm soát cho **Stage 4 — Simulation & Scenarios**.

## Kết quả đã xác minh

- TurtleBot4 simulation chạy được trong Depot và Warehouse.
- Robot model spawn thành công và nhận chuyển động có kiểm soát.
- Có các interface cần thiết cho bước phát triển frontier exploration: LiDAR, RGB-D camera info, odometry và TF.
- RViz hiển thị được robot, dữ liệu mô phỏng và frame phục vụ kiểm tra.
- Simulation có thể dừng và khởi động lại lặp lại.
- Một lần Gazebo freeze đã được phục hồi bằng cách đóng phiên bị treo và khởi động lại simulation stack.

Nguồn mô tả chính: [`report/2026-08-03.md`](../../report/2026-08-03.md).

## Nội dung trong thư mục

- [`scenario_matrix.md`](scenario_matrix.md): ma trận các world và hạng mục đã xác minh.
- [`verification_commands.md`](verification_commands.md): lệnh kiểm tra và quy trình capture dùng cho lần chạy tái lập tiếp theo.

## Giới hạn bằng chứng

Phiên 03/08 không lưu raw terminal logs, rosbag hoặc ảnh RViz/Gazebo trong repository. Vì vậy thư mục này **không tuyên bố** các artifact đó đang tồn tại.

Các tài liệu ở đây có hai mục đích:

1. khóa phạm vi những gì đã thực sự được xác minh;
2. chuẩn hóa cách capture raw evidence ở lần chạy tiếp theo để Stage 5 có dữ liệu tái lập tốt hơn.

Khi có raw artifact mới, lưu dưới thư mục con theo ngày hoặc run ID và cập nhật manifest này. Không thay thế kết quả đã xác minh bằng dữ liệu giả lập hoặc mô tả không có nguồn.
