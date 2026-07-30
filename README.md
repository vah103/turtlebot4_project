# TurtleBot4 Graduation Project

Repository cho đồ án **semantic-risk-aware autonomous exploration** sử dụng TurtleBot4 Standard, ROS 2 Jazzy, Nav2 và RGB-D.

## Trạng thái hiện tại

- Stage 1 — Foundation & Inputs: hoàn thành.
- Stage 2 — Localization & Nav2: hoàn thành.
- Stage 3 — Navigation Benchmark: hoàn thành.
- Stage 4 — Simulation & Scenarios: chưa bắt đầu.
- Tiến độ kỹ thuật theo checklist có trọng số: 32%.

Kết quả Stage 3 trên robot thật ngày 30/07/2026:

- 3 vòng × 4 goal cố định, tổng cộng 12 lượt.
- 12/12 lượt `SUCCEEDED`.
- Success rate: 100%.
- Recoveries: 0.
- Thời gian trung bình: 8.43 s/goal.
- Quãng đường trung bình: 1.72 m/goal.
- Rosbag chính thức: 43,542 messages trong 383.514 s.

Xem chi tiết tại `PROJECT_STATUS.md` và `report/2026-07-30.md`.

## Môi trường đã xác nhận

- Ubuntu 24.04
- ROS 2 Jazzy
- TurtleBot4 Standard
- Namespace `/bot1`
- `ROS_DOMAIN_ID=0`
- Localization bằng AMCL
- Navigation bằng Nav2

## Cấu trúc chính

```text
.joy/                         Metadata, roadmap và command library cho Joy
maps/                         Bản đồ đã version
report/                       Báo cáo theo ngày
ros2_ws/                      ROS 2 workspace và project tools
stage3_navigation_benchmark/  Runner, config và bằng chứng Stage 3
PROJECT_STATUS.md             Trạng thái kỹ thuật hiện tại
```

## Build và kiểm tra project tools

```bash
source /opt/ros/jazzy/setup.bash
cd ros2_ws
colcon build --symlink-install
source install/setup.bash
colcon test
colcon test-result --verbose
```

Các node trong `tb4_project_tools` chỉ giám sát dữ liệu. Những lệnh SLAM, Localization, Nav2, benchmark và docking được lưu riêng trong `.joy/commands.json` và báo cáo tương ứng.

## Stage 3 — Navigation Benchmark

Các file nhỏ cần được lưu trong Git:

```text
stage3_navigation_benchmark/scripts/navigation_benchmark.py
stage3_navigation_benchmark/config/benchmark_goals.yaml
stage3_navigation_benchmark/config/lab_map.yaml
stage3_navigation_benchmark/config/lab_map.pgm
stage3_navigation_benchmark/runs/stage3_20260730_official_02/
  baseline.sha256
  benchmark_goals.yaml
  benchmark_results_20260730_151159.csv
  benchmark_terminal.log
  lab_map.yaml
  lab_map.pgm
  rosbag_info.txt
  rosbag/metadata.yaml
  rosbag_terminal.log
  summary.txt
```

Không commit trực tiếp các payload rosbag lớn như `*.mcap` hoặc `*.db3`. Dùng Git LFS hoặc kho lưu trữ ngoài Git khi cần lưu lâu dài.

## Công việc tiếp theo

Stage 4 được thực hiện chủ yếu tại nhà:

1. Chọn và cài simulator tương thích ROS 2 Jazzy.
2. Chuẩn bị robot model, world và one-command launch.
3. Xác minh LiDAR, RGB-D, odometry và TF mô phỏng.
4. Tạo các scenario có thể reset và chạy lặp lại.
5. Ghi rõ khác biệt simulation-to-real.

## An toàn

Không gửi lệnh chuyển động, `cmd_vel`, Nav2 goal hoặc docking khi không có người vận hành tại lab. Luôn kiểm tra pin, TF, LiDAR, localization, costmap và khu vực xung quanh trước khi chạy robot thật.
