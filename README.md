# TurtleBot4 Graduation Project

Repository cho đồ án **semantic-risk-aware autonomous exploration** sử dụng TurtleBot4 Standard, ROS 2 Jazzy, Nav2 và RGB-D.

## Trạng thái hiện tại

- Stage 1 — Foundation & Inputs: hoàn thành.
- Stage 2 — Localization & Nav2: hoàn thành.
- Stage 3 — Navigation Benchmark: hoàn thành.
- Stage 4 — Simulation & Scenarios: hoàn thành.
- Stage 5 — Frontier Exploration Baseline: đang thực hiện.
- Tiến độ kỹ thuật theo checklist có trọng số: **42%**.

Nguồn trạng thái chuẩn trong repository:

- `.joy/project.json`
- `.joy/roadmap.json`
- `PROJECT_STATUS.md`

Kiểm tra tính nhất quán bằng:

```bash
python3 .github/scripts/validate_project_state.py
```

## Kết quả đã xác minh

### Stage 3 — Navigation Benchmark

Benchmark trên robot thật ngày 30/07/2026:

- 3 vòng × 4 goal cố định, tổng cộng 12 lượt.
- 12/12 lượt `SUCCEEDED`.
- Success rate: 100%.
- Recoveries: 0.
- Thời gian trung bình: 8.43 s/goal.
- Quãng đường trung bình: 1.72 m/goal.
- Rosbag chính thức: 43,542 messages trong 383.514 s.

Chi tiết: `report/2026-07-30.md` và `stage3_navigation_benchmark/`.

### Stage 4 — Simulation & Scenarios

Simulation được xác minh ngày 03/08/2026:

- Hai world: Depot và Warehouse.
- Robot spawn và di chuyển có kiểm soát.
- LiDAR, RGB-D camera info, odometry và TF có mặt.
- RViz hoạt động ở trạng thái sử dụng được.
- Restart simulation lặp lại thành công.
- Một lần Gazebo freeze đã được phục hồi bằng cách khởi động lại phiên mô phỏng.

Chi tiết:

- `report/2026-08-03.md`
- `evidence/stage4_2026_08_03/README.md`
- `evidence/stage4_2026_08_03/scenario_matrix.md`
- `evidence/stage4_2026_08_03/verification_commands.md`

Bộ evidence này chỉ ghi những kết quả đã được xác minh. Raw terminal logs và ảnh chụp chưa được lưu trong repository nên không được mô tả như thể đã tồn tại.

## Môi trường đã xác nhận

### Robot thật

- Ubuntu 24.04
- ROS 2 Jazzy
- TurtleBot4 Standard
- Namespace `/bot1`
- `ROS_DOMAIN_ID=0`
- Localization bằng AMCL
- Navigation bằng Nav2

### Simulation

- Gazebo/TurtleBot4 simulation
- Depot và Warehouse
- LiDAR, RGB-D camera info, odometry và TF
- RViz để kiểm tra robot, dữ liệu cảm biến và frame

## Cấu trúc chính

```text
.joy/                         Metadata, roadmap và command library cho Joy
evidence/                     Bộ bằng chứng và tài liệu tái lập theo stage
data/lama_runs/               Snapshot LaMa đầy đủ, chỉ lưu local (Git ignored)
data/lama_samples/            Ba mẫu LaMa đại diện đã kiểm tra và lưu Git
docs/lama_local_layout.md     Chuẩn đường dẫn và cách migrate LaMa
maps/                         Bản đồ đã version
report/                       Báo cáo theo ngày
ros2_ws/                      ROS 2 workspace và project tools
scripts/lama/                 Script migrate/setup LaMa
stage3_navigation_benchmark/  Runner, config và bằng chứng Stage 3
third_party/lama/             LaMa runtime/model local (Git ignored)
third_party/lama_upstream/    Mã nguồn LaMa được ghim bằng Git submodule
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

## Công việc hiện tại — Stage 5

1. Định nghĩa frontier cell từ `nav_msgs/OccupancyGrid`.
2. Triển khai frontier-cell detection và clustering.
3. Chọn điểm đại diện an toàn cho từng cụm.
4. Publish RViz markers để kiểm tra detector.
5. Bổ sung geometric scoring.
6. Gửi safe frontier goal qua Nav2.
7. Đánh giá coverage, thời gian, quãng đường, failed goal và stopping condition.

## Chính sách lưu bằng chứng

- Commit code, config, CSV, text log, checksum, metadata và ảnh minh chứng có kích thước hợp lý.
- Không commit trực tiếp payload rosbag lớn như `*.mcap` hoặc `*.db3`.
- Không commit full LaMa run, upstream checkout hoặc model checkpoint; dùng
  `data/lama_runs`, `third_party/lama` và `models/lama` ở local.
- Dùng Git LFS hoặc kho lưu trữ ngoài Git cho dữ liệu lớn.
- Không đánh dấu checkpoint hoàn thành nếu repository không có report hoặc evidence tương ứng.

## An toàn

Không gửi lệnh chuyển động, `cmd_vel`, Nav2 goal hoặc docking khi không có người vận hành tại lab. Luôn kiểm tra pin, TF, LiDAR, localization, costmap và khu vực xung quanh trước khi chạy robot thật.
