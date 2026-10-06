# Stage 4 verification and capture guide

Tài liệu này chuẩn hóa cách kiểm tra lại simulation và lưu raw evidence ở lần chạy tiếp theo. Tên topic dưới đây khớp với phiên kiểm tra Stage 4 đã ghi nhận; luôn xác nhận lại bằng `ros2 topic list` trước khi chạy.

## 1. Chuẩn bị thư mục run

```bash
RUN_ID="stage4_$(date +%Y%m%d_%H%M%S)"
OUT="$HOME/turtlebot4_project/evidence/stage4_2026_08_03/runs/$RUN_ID"
mkdir -p "$OUT"
```

## 2. Ghi môi trường ROS 2

```bash
{
  echo "date=$(date --iso-8601=seconds)"
  echo "ROS_DISTRO=${ROS_DISTRO:-unset}"
  echo "ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-unset}"
  ros2 node list
} | tee "$OUT/environment.txt"

ros2 topic list | sort | tee "$OUT/topics.txt"
```

## 3. Capture interface tối thiểu

```bash
ros2 topic echo /scan --once \
  | tee "$OUT/scan_once.txt"

ros2 topic echo /odom --once \
  | tee "$OUT/odom_once.txt"

ros2 topic echo /rgbd_camera/camera_info --once \
  | tee "$OUT/rgbd_camera_info_once.txt"
```

Đo rate trong một khoảng ngắn:

```bash
timeout 30 ros2 topic hz /scan \
  | tee "$OUT/scan_hz.txt"

timeout 30 ros2 topic hz /odom \
  | tee "$OUT/odom_hz.txt"
```

## 4. Kiểm tra TF

Điều chỉnh frame nếu simulation dùng tên khác:

```bash
timeout 15 ros2 run tf2_ros tf2_echo odom base_link \
  | tee "$OUT/tf_odom_base_link.txt"
```

Có thể xuất cây TF khi package hỗ trợ:

```bash
cd "$OUT"
ros2 run tf2_tools view_frames
```

## 5. Evidence hình ảnh

Chụp ít nhất:

- Gazebo với robot trong Depot;
- Gazebo với robot trong Warehouse;
- RViz hiển thị robot, LaserScan/OccupancyGrid và TF;
- ảnh sau khi restart thành công.

Tên file đề xuất:

```text
depot_gazebo.png
depot_rviz.png
warehouse_gazebo.png
warehouse_rviz.png
restart_verified.png
```

## 6. Kiểm tra restart

1. Ghi lại chính xác launch command đang dùng vào `launch_command.txt`.
2. Dừng simulation theo cách bình thường.
3. Xác nhận không còn process Gazebo/ROS 2 của phiên cũ.
4. Chạy lại cùng launch command.
5. Lặp lại các kiểm tra topic và TF.
6. Ghi kết quả vào `restart_notes.md`.

Không ghi một lệnh kill cố định vào repository vì tên process và simulator có thể thay đổi theo launch stack. Chỉ sử dụng lệnh dừng phù hợp với phiên đang chạy.

## 7. Tạo checksum

```bash
cd "$OUT"
find . -type f ! -name SHA256SUMS -print0 \
  | sort -z \
  | xargs -0 sha256sum > SHA256SUMS
sha256sum -c SHA256SUMS
```

## 8. Quy tắc kết luận

Một interface chỉ được đánh dấu `Verified` khi:

- command trả dữ liệu hợp lệ;
- file evidence được lưu;
- frame/topic name được ghi rõ;
- kết quả có thể chạy lại sau restart.

Các command trong tài liệu này là quy trình capture cho lần tái lập tiếp theo; chúng không thay thế raw logs chưa được lưu từ phiên 03/08.
