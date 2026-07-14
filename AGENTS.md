# Quy tắc an toàn cho repository

- Không tự sử dụng `sudo`.
- Không tự SSH vào robot hoặc máy khác.
- Không sửa file hệ thống.
- Không gửi bất kỳ lệnh chuyển động nào tới robot.
- `cmd_vel` và mọi biến thể có namespace của topic này bị cấm nếu chưa có phê duyệt rõ ràng của người vận hành.
- Ưu tiên các kiểm tra chỉ đọc đối với ROS graph, topic, TF và trạng thái hệ thống.
- Namespace và tên topic phải cấu hình được; không ghi cứng namespace của robot.
- Không commit các thư mục sinh ra bởi colcon: `build`, `install`, `log`.
- Công cụ giám sát trong repository chỉ được subscribe, không được publish hoặc gọi action/service điều khiển.

