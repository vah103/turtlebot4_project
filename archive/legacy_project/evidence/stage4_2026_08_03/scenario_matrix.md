# Stage 4 scenario matrix

Ngày xác minh: **2026-08-03**

| Hạng mục | Depot | Warehouse | Ghi chú |
|---|---|---|---|
| World khởi động | Verified | Verified | Hai world được mở và dùng cho kiểm tra TurtleBot4 simulation. |
| Robot model spawn | Verified | Verified | Robot xuất hiện trong môi trường. |
| Chuyển động có kiểm soát | Verified | Verified | Robot phản hồi với lệnh chuyển động trong phạm vi kiểm tra. |
| LiDAR | Verified | Verified | Có dữ liệu phục vụ occupancy/exploration. |
| RGB-D camera info | Verified | Verified | Camera information có mặt; đánh giá perception chi tiết thuộc Stage 6. |
| Odometry | Verified | Verified | Có trạng thái chuyển động của robot. |
| TF | Verified | Verified | Frame đủ để kiểm tra trong RViz và chuẩn bị goal/marker. |
| RViz | Verified | Verified | Robot, dữ liệu mô phỏng và frame có thể quan sát. |
| Restart repeatability | Verified | Verified | Simulation được dừng và khởi động lại thành công. |

## Sự cố đã quan sát

| Sự cố | Kết quả phục hồi | Ảnh hưởng đến kết luận |
|---|---|---|
| Một phiên Gazebo bị freeze | Đóng phiên bị treo và khởi động lại simulation stack thành công | Không chặn completion gate Stage 4; cần tiếp tục theo dõi trong các run Stage 5 dài hơn. |

## Completion gate

Stage 4 được đóng vì đã có:

- world và robot model sử dụng được;
- các interface tối thiểu cho frontier exploration;
- RViz/TF để kiểm tra trực quan;
- khả năng restart và phục hồi sau lỗi phiên mô phỏng;
- nguyên tắc simulation-to-real được ghi rõ.

Ma trận này không phải benchmark hiệu năng. Coverage, thời gian, quãng đường, failed goal và stopping condition sẽ được đo ở Stage 5.
