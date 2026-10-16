# MX072 Run 04 / D02 → VP1 → LaMa3 → Room Node V4-R4.1 → Dynamic G2 (2026-10-16)

## Mục đích và quy tắc không nhìn trước

Thử nghiệm **offline phản thực**: giữ nguyên goal VP1 đã chọn ở Run 04 Decision 02; giả lập robot di chuyển 4,5 m trên vùng known-free quan sát tại D02, quét 9 lần cách nhau 0,5 m rồi coi dữ liệu tại VP1 như một decision mới. Dùng ground truth **chỉ để tạo các LiDAR ray returns**, không truy cập GT trực tiếp trong LaMa, Room Node hoặc G2 lựa chọn goal mới. Không dùng quan sát nào của D03 trở đi.

Nguồn input D02: `room_r41/inference/decision_000002_{input,semantic}.npz` của Run04, robot native (647,611). VP1 đã khóa tại native (668,635), core (168,135), tức (13,5;16,8)m. Chi tiết phép quét/đường đi và kết quả trước đó được lưu trong cùng nhánh.

## 1. Bản đồ sau VP1

- Room 1 theo semantic D02 ban đầu: **3.889 unknown**. LiDAR offline dọc đường D02 → VP1 cho **2.868 ô mới**, gồm 2.822 free và 46 occupied; còn 1.021 unknown tính theo ranh giới phòng cũ.
- Bản đồ observed mới lưu ở DELL trong `/tmp/MX072_RUN04_D02_G2A_20261016/D02_VP1_RECONSTRUCTED_POSTSCAN_OBSERVED.npz`. Không sửa dữ liệu native Run04 gốc.
- Có 46 occupied mới trong phòng trước đây LaMa D02 chưa dự đoán là tường ở ngưỡng 0,5.

## 2. Chạy lại LaMa trên đúng observed mới

- G1/G2/G3 từ `/home/dell/MapEx/pretrained_models/weights/lama_ensemble/train_{1,2,3}`, CPU, lần lượt từng model; cùng transform `default_map_eval` và padding tự động căn về 208×272 pixel. Không load big-G.
- **Kiểm chứng parity**: tái tính lại snapshot D02 cũ từ cùng checkpoint, mean-map max absolute error **4,3213e-7**, variance-map max error **5,1409e-7** so với dữ liệu Run04, vì vậy pipeline đầu vào đã được căn chỉnh. Lượt thử padding thủ công 1.0 trước đó sai và **đã bị loại**, không dùng làm kết quả.
- Chỉ saved current-after-VP1 mean + variance từ phép inference đúng, không sao chép D03 mean hoặc heuristic bản đồ thực.

## 3. Room Node R4.1 sau VP1

- Chạy exact source R4.1 bằng sidecar qua **tracker state D01 → D02 → VP1 giả lập**, không khởi tạo tracker từ dữ liệu tương lai.
- Tái dựng D01 và D02 semantic có **0 pixel khác biệt** so với Run04 nguồn, trên cả hai snapshot.
- Room 1 nối với vị trí robot sau VP1 theo semantic mới: **6.398 ô**, gồm **5.639 observed free** và **759 unknown**.
- Không còn giả thuyết phòng mới được R4.1 chấp nhận tại snapshot VP1.
- Với 759 ô unknown thuộc Room 1 mới, **287 ô** có mean occupancy thay đổi hơn 0,1 so với dự đoán D02 ở cùng tọa độ. Trong số 759, 742 ô hiện có LaMa mean <0,5; 17 ô có mean ≥0,5.

## 4. G2 động: chọn goal mới từ VP1

- Tìm kiếm **486 ứng viên** thuộc connected room mới, observed knownfree, reachable qua 4-neighbor grid với inflation 2 ô xung quanh ô non-free, khoảng cách đến nonfree đủ lớn.
- Tính visibility danh định với observed occupied + LaMa mean occupancy ≥0,5 làm vật cản, chỉ cộng những ô unknown mới hiện được dự đoán free; thứ hạng ưu tiên độ bao phủ dự đoán và quãng đường.
- Đánh giá pathwise dự kiến trên các ứng viên đầu bằng scan mỗi 0,5m; điểm đạt gần maximum (trong 98% best gain) với chi phí đi lại thấp được chọn.
- **NEXT = native (692,671), core (192,171), (x,y)=(17,1;19,2)m**. Đường knownfree grid từ VP1 dự kiến **6,0m**.
- Kiểm tra lại 2.500 tia, tầm 20m, lấy mẫu 0,25-cell dọc đường, dự kiến nhìn **451/742 ô unknown được dự đoán free**, còn **291 predicted-free unknown**. Con số này là *predictive visibility* từ mean mới, **chưa phải actual/GT-based LiDAR**.
- Goal cũ VP2 native (671,632), (13,2;17,1)m, chỉ có **46 ô predicted free unknown** quan sát tại điểm đứng theo phép kiểm tra mới, nên **không giữ VP2 như goal cam kết**.
- Không chọn cố định goal tiếp theo sau NEXT: khi robot quan sát vật cản mới trong hành trình, cập nhật bản đồ và tái suy luận; đến NEXT chưa được phép tuyên bố room complete.

## 5. Giới hạn

Đây là thử nghiệm offline, không phải thực thi MapEx native hay hệ thống điều khiển robot. Đường đi mới là tuyến 4-connected trên free known với dilation 2 ô, không chứng minh an toàn footprint thực. Phép dự đoán visibility dùng nhị phân hóa LaMa mean và mô phỏng ray khác raycast probabilistic native; không thể tuyên bố global optimality hay runtime/saving thực tế. Bản đồ GT chỉ xuất hiện tại bước tạo sensor data đã hoàn tất trước LaMa inference và goal selection.

Artifact chính: `D02_VP1_NEW_LAMA_ROOM_G2_4PANEL_20261016.png`. Bảng bằng chứng machine-readable: `D02_VP1_DYNAMIC_G2_EVIDENCE_SUMMARY_20261016.json`. Dữ liệu trung gian nguồn trong thư mục riêng `/tmp/MX072_RUN04_D02_G2A_20261016/` của DELL.
