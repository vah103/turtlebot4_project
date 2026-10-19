# NR-RN-04 — Điều tra giới hạn tối ưu của MapEx + G1
**19/10/2026 · New Room, 2D native · Kết quả offline, KHÔNG phải run G2 tự chủ**

## 1. Câu hỏi
Liệu hành trình MapEx + Room Node G1 V4-R4.1 đã gần tối ưu đến mức G2 không còn khả năng giảm quãng đường? Phép kiểm tra dùng **run NR-RN-04** (Run04 gốc COM1 được tái chạy trên DELL với parity 25/25 decision), không dùng các run G2 thất bại để suy luận trực tiếp.

### Mốc đối chứng
- 709 bước native; 25 lần chọn goal; **199,107583 m** odometry và **51.304/53.389 ô P1** được quan sát.
- LOCAL: **104,179 m**; GLOBAL: **94,929 m** (cộng đúng quãng đường nguồn).
- SOURCE_TERMINAL = không còn frontier chọn được; **không** có nghĩa bảo đảm toàn GT được quan sát.
- Tín hiệu đi lại: **18,883 m (9,5%)** quãng đường nằm trong bán kính 0,8 m từ một vị trí đã tới trước đó ít nhất 15 m hành trình. Những đoạn này chỉ tạo thêm **20 ô mới**. Đây là **độ lặp đường**, không đồng nghĩa 18,883 m có thể loại bỏ: hành lang nối giữa phòng có thể bắt buộc phải đi qua nhiều lần.

## 2. Phòng 2 và chi phí của các goal còn sót

| Decision | Đoạn tick | Đường thực tế | Ô P1 mới |
|---|---:|---:|---:|
| D08–D10 | 157–207 | 13,014 m | 4.931 |
| D11 | 207–268 | 17,383 m | 128 |
| D12 | 268–292 | 6,818 m | 25 |
| **Tổng phòng 2** | 157–292 | **37,215 m** | **5.084** |

Tức là **24,201 m cuối** của đợt LOCAL phòng 2 chỉ tạo ra **153 ô mới (3,0%)**. Tuy nhiên, đoạn này cũng dịch chuyển robot về phía lối rời phòng; không được cộng cả 24,201 m vào “lãng phí”.

Phân bố scan thực: ở D11, **102/128 ô** đến từ hai scan tick **249–250** (67 + 35 ô). Ở D12, **24/25 ô** nằm ở tick **287–289**. Vì vậy không thể nói chỉ cần rời phòng ngay sau D10 vẫn giữ nguyên quan sát.

Những decision có mật độ quan sát thấp khác: D04 **5,765 m/28 ô**, D19 **5,689 m/62 ô**, D23 **5,318 m/36 ô**. Ngược lại, D13 GLOBAL **14,018 m/2.576 ô**, D24 GLOBAL **31,925 m/2.022 ô**: phải giữ sự đánh đổi giữa quan sát dọc đường và quãng đường.

## 3. Kiểm chứng counterfactual với LiDAR native thực

Tôi khôi phục đúng cảm biến MapEx trên dữ liệu P1: **2.500 tia, tầm 20 m**. Trước tiên, phát lại từ bản đồ observed trước mỗi đoạn và chạy nguyên đường gốc; **bản đồ thu được khớp bit-for-bit với snapshot native của run**. Sau đó chỉ thay đường trong năm đoạn; giữ nguyên vị trí đầu/cuối từng đoạn; ở ngoài các đoạn đó, giữ nguyên các vị trí thật của robot. LiDAR của phương án mới được quét lại **thực sự**, không chỉ cộng dồn vùng quan sát dự đoán.

**Quy tắc rất quan trọng:** Các viewpoint trên tuyến thay thế lấy từ hành trình gốc *trong tương lai*, và trong một lượt kiểm tra có sử dụng GT để ràng buộc khoảng cách tới vật cản. Đây là **offline oracle để đo tiềm năng**, tuyệt đối **không phải một policy G2 hợp lệ lúc robot đang chạy**. Không đưa GT vào G1/G2.

### 3.1. Tối ưu thuần quãng đường trên FREE đã biết

| Đoạn | Gốc | Đường thay thế | Chênh lệch |
|---|---:|---:|---:|
| D04 | 5,765 m | 5,012 m | 0,753 m |
| D11–D12 | 24,201 m | 19,630 m | 4,571 m |
| D13 | 14,018 m | 11,418 m | 2,600 m |
| D19 | 5,689 m | 4,718 m | 0,971 m |
| D24 | 31,925 m | 27,521 m | 4,404 m |

**Kết quả phát lại toàn tuyến, giữ cố định phần đường đi còn lại:** **185,808679 m**, **51.309 ô P1**, **không thiếu ô đã biết nào** của bản gốc. Tức ngắn **13,298904 m (6,68%)**.

**Nhưng không đủ điều kiện đánh giá công bằng về planner:** các đường ngắn này có những vị trí cách tường thật chỉ **0,20 m**, trong khi khoảng cách nhỏ nhất ở các đoạn gốc là **0,40–0,60 m**. Trọng số A* của MapEx cố ý né tường, không thuần tối ưu chiều dài. Đó là một yếu tố có giá trị, không được xóa bỏ mà không nói rõ.

### 3.2. Giữ **ít nhất** khoảng cách vật lý nhỏ nhất tới tường của đường gốc

Tôi chạy kiểm tra oracle thứ hai, dùng GT **chỉ để loại các đường vi phạm mức clearance vật lý của chính đoạn gốc**; mọi đường vẫn ở vùng observed-FREE tại **đầu đoạn**. Các min-clearance giới hạn của D04, D11–D12, D13, D19, D24 lần lượt là **0,6 / 0,5 / 0,5 / 0,4 / 0,6 m**.

| Đoạn | Gốc | Đường thay thế đạt min-clearance | Tiết kiệm |
|---|---:|---:|---:|
| D04 | 5,765 m | 5,489 m | 0,276 m |
| D11–D12 | 24,201 m | 19,925 m | 4,276 m |
| D13 | 14,018 m | 11,836 m | 2,182 m |
| D19 | 5,689 m | 4,947 m | 0,742 m |
| D24 | 31,925 m | 30,055 m | 1,870 m |

**Kết quả phát lại LiDAR trên toàn hành trình:** **189,761685 m**, **51.304 ô P1**, **không mất bất kỳ ô đã biết nào** của bản gốc, giảm **9,345898 m (4,69%)**. Có bốn ô khác nhau về **nhãn FREE/OCC** tại cuối run, nhưng tập ô đã biết giống nhau. Giữ *minimum wall distance* **không** đồng nghĩa có cùng mức rủi ro toàn hành trình, phân bố khoảng cách sát tường, footprint robot hoặc chi phí planning.

**LƯU Ý:** Phương án 189,76 m **không thể chạy online đúng nguyên dạng** vì ràng buộc GT-clearance và các waypoint dùng tri thức hồi cứu. Đây là bằng chứng tồn tại một đường *vật lý* tốt hơn dưới những ràng buộc chọn lọc; chưa chứng minh MapEx + G1 + G2 tự chủ làm được.

## 4. Vì sao MapEx gốc vẫn là đối chứng mạnh?

Tôi tính lại A* **với đúng trọng số né vật cản theo distance transform của MapEx**, không chỉ dùng đường ngắn đều trọng số. Với cùng các checkpoint của năm đoạn, đường sinh ra có chiều dài gần như chính đường gốc:

- D04: **5,765 m** (không tiết kiệm).
- D11–D12: **24,249 m** (dài hơn gốc ~0,048 m).
- D13: **14,018 m** (không tiết kiệm).
- D19: **5,842 m** (dài hơn gốc ~0,153 m).
- D24: **31,467 m** (ngắn hơn gốc ~0,458 m).

Vì vậy một G2 **giữ nguyên chính xác native weighted A*** **không thể mặc nhiên đạt 189,76 m** chỉ bằng thay thế những đoạn ngắn tôi đã dựng. Ở khía cạnh đến **cùng các mốc**, đường MapEx + G1 gần tối ưu với chính hàm chi phí đang dùng. Phần còn lại đáng nghiên cứu là **mốc viewpoint nào thực sự cần tới, lựa chọn và thứ tự các mốc**, chứ không cố cắt góc tường.

## 5. Thứ tự các phòng (hindsight only)

Giữ nguyên sáu cặp điểm vào/ra phòng và đoạn LOCAL bên trong từng phòng, chỉ đổi thứ tự nối các phòng trên **bản đồ observed ở cuối run**:

- Thứ tự thực: **1 → 2 → 3 → 4 → 5 → 6**.
- Thứ tự có đường nối ngắn hơn trong tập checkpoint cố định: **2 → 3 → 4 → 5 → 1 → 6**.
- A* chiều dài đều trọng số: tổng các đoạn nối **90,2 → 79,8 m**, chênh **10,4 m**.
- A* dùng **trọng số né tường gốc**: đường nối theo chính hàm chi phí đó **100,2 → 90,0 m**, chênh **10,2 m**.

Hai phép tính là **đường nối oracle** trên map cuối, không thể xem là quãng đường thực sẽ được tiết kiệm: robot chưa biết tất cả các phòng ở thời điểm đầu; thay tuyến hành lang sẽ làm khác quan sát và điều kiện nhận diện phòng. Không cộng 10,2 m hay 10,4 m với 9,35 m ở trên.

## 6. Ý nghĩa đối với G2

**Kết luận nghiên cứu:** Không thể nói MapEx + G1 đã tối ưu toàn cục. Dữ liệu native chứng minh tồn tại tuyến phát lại ngắn hơn và đạt mức observed coverage gốc. Nhưng chưa thể nói có **G2 hợp lệ, dùng thông tin tại thời điểm quyết định, giữ native planner và an toàn**, đã tiết kiệm 9–13 m.

Thứ tự kiểm thử hợp lý:

1. **Shadow theo epoch (không điều khiển robot):** Giữ sinh ứng viên bằng official G1 WFD, dùng cùng observed + LaMa mean/variance hiện tại. Đánh giá **hợp quan sát dọc cả tuyến**, không cộng trùng gain của nhiều goal. Kiểm tra đoạn D11–D12 và D24 nhưng **không hard-code** bất kỳ tọa độ nào.
2. **Thử nghiệm native giữ nguyên clearance-weighted A***: chỉ thay thứ tự/nhóm viewpoint, không thay hàm trọng số planner. Đây là phép thử chứng minh G2 thực sự giúp chứ không đơn giản đổi bài toán thành đi sát tường hơn.
3. **Nhánh ablation riêng** cho việc tối ưu độ dài dưới ràng buộc clearance *suy ra từ bản đồ hiện có* và bán kính an toàn của robot. Không dùng GT, không đặt ngưỡng riêng cho New Room. Phải đo an toàn, travel, coverage song song.
4. **Chỉ công nhận thắng** khi run native tự chủ kết thúc có biên nhận, ít nhất **51.304 ô P1**, đi **dưới 199,107583 m**, cùng start/LaMa/sensor/planning assumptions hoặc công khai ablation nếu đổi planner. Kiểm tra Hospital sau khi kết quả New Room hợp lệ.

## 7. Bằng chứng tái lập trên DELL

Thư mục duy nhất: `/home/dell/DELL_NATIVE2D/analysis/g2_autonomous_research_20261018/`

- `NR_RN_04_DEEP_RETURN_VIEWPOINT_AUDIT.json`: 25 decision, từng phòng, GLOBAL, đường lặp, thứ tự oracle.
- `NR_RN_04_EXACT_LIDAR_COUNTERFACTUAL_EVAL.json`: kiểm chứng bit-exact LiDAR riêng từng đoạn.
- `NR_RN_04_FULL_FIXED_TRAJECTORY_NATIVE_LIDAR_ORACLE.json`: kết quả oracle **185,808679 m**.
- `NR_RN_04_CLEARANCE_PLANNER_TRADEOFF.json`: so sánh uniform A* và original weighted A*.
- `NR_RN_04_SAME_CLEARANCE_FULL_NATIVE_ORACLE.json`: kết quả min-clearance oracle **189,761685 m**.
- `NR_RN_04_WEIGHTED_ROOM_ORDER_ORACLE.json`: kiểm tra thứ tự phòng với native clearance-weighted cost.
- `exact_lidar_counterfactual_eval.py`, `full_fixed_trajectory_native_lidar_oracle.py`, `full_fixed_same_clearance_native_oracle.py`: mã kiểm tra, **không** phải G2 runtime.

**SHA-256 bằng chứng chính:** 
- `NR_RN_04_FULL_FIXED_TRAJECTORY_NATIVE_LIDAR_ORACLE.json`: `1f17a471b7a5033f803500247a472a1dccbb22075637a139dea0baea42c6b17c`
- `NR_RN_04_SAME_CLEARANCE_FULL_NATIVE_ORACLE.json`: `82ed70e090155db7d6db5a18e3f642155b0dfba3321bc7991198ab9eb54a1091`

Đối chứng và các run lịch sử **không bị sửa**.