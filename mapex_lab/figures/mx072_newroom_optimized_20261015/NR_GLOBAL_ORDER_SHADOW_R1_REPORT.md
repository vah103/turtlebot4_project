# G2 GLOBAL — so sánh near-first và far-first (shadow R1)

**Ngày:** 19/10/2026  
**Dữ liệu:** NR-RN-04 (MapEx+G1), NR-G2PVU-02 (R1), NR-G2PVU-03 (R2). Đây là **shadow offline trên các decision GLOBAL đã lưu**: không điều khiển robot, không dùng GT hoặc các map từ tương lai để lập đường và chấm điểm.

## 1. Điều cần tránh: so hai đường có điểm kết thúc khác nhau

Từ vị trí S hiện tại, F là frontier GLOBAL đang được MapEx chọn, N là frontier **gần hơn theo native A***.

- Near-first: S → N → F, kết thúc tại F.
- Far-first: S → F → N, kết thúc tại N.

**Không thể so hai quãng đường này như hai chính sách cùng mục tiêu:** chúng có điểm kết thúc khác nhau.

Hai phép so có ý nghĩa:

1. **Cùng terminal F nếu N chắc chắn phải xử lý:** S → N → F so với S → F → N → F. Near-first sẽ có lợi thế hình học do tránh một chuyến quay lại, nhưng đây gần như là tính chất tam giác đường đi. **Không chứng minh** N chắc chắn còn cần đến sau khi tới F.
2. **Cùng terminal E là một frontier thứ ba ở snapshot hiện tại:** S → N → F → E so với S → F → N → E. Lúc này near-first **có thể thắng hoặc thua**, tùy E. Cũng chưa chứng minh cả ba goal vẫn cần sau các lần LiDAR sau.

Thêm vào đó đo **độ bất định UNKNOWN dự đoán được quan sát mới** trên hợp các vị trí dọc tuyến, không cộng trùng ô đã nhìn thấy từ S→F. Dự đoán dùng mean/variance LaMa cùng epoch, 180 tia xấp xỉ và một mẫu mỗi 50 ô đường đi; không phải LiDAR native thực hay phép đánh giá số học đầy đủ của MapEx.

## 2. Quy tắc, đầu vào và kiểm thử

- Frontier **chỉ** từ danh sách WFD official tại chính decision GLOBAL đó.
- N chỉ xét từ tập frontier có đường A* dự kiến **ngắn hơn** đường từ robot tới F; chọn tối đa 6 N gần nhất cho chi phí shadow, không có ngưỡng theo New Room.
- A* là trọng số native trên map observed trước decision: inflate 3×3, distance transform clipped trong [1,10], 4-neighbor. **UNKNOWN được native planner cho đi qua** theo cấu hình gốc, không tự đổi thành known FREE. Có cờ tách lộ trình đi qua UNKNOWN.
- Độ dài được đo trên tuyến A* có trọng số, **không phải** shortest Euclidean. Ở hai candidate R2 D20, tuyến qua N ngắn hơn trực tiếp khoảng 0,4 m về chiều dài hình học (do hàm chi phí né tường). Không dùng bất đẳng thức tam giác mặc định cho các đường này.
- Các đường tới chính F được tính lại và đối chiếu với `real_native_astar_rc` lưu trong run. **15/15 decision đã đưa vào phép so khớp từng ô đường A***.
- D01 (t0) không có snapshot pre-decision, các decision không có N gần hơn được bỏ qua. Ở R1 có một decision A* crop 203×263 khác đường native nên được loại và ghi dấu `NATIVE_PATH_MISMATCH`, không che giấu.

## 3. Kết quả toàn bộ ba run

| Run | GLOBAL decision | Có so thứ tự hợp lệ | N có thể xét | Kịch bản chung terminal E |
|---|---:|---:|---:|---:|
| MapEx + G1 gốc | 6 | 4 | 13 | 24 |
| PVU-R1 | 7 | 4 | 18 | 28 |
| PVU-R2 | 9 | 7 | 33 | 60 |
| **Tổng** | **22** | **15** | **64** | **112** |

Trong **112 kịch bản** với cùng endpoint E: near-first ngắn hơn **79**, far-first ngắn hơn **20**, bằng nhau **13**. Đây là các kịch bản *phụ thuộc nhau trên cùng snapshot*, **không** phải tỷ lệ thành công 79/112 của thuật toán.

**36/64** cặp được thử có cả hai chặng near↔far nằm hoàn toàn trong FREE đã biết tại thời điểm quyết định; các cặp khác đi qua UNKNOWN (được MapEx cho phép) nên cần đánh giá rủi ro/cập nhật khi chạy thật.

Thời gian tính bóng mẫu khoảng **0,15–0,21 giây/decision** (median trong từng run, sau khi bộ raycast numba đã sẵn sàng), không gồm LaMa. Chưa đo chi phí tích hợp online và worst-case, do đó không hứa overhead native.

## 4. D21 của PVU-R2: phép kiểm tra có tính phân biệt

Tại t571, robot S=[22,15] (core P1); frontier F=[167,243] là lựa chọn GLOBAL của MapEx. Cả N=[46,9], X=[11,97] và E=[20,65] đều có trong danh sách frontier cùng decision.

**Cùng terminal F:**

- S → F = **37,3 m**.
- S → N → F = **39,9 m** (chỉ +2,6m detour nếu N có ích).
- S → F → N → F = **111,1 m**. Chênh ~71,2m phản ánh giả định phải quay lại F, **không phải tiết kiệm chắc chắn**.

**Giữ terminal E=[20,65] (ứng viên thứ ba có thật cùng epoch):**

- Với N=[46,9]: near-first S→N→F→E = **73,6 m**; far-first S→F→N→E = **92,0 m** → near-first ngắn hơn **18,4 m**.
- Với X=[11,97]: near-first S→X→F→E = **89,8 m**; far-first S→F→X→E = **75,0 m** → **far-first ngắn hơn 14,8 m**.

Vậy **“luôn gần trước” là sai**, dù D21 có cơ hội hoãn-thăm rất rõ.

Trong shadow, tuyến N=[46,9] dự đoán nhìn thấy thêm **56 ô UNKNOWN** so với đi thẳng tới F; kiểm tra riêng bằng cảm biến native offline trước đây cho thấy thực tế chỉ **11 ô** — con số dự đoán bị cao hơn. Không được biến heuristic 180 tia thành ước lượng xác suất/cell-gain đã hiệu chuẩn; phải kiểm thử thêm nhiều decision/môi trường.

## 5. Kết luận thiết kế

**Đã chứng minh:** Trên các snapshot thực, cả frontier gần và xa đều đã có; A* native tạo đường, đôi khi khác biệt thứ tự rất lớn. Tại cùng điểm kết thúc E, có cả trường hợp near-first tốt hơn và far-first tốt hơn. PVU chỉ tối ưu LOCAL không giải quyết được việc sắp lịch GLOBAL.

**Chưa chứng minh:** đổi goal thực sự sẽ giảm 73,37m, hoặc vẫn đạt 51.304 observed cells / SOURCE_TERMINAL. Một ứng viên N có thể được quét từ S→F và biến mất trước khi cần quay lại. Các giả định so sánh đều chỉ trên snapshot hiện tại, không tái chạy MapEx + G1 + G2.

**Đề xuất thiết kế tiếp (chưa triển khai):** G2-GLOBAL-ORDER phải giữ endpoint/continuation cost rõ ràng, sử dụng expected *incremental unique visibility* khi thêm N, và ước lượng chi phí hoãn N sau khi tới F. Không đặt trọng số hay “always near-first” theo New Room. Trong shadow, báo cáo cả kịch bản N vẫn phải thăm và kịch bản N biến mất vì được quan sát trên đường. Giữ GLOBAL MapEx nguyên trạng cho tới khi có benchmark đa môi trường và native coverage parity.

## Tệp thực chứng

- [Bộ ảnh cuộn D21 theo 6 phương án](NR_GLOBAL_D21_NEAR_FAR_ORDER_SCROLL.html) — observed full map, các tuyến native A* và các endpoint.
- [Mã tính shadow đọc trực tiếp run](global_pair_order_shadow_r1.py).
- [Chi tiết 22 decision và 112 kịch bản](NR_GLOBAL_NEAR_FIRST_VS_FAR_FIRST_SHADOW.json).
- DELL: `/home/dell/DELL_NATIVE2D/analysis/g2_autonomous_research_20261018/`
- Mỗi run nguyên bản và receipt không bị sửa.

*GT không dùng làm input của script này; số liệu LiDAR thực ở D21 nói trên là phép kiểm chứng hậu nghiệm trước đây và không tham gia xếp hạng shadow.*
