# Vì sao G2-PVU-R2 tăng 73,37 m GLOBAL? — Truy nguyên native 2D

**Ngày 19/10/2026 · Dữ liệu cố định trên DELL · New Room P1 0,1 m**  
**Đối chứng:** NR-RN-04 MapEx + G1 V4-R4.1, SOURCE_TERMINAL.  
**Thử nghiệm:** NR-G2PVU-03 MapEx + G1 + G2-PVU-R2, SOURCE_TERMINAL, D01–D02 parity PASS.

## Kết quả chắc chắn

| | MapEx + G1 | G2-PVU-R2 | Chênh lệch |
|---|---:|---:|---:|
| Odometry | 199,107583 m | 265,756112 m | +66,648529 m |
| GLOBAL | 94,928870 m | 168,302320 m | **+73,373450 m** |
| LOCAL | 104,178713 m | 97,453792 m | −6,724921 m |
| Số ô P1 đã quan sát | 51.304 | 51.304 | 0 |
| Decision | 25 | 27 | +2 |

Con số LOCAL giảm 6,72 m **không phải so sánh hiệu quả cùng một tập phòng**: một vùng gốc khám phá trong LOCAL (phòng 4 của run gốc) được R2 đi qua khi GLOBAL. Không suy luận “R2 tốt hơn trong phòng” chỉ từ tổng mode.

## 1. Hai chuyến quay lại muộn chiếm phần lớn sai khác

- R2 D23, tick 705–819: **33,66 m GLOBAL**, **271 ô**; đi từ vùng dưới-phải về giữa-trên.
- R2 D27, tick 875–936: **17,01 m GLOBAL**, **11 ô**; từ giữa-trên trở về trái.
- Tổng **50,67 m**, tương đương **69,1% chênh lệch GLOBAL 73,37 m**. Các GLOBAL trước đó có chênh lệch ròng **22,70 m**. Đây là phân rã *số học*, **không khẳng định** 50,67 m đều có thể bị loại bỏ.

Sau khi đi đến khu vực dưới-phải tại tick 694, R2 phải quay về xử lý phần còn lại ở giữa-trên và trái; đối chứng MapEx + G1 đã xử lý các khu vực đó trước khi sang dưới-phải. Đường đi muộn đã thực sự quan sát thêm ô, vì vậy không phải hành trình hoàn toàn vô ích.

## 2. Khớp phòng theo VỊ TRÍ, không khớp bằng room_id

Mã room_id phụ thuộc thứ tự nhận diện. So sánh connected room mask của G1 trên từng run cho thấy:
- Gốc room 1 ↔ R2 room 1: IoU **1,000**.
- Gốc room 2 (trên-phải) ↔ R2 room 3: IoU **0,990**.
- Gốc room 3 (giữa-trên) ↔ R2 room 6: IoU **0,606** (mask dự đoán biến đổi).
- Gốc room 5 (dưới-trái) ↔ R2 room 2: IoU **0,974**.
- Gốc room 6 (dưới-phải) ↔ R2 room 5: IoU **1,000**.
- Gốc room 4 (trên-trái): R2 đi qua khu vực này trong GLOBAL, không có decision LOCAL tương ứng.

**Thứ tự thực tế:**
- Gốc: dưới-giữa → trên-phải → giữa-trên → trên-trái → dưới-trái → dưới-phải.
- R2: dưới-giữa → dưới-trái → trên-phải → trên-trái → dưới-phải → **quay lại giữa-trên → quay lại trái**.

Việc gán vùng ở đây từ **mask dự đoán G1 của run gốc**, chỉ dùng hậu kiểm; các mask giữa phòng có thể chồng lấn và không phải GT.

## 3. Chuỗi nhân quả, tách bằng chứng và suy luận

**D02 / tick 12, điểm rẽ đầu tiên.** Hai run bắt đầu cùng robot ở [147,111] P1. R2 chọn frontier LOCAL có thứ tự khác (index 4 thay vì index 2 của MapEx). Sau đó robot kết thúc LOCAL room 1 ở hai phía khác nhau:
- Gốc: [173,202], tick 105.
- R2: [191,112], tick 107.

Ở đây G2 chỉ thay điểm LOCAL, không sửa scorer GLOBAL. MapEx GLOBAL trên hai trạng thái khác nhau chọn hướng vùng trên-phải ở gốc, nhưng R2 chuyển sang dưới-trái. Có thể khẳng định khác biệt LOCAL xảy ra trước khi đổi thứ tự phòng, nhưng **chưa thể quy trọn quãng đường tăng thêm cho một decision D02** nếu không có ablation can thiệp riêng.

**Sau khi rời vùng trên-phải (gốc room 2).** Robot ở gần như cùng chỗ:
- Gốc D13: [32,140], tick 292, 35.624 ô đã biết; GLOBAL chọn [53,104] (gần giữa-trên), MapEx cost −1,412.
- R2 D20: [32,139], tick 477, 44.933 ô đã biết; GLOBAL chọn [15,17] (xa sang trái), cost −0,644. Vẫn có frontier khác gần vùng giữa-trên, ví dụ [33,115], score −0,233.

Tại cùng vùng robot này, R2 đã biết nhiều hơn **9.343 ô** mà gốc chưa biết; chính vì thứ tự khám phá trước đó khác, bản đồ và LaMa khác. Không thể xem hai scorer GLOBAL là đầu vào giống nhau. Nhưng thực tế R2 có frontier gần chưa hết và vẫn chọn vùng xa, là tín hiệu của hàm tối ưu có chân trời ngắn.

**D21 R2 / tick 571 — điểm quyết định có thể kiểm chứng trực tiếp.** Robot [22,15], vừa đến vùng trên-trái. Trạng thái hiện có các official GLOBAL WFD candidates sau:

| Ứng viên | A* nguyên bản trên FREE đã biết tại t570 | MapEx cost |
|---|---:|---:|
| [167,243], được chọn đi vùng dưới-phải | 37,3 m | **−0,17338** |
| [20,65], vùng gần giữa-trên | 17,4 m | −0,15064 |
| [11,97], vùng gần giữa-trên | 22,7 m | −0,07711 |
| [46,9], cạnh robot vùng trái | **3,0 m** | −0,04728 |

MapEx chọn vùng dưới-phải vì score hiện tại đứng đầu. Lựa chọn này mang về **1.994 ô** trong GLOBAL tiếp theo, nên **không phải một lựa chọn vô ích tại chỗ**. Nhưng robot còn những frontier gần lúc khởi hành và phải vượt bản đồ để quay lại giải quyết chúng.

Trước D21, R2 đã biết **6.491 / 6.930 ô (93,7%)** trong mask G1 của phòng giữa-trên từ đối chứng. Đây chỉ là mask tham khảo, có vùng chồng lấn; không chứng minh phòng vật lý đã hoàn thành.

## 4. Phép thử LiDAR native nhằm xác định các frontier có thể xử lý sớm

Dùng chính **observed map + official global WFD tại D21**, native A* có trọng số né tường, đường qua ô FREE đã biết tại thời điểm đó. P1 GT chỉ được dùng để mô phỏng LiDAR/collision cho hậu kiểm, **không** dùng làm input để chọn goal.

Chạy lại cảm biến MapEx (2.500 tia, 20 m). **Kiểm chứng nguồn**: phát lại tất cả 365 scan t571–935 trên odom gốc, đối chiếu ở tick 571, 694, 704, 818, 874, 935: **6/6 khớp bit-for-bit, 0 ô khác biệt**.

- Nếu từ D21 tới frontier [46,9] theo đường A* 3,0 m, LiDAR nhìn thêm **đúng 11 ô**, **trùng khít toàn bộ tập 11 ô** mà R2 về sau mới thu ở chặng GLOBAL 17,01 m cuối cùng.
- Nếu đi từ D21 tới [11,97] theo đường FREE đã biết dài 22,7 m, LiDAR nhìn thêm **285 ô**; trong đó **270/271 ô** mà R2 chỉ thu ở GLOBAL t705–819 và **15/59 ô** ở LOCAL tiếp theo.
- Tới [20,65] theo đường 17,4 m: thu 146 ô, gồm **87/271 ô** của GLOBAL trở lại và **59/59 ô** của LOCAL về sau.
- Các probe là **đường thay thế đơn lẻ từ snapshot D21**, không tính được ảnh hưởng đến các decision G1/MapEx tiếp theo nếu thực sự đổi goal.

**Ý nghĩa:** ít nhất hai công việc quan sát về sau đã có waypoint hợp lệ và tầm nhìn khả thi ngay trước khi robot rời xa vùng đó. Vì vậy vấn đề **không** phải G1 không sinh được candidate, cũng không phải A* không cho đường đi tới candidate; vấn đề là **thứ tự ưu tiên GLOBAL** và không đặt giá trị cho nguy cơ phải quay lại xa.

## 5. Cùng ứng viên D21, thay thứ tự đường đi (offline ablation)

Dùng các frontier đã có tại D21 và đường native weighted A* qua ô FREE đã biết; LiDAR sau mỗi 3 ô của tuyến được mô phỏng chính xác. **Chưa tái chạy toàn bộ MapEx + G1 + G2**: lịch trình waypoint được cố định offline và không replanning.

| Thứ tự từ D21 | Odometry đoạn sau D21 | Số ô P1 sau đoạn |
|---|---:|---:|
| R2 thực tế: tiếp tục hành trình tự chủ | **104,922 m** | **51.304** |
| Xa [167,243] → gần [11,97] → trái [46,9] | 83,914 m | 51.210 |
| Trái [46,9] → giữa [11,97] → xa [167,243] | **52,134 m** | 51.203 |
| Trái [46,9] → [20,65] → [11,97] → xa [167,243] | 56,982 m | **51.247** |

Với cùng ba đích chính, thứ tự ghé gần trước ngắn hơn khoảng **31,78 m** so với xa trước trong phép tái dựng; coverage khác 7 ô. Phương án có thêm waypoint thứ ba còn thiếu **57 ô** của terminal R2 dù đường ngắn hơn đáng kể. **Không cộng các số này với mức chênh GLOBAL; không gọi bất kỳ đường offline nào là một run thắng hợp lệ.** Một số frontier có thể biến mất khi quét, nên cũng không mặc định bắt buộc đi tất cả các candidate.

## 6. Kết luận thiết kế

**Nguyên nhân gần có bằng chứng:** PVU-R2 chỉ thay xếp hạng frontier **LOCAL**. Nó làm đổi chuỗi điểm ra phòng và thứ tự đi vùng. Scorer **GLOBAL không được thay đổi**, tiếp tục ưu tiên expected variance trên khoảng cách đến từng frontier, không cân nhắc việc vài candidate gần cần giải quyết trước khi đi rất xa. Điều đó tạo hai chuyến quay lại muộn với tổng 50,67 m GLOBAL, và góp phần vào chênh ròng 73,37 m.

**Không thể khẳng định:** toàn bộ 73,37 m là lãng phí, chỉ sửa D21 sẽ tiết kiệm 50 m, hoặc offline tour đạt terminal coverage. Số liệu đã chứng minh những cơ hội cục bộ thực tế bị trì hoãn, chưa định lượng được causal treatment effect toàn run nếu thay policy.

**Hướng nghiên cứu tiếp:** G2 cần xem **GLOBAL và LOCAL như một lịch trình chung**. Ở GLOBAL, so sánh trên chính tập frontier online và đường A* native:
- Có nên xử lý một **frontier nhỏ nhưng đang ở sát đường hiện tại** trước khi đi xa?
- Nếu hoãn frontier ấy, khoảng cách phải quay lại từ một số frontier xa hiện có có thể tăng thế nào?
- Có thể quan sát được phần lớn vùng của frontier nhỏ từ các tuyến đang dự định không? Tránh ghé thừa.

Chỉ ưu tiên ghé gần nếu có **lợi ích so với hoãn theo cùng nhiệm vụ quan sát**, không đặt ngưỡng thủ công dành riêng cho New Room. Phải shadow ở D21 và các GLOBAL decision khác, sau đó kiểm nghiệm run native ở New Room/Hospital.

## Hồ sơ tái lập

- [Galerie R2 27 decision](NR_G2PVU_R2_REAL_27_DECISIONS_SCROLL.html) — chú ý D02, D20, D21, D23, D27.
- [Bằng chứng theo mỗi epoch](NR_RN04_VS_G2PVU03_BY_EPOCH_CAUSAL_TRACE.json).
- [Khớp mask phòng giữa hai run](NR_RN04_G2PVU03_PHYSICAL_ROOM_CROSSWALK.json).
- [Quan sát mới trong các vùng G1 tham khảo](NR_RN04_G2PVU03_REFERENCE_ROOM_SPATIAL_DISCOVERIES.json).
- [Probe native LiDAR theo frontier tại D21](NR_G2PVU03_D21_SAME_EPOCH_FRONTIER_LIDAR_PROBES.json).
- [Thứ tự đi các frontier và coverage](NR_G2PVU03_D21_NATIVE_LIDAR_ORDER_ABLATION.json).
- [Chứng nhận phát lại 6/6 bit-exact](NR_G2PVU03_D21_LIDAR_REPLAY_BITEXACT.json).

Mã và các file JSON gốc cũng có trong thư mục phân tích trên DELL: /home/dell/DELL_NATIVE2D/analysis/g2_autonomous_research_20261018/. Hai run gốc không bị thay đổi.
