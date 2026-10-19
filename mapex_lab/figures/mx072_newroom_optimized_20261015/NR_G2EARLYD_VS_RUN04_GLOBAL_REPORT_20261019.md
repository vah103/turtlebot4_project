# GLOBAL — MapEx + G1 Run04 vs G2-EARLYDOOR-R1
Native 2D New Room · 19/10/2026 · đối chiếu 6 GLOBAL theo thứ tự thời gian

## Kết quả chính

| | MapEx+G1 `NR-RN-04` | G2-EARLYDOOR `NR-G2EARLYD-02` |
|---|---:|---:|
| Tổng GLOBAL | **94.928869 m** | **121.895432 m** |
| Tổng LOCAL | 104.178714 m | **91.306578 m** |
| Tổng hành trình | **199.107583 m** | 213.202010 m |
| GLOBAL decision | 6 | 6 |
| LOCAL decision | 19 | 22 |
| Observed P1 cuối run | 51.304 | 51.303 |

EARLYDOOR đi GLOBAL dài hơn **26.966563 m**. Hai run không đạt chính xác cùng terminal observed P1 (EARLYDOOR thiếu 1 ô), và có lịch sử LOCAL khác nhau.

## Mỗi lần GLOBAL

| Lượt | Run04 D/tick, robot → goal P1 | Run04 thực đi | EarlyDoor D/tick, robot → goal P1 | EarlyDoor thực đi | Chênh |
|---|---|---:|---|---:|---:|
| 1 | D01 t0: [131,131] → [153,104] | 3.371 m | D01 t0: [131,131] → [153,104] | 3.371 m | 0 |
| 2 | D07 t105: [173,202] → [40,179] | 14.912 m | D07 t84: [185,203] → [39,179] | 16.260 m | +1.347 m |
| **3** | **D13 t292: [32,140] → [53,104]** | **14.018 m** | **D14 t245: [67,232] → [155,54]** | **40.396 m** | **+26.378 m** |
| 4 | D17 t399: [20,57] → [46,39] | 10.636 m | D18 t449: [131,34] → [85,30] | 17.383 m | +6.747 m |
| 5 | D20 t471: [34,9] → [158,32] | 20.067 m | D23 t551: [14,29] → [48,59] | 13.925 m | −6.142 m |
| 6 | D24 t587: [187,28] → [167,242] | 31.925 m | D27 t636: [19,93] → [167,243] | 30.561 m | −1.364 m |

Chiều dài đo từ native odometry giữa các decision liên tiếp. Ghép theo số thứ tự GLOBAL chỉ là cách nhìn diễn tiến, **không phải một thí nghiệm cùng trạng thái**.

## Phân tích INPUT của MapEx GLOBAL

Cùng hàm GLOBAL `FrontierPlanner.score_frontiers`, `visvarprob`: original global WFD trong observed map; expected variance IG từ mean/variance LaMa qua raycast probabilistic 250 tia 20m; chọn chi phí âm `-IG / Euclidean(robot,frontier)` trong ô P1. Sau đó robot lập đường native clearance-weighted A*. Do đó cùng MapEx nhưng robot pose, observed UNKNOWN, các WFD và mean/variance khác sẽ cho goal khác.

| Lượt | Khoảng cách hai robot khi bắt đầu GLOBAL | Số ô biết/chưa biết khác nhau | WFD trùng tọa độ |
|---|---:|---:|---:|
| 1 | 0 m | 0 | 13 |
| 2 | 1.204 m | 513 | 12 |
| **3** | **9.843 m** | **226** | **11** |
| 4 | 11.336 m | 9.524 | 3 |
| 5 | 2.828 m | 5.607 | 2 |
| 6 | 18.014 m | 42 | 2 |

Mấu chốt sau phòng 2 là vị trí thoát: Run04 robot ở [32,140], EarlyDoor ở [67,232] (cách nhau 9.84 m). Hai frontier GLOBAL [53,104] và [155,54] cùng tồn tại. Frontiers có IG thô:

- [53,104]: Run04 IG **58.868** / Euclid 4.168m → điểm 1.412; EarlyDoor IG **50.490** / 12.876m → 0.392.
- [155,54]: Run04 IG **137.520** / Euclid 15.008m → 0.916; EarlyDoor IG **134.152** / 19.856m → 0.676.

Hoán đổi vị trí robot nhưng giữ nguyên IG/WFD của từng epoch (phép kiểm thử tính điểm 2×2 trước đó) thì goal thắng đổi theo **vị trí robot** trong cả hai bộ. Đây là bằng chứng đủ để giải thích đảo thứ hạng khi GLOBAL thứ 3. Cần tránh tuyên bố một yếu tố duy nhất giải thích toàn bộ run; lịch sử LOCAL khác cũng làm observed/LaMa và WFD thay đổi.

## Chỉ một GLOBAL bị EarlyDoor chủ động thay

Tại EarlyDoor D23 t551, current original MapEx vốn xếp:
- [18,64] score gốc 0.388337 (cao nhất)
- [48,59] score gốc 0.366273 (thấp hơn)

EarlyDoor đã chọn [48,59] vì cặp (GLOBAL goal, portal) được dự kiến và cập nhật trong phòng 4, vẫn là một current WFD reachable. Để điều khiển native, log EarlyDoor ghi `cost=-1.388337` cho goal được thay; đây là **chi phí đã sửa phục vụ điều khiển**, không phải IG thật. Original score `-0.366273` được phục hồi từ SWEEP-R1 D23 có đúng cùng tick t551, robot pose và observed map bit-exact.

Tại EARLYDOOR D07, dự kiến LOCAL cũ đã không còn là WFD → fallback MapEx. Ở D14, D18, D27, goal lưu vẫn hợp lệ nhưng **trùng** chính goal MapEx mới tính. Vì thế chuyến GLOBAL 40.396 m ở D14 **không phải do mục tiêu lưu sớm ép robot**, mà là do MapEx chấm từ vị trí cuối phòng 2 khác.

## Kết luận khoa học

- LOCAL sweep thay đổi chỗ robot thoát khỏi từng phòng, kéo theo nguyên nhân đổi goal GLOBAL dù thuật toán MapEx gốc giống nhau.
- EARLYDOOR có đúng 1 GLOBAL goal intervention trên New Room, xảy ra sau phòng 4; không có sự kiện phát hiện cửa mới giữa LOCAL trong native run này. Có các cập nhật dự kiến vì dữ liệu WFD/IG mới.
- Đánh giá đóng góp riêng của EARLYDOOR phải so với G2-SWEEP-R1 (LOCAL thiết kế giống): GLOBAL −3.452786m, LOCAL +3.147214m, toàn run −0.305573m, end observed P1 bit-exact. So với Run04, tổng vẫn dài hơn +14.094427m.
- Không được kết luận toàn bộ +26.97m GLOBAL do thuật toán chọn GLOBAL sớm; riêng chặng GLOBAL thứ 3 đã khác do vị trí ra phòng 2, và EarlyDoor không đổi goal MapEx tại đó.

## Xem ảnh

`NR_G2EARLYD_VS_RUN04_GLOBAL_6_PAIRS_SCROLL.html` gồm 12 full observed maps, 6 cặp GLOBAL, overlay quỹ đạo native trước và sau, A*, frontier/goal, kèm IG, Euclid, WFD và độ bất đồng observed. Quỹ đạo sau thời điểm GLOBAL được vẽ hồi cứu chỉ để báo cáo kết quả, không dùng trong lập kế hoạch online.

Dữ liệu kiểm chứng: `NR_G2EARLYD_VS_RUN04_GLOBAL_STATES.json`, script `audit_earlydoor_vs_run04_globals.py`. Nguồn native frozen: `/home/dell/DELL_NATIVE2D/runs/NR-RN-04/` và `NR-G2EARLYD-02/
