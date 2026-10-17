# RUN REGISTRY — tên chuẩn thí nghiệm MapEx native 2D

**Ngày áp dụng:** 2026-10-17 · **Phạm vi:** các run New Room/Hospital MX072 + G2-R4/G2-R5 đã được đối chiếu.
**Bảng máy đọc được:** [RUN_REGISTRY.csv](RUN_REGISTRY.csv).

> Đây là đổi **tên hiển thị / bí danh chính thức (canonical display ID)**, không đổi tên hay ghi đè thư mục dữ liệu, JSON, mã nguồn, nhánh Git, HTML/PDF, raw logs hoặc ID trong provenance cũ. Mọi run giữ nguồn thí nghiệm nguyên trạng.

## Quy ước

- `NR` = New Room; `HP` = Hospital.
- `MX` = MapEx gốc GLOBAL; `RN` = MapEx + Room Node V4-R4.1, MapEx/WFD chọn goal trong LOCAL.
- `G2R4` = MapEx + Room Node + G2-R4; `G2R5` = MapEx + Room Node + G2-R5 (MCTS).
- `CHAIN` = goal chain; `EXACT` = tới chính xác ô goal; `PRED` = viewpoint dự đoán UNKNOWN; `WFD` = dùng frontier để quyết định **rời LOCAL** (G2 vẫn chọn goal LOCAL); `STRUCT` = arbitration ưu tiên cấu trúc; `MCTS` = MCTS.
- `-01`, `-02`, ... là lần chạy độc lập **cùng một cấu hình** trên cùng môi trường; khi đổi thuật toán/cơ chế chính, tạo tên biến thể mới. `-PILOT01` là tên riêng cho lượt thử dừng lỗi.
- `SOURCE_TERMINAL` chỉ có nghĩa native đã kết thúc theo điều kiện dừng của mã nguồn (thường không còn frontier phù hợp), **không chứng minh hoàn thành GT, an toàn robot thật, hay đã QA khoa học**.
- `FAILED` và `INCOMPLETE` **không** được đưa vào so sánh hành trình toàn bản đồ cùng coverage với các run hoàn thành.

## Đối chiếu tên cũ ↔ tên mới

| Tên chuẩn | Tên cũ / ngữ cảnh | Trạng thái | Quãng đường native (m) | Bằng chứng gốc |
|---|---|---|---:|---|
| `NR-MX-01` | Run 05 (New Room) | Hoàn thành theo điều kiện dừng native | 369.696 | [Báo cáo](https://github.com/vah103/turtlebot4_project/blob/mx072-newroom-real-25-decisions-ipad-20261015/mapex_lab/figures/mx072_newroom_optimized_20261015/README.md) |
| `NR-RN-01` | Run 04 (New Room) | Hoàn thành theo điều kiện dừng native | 199.108 | [Báo cáo](https://github.com/vah103/turtlebot4_project/blob/mx072-newroom-real-25-decisions-ipad-20261015/mapex_lab/figures/mx072_newroom_optimized_20261015/README.md) |
| `NR-G2R4-CHAIN-PILOT01` | Run 06 (G2-R4) | Dừng lỗi | — | [Báo cáo](https://github.com/vah103/turtlebot4_project/blob/mx072-g2-r4-native-run07-com1-20261017/mapex_lab/analysis/g2_r4_native_run07_20261017/REPORT_RUN07_MATCHED_RUN04_RUN05.md) |
| `NR-G2R4-CHAIN-01` | Run 07 (G2-R4) | Hoàn thành theo điều kiện dừng native | 340.182 | [Báo cáo](https://github.com/vah103/turtlebot4_project/blob/mx072-g2-r4-native-run07-com1-20261017/mapex_lab/analysis/g2_r4_native_run07_20261017/REPORT_RUN07_MATCHED_RUN04_RUN05.md) |
| `NR-G2R4-EXACT-01` | Run 08 (G2-R4) | Hoàn thành theo điều kiện dừng native | 310.183 | [Báo cáo](https://github.com/vah103/turtlebot4_project/blob/mx072-g2-r4-exact-goal-run08-20261017/mapex_lab/analysis/g2_r4_exact_goal_run08_20261017/README_RUN08_EXACT_GOAL.md) |
| `NR-G2R4-PRED-01` | Run 09 (G2-R4) | Hoàn thành theo điều kiện dừng native | 282.224 | [Báo cáo](https://github.com/vah103/turtlebot4_project/blob/mx072-g2-r4-predicted-goal-run09-20261017/mapex_lab/analysis/g2_r4_predicted_goal_run09_20261017/README_RUN09_PREDICTED_GOAL.md) |
| `NR-G2R4-WFD-PILOT01` | Run 10-R1 (G2-R4) | Dừng lỗi | — | [Báo cáo](https://github.com/vah103/turtlebot4_project/blob/mx072-g2-r4-frontier-exit-run10r2-20261017/mapex_lab/analysis/g2_r4_frontier_exit_run10r2_20261017/README_RUN10R2_FRONTIER_EXIT.md) |
| `NR-G2R4-WFD-01` | Run 10-R2 (G2-R4) | Hoàn thành theo điều kiện dừng native | 277.293 | [Báo cáo](https://github.com/vah103/turtlebot4_project/blob/mx072-g2-r4-frontier-exit-run10r2-20261017/mapex_lab/analysis/g2_r4_frontier_exit_run10r2_20261017/README_RUN10R2_FRONTIER_EXIT.md) |
| `NR-G2R4-STRUCT-01` | Run 11 (G2-R4) | Hoàn thành theo điều kiện dừng native | 285.471 | [Báo cáo](https://github.com/vah103/turtlebot4_project/blob/mx072-g2-r4-structural-predicted-run11-result-20261017/mapex_lab/analysis/g2_r4_structural_predicted_run11_result_20261017/README_RUN11_NATIVE_RESULT.md) |
| `NR-G2R5-MCTS-01` | Run 09 (G2-R5) | Hoàn thành theo điều kiện dừng native | 343.25 | [Báo cáo](https://github.com/vah103/turtlebot4_project/blob/mx072-g2-r5-run09-native-20261017/mapex_lab/figures/mx072_g2_r5_run09_20261017/MX072_RUN04_RUN05_RUN09_G2_R5_SCROLL.html) |
| `NR-G2R5-WFD-01` | Run 13 (G2-R5) | Chưa hoàn thành | 82.69 | [Báo cáo](https://github.com/vah103/turtlebot4_project/blob/mx072-g2-r5-run13-frontier-exit-incomplete-20261017/mapex_lab/figures/mx072_g2_r5_run13_20261017/MX072_RUN04_RUN05_RUN09_RUN13_G2_R5_FRONTIER_EXIT_SCROLL.html) |
| `HP-MX-01` | Run 11 (Hospital) | Hoàn thành theo điều kiện dừng native | 832.5093 | [Báo cáo](https://github.com/vah103/turtlebot4_project/blob/mx072-hospital-full-run10-vs-run11-20261016/mapex_lab/figures/mx072_hospital_full_corrected_20261016/README.md) |
| `HP-RN-01` | Run 10 (Hospital) | Hoàn thành theo điều kiện dừng native | 642.2194 | [Báo cáo](https://github.com/vah103/turtlebot4_project/blob/mx072-hospital-full-run10-vs-run11-20261016/mapex_lab/figures/mx072_hospital_full_corrected_20261016/README.md) |

**Lưu ý về số đo:** cột quãng đường ưu tiên odometry toàn hành trình từ báo cáo native; vì vậy `NR-MX-01` 369.696 m và `NR-RN-01` 199.108 m khác số làm tròn/ghi theo cách khác trên trang gallery Run04/05 (370.00 và 199.41 m). `NR-G2R5-WFD-01` chỉ đi được 82.69 m trước khi dừng chưa hoàn thành; không được xếp như run hiệu quả nhất.

## Cách dùng tên mới

- Báo cáo, biểu đồ và cuộc trao đổi mới phải dùng **tên chuẩn**, ví dụ: `NR-G2R4-WFD-01 (cũ: Run 10-R2)`. Khi cần truy nguồn, sử dụng `source_ref` và `source_path` trong CSV; **không** suy đoán đường dẫn từ run ID.
- Tránh viết “Run 09” hoặc “Run 11” đơn độc: ít nhất phải ghi môi trường và họ thuật toán. Ví dụ `NR-G2R4-PRED-01` **khác** `NR-G2R5-MCTS-01`; `HP-MX-01` **khác** `NR-G2R4-STRUCT-01`.
- Khi chạy replicate hoặc thay đổi nhỏ mà vẫn giữ cùng policy/config, cấp suffix kế tiếp, không chiếm lại ID cũ. Khi thay cơ chế, bổ sung một biến thể mới và ghi rõ thay đổi/provenance.
- Cấu trúc `mpx_001..010`, `hpx_001..005`, MX072 gốc 1,195 scans/22 scored decisions và pilot COM1 technical 3 ticks là **dòng dữ liệu khác**, vẫn giữ nguyên tên lịch sử; chưa gộp vào bảng closed-loop native 2D này.
- Các so sánh trên mới là **một lượt/một cặp** và có những bộ đo không tương đương; xem báo cáo gốc trước khi khẳng định tính ưu việt. Không coi tên “baseline” là trạng thái được IR1 chấp thuận.

## Tình trạng hiệu lực

**DONE:** thiết lập 13 ID không trùng, liên kết exact branch/artifact, giữ 2 pilot FAIL và 1 run INCOMPLETE tách riêng; alias của run cũ có thể truy ngược.
**NEXT:** mỗi run mới bổ sung 1 dòng CSV và bảng ở tài liệu này sau khi có source-ref và receipt; không đổi tên các artifact đã freeze.
