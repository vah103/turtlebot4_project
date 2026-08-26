# STATUS

## Current stage

**Stage 1 — Chuẩn hóa môi trường Hospital**

## Done

- Tạo workspace `mapex_hospital_research/`.
- Chốt lộ trình nghiên cứu theo hướng: failure → bottleneck → oracle → research gap.
- Tạo cấu trúc lưu code, config, experiment, result và tài liệu handoff.
- Chốt semantic của `known_fraction` và `coverage` trong `docs/DATA_SCHEMA.md`.
- Sửa stage analysis để không normalize final state riêng của từng run thành 100%.
- Làm rõ same-stage comparison: tại cùng coverage stage, coverage là biến alignment; so time/distance-to-stage, goal outcomes và diagnostic metrics thay vì so coverage với coverage.
- Thiết lập workflow báo cáo teacher-facing: repo là source of evidence; Google Docs `TurtleBot4` → tab `Báo cáo nghiên cứu MapEx Hospital` là báo cáo chính cho giảng viên.
- Ghi cố định Google Doc ID và report tab ID trong `docs/TEACHER_REPORT.md` để tránh chọn nhầm file trùng tên.
- Thêm `docs/TEACHER_REPORT.md` và quy tắc agent phải cập nhật báo cáo sau milestone có kết quả đã xác minh.
- Xác nhận `hospital_flat_stack.launch.py` chạy ổn định simulation + SLAM + Nav2.
- Chốt fixed logging canvas `hospital_canvas_v1`: 0.05 m, 1504 x 2123, origin (-25.6, -60.1).
- Chốt định nghĩa canonical evaluation ROI `hospital_connected_free_v1`: connected structural free space từ spawn, loại obstacle, disconnected pockets, ngoài Hospital bounds và canvas padding.
- Ghi ROI spec vào `ground_truth/hospital/roi_v1.yaml` và cập nhật `config/hospital.yaml` + `EXPERIMENT_PROTOCOL.md`.

## In progress

- Generate canonical ROI mask một lần và freeze `denominator_cells` trước run `nearest_001`.
- Chốt các protocol field còn lại: timeout/stopping condition/resource budget nếu cần.

## Next actions

1. Generate `hospital_connected_free_v1` mask theo `ground_truth/hospital/roi_v1.yaml` và ghi denominator cố định.
2. Chốt goal timeout, stopping condition và các field protocol còn TODO.
3. Gán protocol version `hospital_v1` sau khi denominator đã freeze.
4. Tạo launch mới cho Nearest baseline trong workspace này.
5. Chạy thử 1 run Nearest để kiểm tra logging và cấu trúc dữ liệu.

## Latest result

Hospital full stack đã được xác nhận chạy bình thường. Fixed logging canvas và định nghĩa canonical ROI đã được chốt; chưa bắt đầu benchmark. ROI denominator vẫn phải được generate/freeze trước `nearest_001`.

## Important decisions

- Không dùng kết luận của báo cáo cũ làm giả định ban đầu.
- Nearest là baseline đầu tiên; UPEN và IG-Hector chưa cần ở giai đoạn chẩn đoán.
- MapEx phải được chạy closed-loop trước khi kết luận failure.
- Mỗi run phải lưu metadata + metric + dữ liệu quyết định đủ để phân tích offline sau này.
- `known_fraction` là progress/debugging proxy trên fixed logging canvas, denominator cố định.
- `coverage` là exploration metric chính trên canonical evaluation ROI, denominator cố định.
- `hospital_connected_free_v1` chỉ gồm 8-connected structural free cells chứa robot start trên grid 0.05 m; obstacle cells không nằm trong coverage denominator và được đánh giá correctness riêng bằng occupied IoU.
- Stage chính dùng absolute exploration state; không kéo giãn final state của từng run thành 100%.
- Tại cùng coverage stage, coverage chỉ dùng để căn chỉnh trạng thái; so time/distance-to-stage, goal outcomes và các diagnostic metrics.
- Overall exploration efficiency báo riêng bằng Coverage-vs-time, Coverage-vs-distance, AUC và final coverage dưới cùng fixed budget.
- Có thể phân tích phụ theo normalized time/distance progress nhưng phải dùng common fixed budgets.
- Google Docs teacher report chỉ nhận kết quả đã xác minh; pilot/debug không được trình bày như kết quả chính thức.
- Dữ liệu nặng không commit lên GitHub.

## Handoff rule

Mỗi khi hoàn thành một bước lớn, cập nhật file này trước khi kết thúc phiên làm việc. Nếu milestone có kết quả đủ tin cậy, đồng thời cập nhật Google Docs teacher report theo `docs/TEACHER_REPORT.md`.
