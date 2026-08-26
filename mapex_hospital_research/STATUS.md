# STATUS

## Current stage

**Stage 1 — Chuẩn hóa môi trường Hospital**

## Done

- Tạo workspace `mapex_hospital_research/`.
- Chốt lộ trình nghiên cứu theo hướng: failure → bottleneck → oracle → research gap.
- Tạo cấu trúc lưu code, config, experiment, result và tài liệu handoff.
- Chốt semantic của `known_fraction` và `coverage` trong `docs/DATA_SCHEMA.md`.
- Sửa stage analysis để không normalize final state riêng của từng run thành 100%.

## In progress

- Chuẩn hóa cấu hình Hospital dùng chung cho Nearest và MapEx.
- Chốt fixed logging canvas và canonical evaluation ROI/mask thực tế.

## Next actions

1. Kiểm tra launch simulation + SLAM + Nav2 hiện có trong `ros2_ws/src/frontier_exploration/`.
2. Chốt spawn pose, map resolution, timeout, stopping condition, Nav2/SLAM params.
3. Chốt fixed logging canvas: resolution, size, origin, alignment.
4. Chốt canonical evaluation ROI: source/mask, valid-cell rule, excluded cells và denominator.
5. Chốt fixed time/distance budget nếu dùng secondary resource-progress analysis.
6. Điền giá trị thật vào `EXPERIMENT_PROTOCOL.md` và `config/hospital.yaml`.
7. Tạo launch mới cho Nearest baseline trong workspace này.
8. Chạy thử 1 run Nearest để kiểm tra logging và cấu trúc dữ liệu.

## Latest result

Chưa có run nghiên cứu mới. Metric/stage definitions đã được làm rõ trước khi bắt đầu thu dữ liệu benchmark.

## Important decisions

- Không dùng kết luận của báo cáo cũ làm giả định ban đầu.
- Nearest là baseline đầu tiên; UPEN và IG-Hector chưa cần ở giai đoạn chẩn đoán.
- MapEx phải được chạy closed-loop trước khi kết luận failure.
- Mỗi run phải lưu metadata + metric + dữ liệu quyết định đủ để phân tích offline sau này.
- `known_fraction` là progress/debugging proxy trên fixed logging canvas, denominator cố định.
- `coverage` là exploration metric chính trên canonical evaluation ROI, denominator cố định.
- Stage chính dùng absolute exploration state; không kéo giãn final state của từng run thành 100%.
- Có thể phân tích phụ theo normalized time/distance progress nhưng phải dùng common fixed budgets.
- Dữ liệu nặng không commit lên GitHub.

## Handoff rule

Mỗi khi hoàn thành một bước lớn, cập nhật file này trước khi kết thúc phiên làm việc.
