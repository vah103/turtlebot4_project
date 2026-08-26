# STATUS

## Current stage

**Stage 1 — Chuẩn hóa môi trường Hospital**

## Done

- Tạo workspace `mapex_hospital_research/`.
- Chốt lộ trình nghiên cứu theo hướng: failure → bottleneck → oracle → research gap.
- Tạo cấu trúc lưu code, config, experiment, result và tài liệu handoff.

## In progress

- Chuẩn hóa cấu hình Hospital dùng chung cho Nearest và MapEx.

## Next actions

1. Kiểm tra launch simulation + SLAM + Nav2 hiện có trong `ros2_ws/src/frontier_exploration/`.
2. Chốt spawn pose, map resolution, timeout, stopping condition, Nav2/SLAM params.
3. Điền giá trị thật vào `EXPERIMENT_PROTOCOL.md` và `config/hospital.yaml`.
4. Tạo launch mới cho Nearest baseline trong workspace này.
5. Chạy thử 1 run Nearest để kiểm tra logging và cấu trúc dữ liệu.

## Latest result

Chưa có run nghiên cứu mới.

## Important decisions

- Không dùng kết luận của báo cáo cũ làm giả định ban đầu.
- Nearest là baseline đầu tiên; UPEN và IG-Hector chưa cần ở giai đoạn chẩn đoán.
- MapEx phải được chạy closed-loop trước khi kết luận failure.
- Mỗi run phải lưu metadata + metric + dữ liệu quyết định đủ để phân tích offline sau này.
- Dữ liệu nặng không commit lên GitHub.

## Handoff rule

Mỗi khi hoàn thành một bước lớn, cập nhật file này trước khi kết thúc phiên làm việc.
