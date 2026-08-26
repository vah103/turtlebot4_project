# STATUS

## Current stage

**Stage 2 — Nearest baseline (pilot + logging validation)**

## Done

- Tạo workspace `mapex_hospital_research/`.
- Chốt lộ trình nghiên cứu theo hướng: failure → bottleneck → oracle → research gap.
- Chốt semantic của `known_fraction` và `coverage` trong `docs/DATA_SCHEMA.md`.
- Xác nhận `hospital_flat_stack.launch.py` chạy ổn định simulation + SLAM + Nav2.
- Chốt fixed logging canvas `hospital_canvas_v1`: `0.05 m`, `1504 x 2123`, origin `(-25.6, -60.1)`.
- Chốt canonical evaluation ROI `hospital_connected_free_v1` là 8-connected structural free space chứa robot start, loại obstacle/disconnected/outside/padding.
- Thêm deterministic ROI generator có COLLADA normalization + segment dedup.
- Generate/freeze ROI trên Ubuntu:
  - denominator `215435` cells
  - SHA-256 `05d45b7aba66cb6dbb71e0406005f4a3e21875901af3ae72164b17f1d3add8d1`
  - `1040` unique wall segments
  - `2` elevator blockers
- Gán protocol `hospital_v1`.
- Chốt Nearest baseline dùng WFD trên raw `/map`, min cluster `5`, Euclidean nearest frontier representative, costmap + Nav2 planning validation.
- Chốt per-goal hard timeout `180 s`, stall timeout `30 s`, strict no-reachable-frontier completion.
- Tạo `launch/hospital_nearest.launch.py` để chạy full Hospital stack + Nearest.
- Tạo `scripts/research_recorder.py` và nối trực tiếp vào Nearest launch để tự ghi metadata, metrics, trajectory và decisions.

## In progress

- Chạy `nearest_pilot_001` để xác nhận recorder, fixed-canvas alignment, coverage denominator và termination thực tế.
- Sau pilot, kiểm tra dữ liệu trước khi bắt đầu official Nearest runs.

## Next actions

1. Pull phiên bản repo mới sau khi giữ lại local generated ROI mask.
2. Chạy `python3 mapex_hospital_research/launch/hospital_nearest.launch.py`.
3. Kiểm tra `experiments/nearest/nearest_pilot_001/metadata.json`, `metrics.csv`, `trajectory.csv`, `decisions.csv`.
4. Nếu pilot hợp lệ, chốt stage thresholds/common offline analysis budget nếu cần.
5. Chạy minimum 5, target 10 official Nearest runs.
6. Sau đó chuyển sang MapEx closed-loop dưới đúng `hospital_v1`.

## Latest result

Stage 1 đã hoàn tất. ROI chính thức đã freeze và Nearest pilot stack + recorder đã được triển khai. Chưa có benchmark research run mới; bước kế tiếp là `nearest_pilot_001`.

## Important decisions

- Không dùng kết luận của báo cáo cũ làm giả định ban đầu.
- Nearest là baseline đầu tiên; UPEN và IG-Hector chưa cần ở giai đoạn chẩn đoán.
- MapEx phải được chạy closed-loop trước khi kết luận failure.
- `known_fraction` là progress/debugging proxy trên fixed logging canvas, denominator cố định.
- `coverage` là exploration metric chính trên frozen ROI `hospital_connected_free_v1`.
- ROI denominator `215435` và SHA-256 ở trên không được thay đổi trong `hospital_v1`.
- Stage chính dùng absolute exploration state; không normalize final state riêng của từng run thành 100%.
- Tại cùng coverage stage, coverage chỉ dùng để căn chỉnh trạng thái; so time/distance-to-stage, goal outcomes và diagnostic metrics.
- `hospital_v1` không dùng fixed time/distance budget làm controller termination; common budget có thể được chọn offline sau pilot nếu cần cho secondary analysis.
- Dữ liệu nặng không commit lên GitHub.
- Google Docs teacher report chỉ nhận milestone/kết quả đã xác minh; pilot/debug không được trình bày như kết quả chính thức.

## Handoff rule

Mỗi khi hoàn thành một bước lớn, cập nhật file này trước khi kết thúc phiên làm việc. Nếu milestone có kết quả đủ tin cậy, đồng thời cập nhật Google Docs teacher report theo `docs/TEACHER_REPORT.md`.
