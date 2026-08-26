# Experiment Notes

Ghi ngắn gọn các quyết định hoặc sự cố có thể ảnh hưởng kết quả.

## Template

### YYYY-MM-DD — <run or change>

- What changed:
- Why:
- Affected runs:
- Does baseline need rerun?:
- Observation:
- Next action:

## 2026-08-26 — Freeze Hospital canvas and ROI definition

- What changed: chốt `hospital_canvas_v1` và định nghĩa `hospital_connected_free_v1`.
- Why: coverage cần denominator cố định, không phụ thuộc crop/map extent của từng run.
- Affected runs: toàn bộ Nearest/MapEx run mới trong lộ trình `mapex_hospital_research`.
- Does baseline need rerun?: chưa; benchmark mới chưa bắt đầu. Nếu ROI thay đổi sau `nearest_001` thì phải chạy lại baseline.
- Observation: full Hospital stack đã được xác nhận chạy bình thường; fixed canvas hiện có là 0.05 m, 1504 x 2123, origin (-25.6, -60.1).
- ROI rule: structural obstacles = wall mesh + flat elevator blockers; rasterize ở z=0.30 m, thickness 2 cells, close one-cell cracks; trong Hospital bounds lấy 8-connected free component chứa robot start `(0,0)` ở SLAM-start frame.
- Next action: generate mask một lần, ghi/freeze `denominator_cells`, sau đó mới gán protocol `hospital_v1` và chạy `nearest_001`.

## Initial note

Workspace mới được tạo để làm lại nghiên cứu từ đầu. Chưa có run mới và chưa chốt bottleneck.
