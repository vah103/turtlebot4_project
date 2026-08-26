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

## 2026-08-26 — Freeze Hospital ROI v1

- What changed: generate và freeze `hospital_connected_free_v1`.
- Denominator: `215435` cells.
- Mask SHA-256: `05d45b7aba66cb6dbb71e0406005f4a3e21875901af3ae72164b17f1d3add8d1`.
- Structural audit: `1040` unique wall segments, `2` elevator blockers.
- Why: coverage cần một denominator cố định giống hệt giữa Nearest và MapEx.
- Affected runs: toàn bộ run dùng protocol `hospital_v1`.
- Does baseline need rerun?: chưa có official baseline run. Sau khi baseline bắt đầu, mọi thay đổi ROI yêu cầu protocol/ROI ID mới và baseline rerun.
- Observation: ROI generator dùng COLLADA unit/up-axis normalization + unique plane-intersection segments trước rasterization, sau đó lấy 8-connected free component chứa spawn.
- Next action: chạy `nearest_pilot_001` với research recorder để xác nhận metrics/alignment/termination trước official Nearest runs.

## 2026-08-26 — Freeze Hospital canvas and ROI definition

- What changed: chốt `hospital_canvas_v1` và định nghĩa `hospital_connected_free_v1`.
- Why: coverage cần denominator cố định, không phụ thuộc crop/map extent của từng run.
- Affected runs: toàn bộ Nearest/MapEx run mới trong lộ trình `mapex_hospital_research`.
- Does baseline need rerun?: chưa; benchmark mới chưa bắt đầu. Nếu ROI thay đổi sau baseline thì phải chạy lại baseline.
- Observation: full Hospital stack đã được xác nhận chạy bình thường; fixed canvas hiện có là 0.05 m, 1504 x 2123, origin (-25.6, -60.1).
- ROI rule: structural obstacles = wall mesh + flat elevator blockers; rasterize ở z=0.30 m, thickness 2 cells, close one-cell cracks; trong Hospital bounds lấy 8-connected free component chứa robot start `(0,0)` ở SLAM-start frame.

## Initial note

Workspace mới được tạo để làm lại nghiên cứu từ đầu. Chưa có run mới và chưa chốt bottleneck.
