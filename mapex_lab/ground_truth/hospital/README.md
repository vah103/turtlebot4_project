# Hospital Ground Truth

Đây là nơi đặt config/code nhẹ liên quan đến ground truth của Hospital.

## Canonical exploration ROI

- spec: `roi_v1.yaml`
- ID: `hospital_connected_free_v1`
- generated mask: `generated/hospital_connected_free_v1.npy` (local-only, không commit)

ROI này dùng làm denominator cố định cho Coverage.

## Structural ground truth cho IoU/TU

- spec: `structural_gt_v1.yaml`
- ID: `hospital_structural_gt_v1`
- generated artifact: `generated/hospital_structural_gt_v1.npz` (local-only, không commit)
- trạng thái hiện tại: **chưa generate/validate**

NPZ phải nằm trên `hospital_canvas_v1` và có:

- `data`: `-1` = ngoài vùng chấm, `0` = free, `100` = occupied;
- `evaluation_mask`: building footprint dùng để chấm structural metrics;
- `resolution`, `origin_x`, `origin_y`.

`mapex_lab/scripts/evaluate_mapex_run.py` dùng artifact này để tính:

- occupied IoU từ ensemble mean prediction, threshold `> 0.5`;
- Topological Understanding với 100 goal cố định, 4-connected path planning;
- IoU/TU theo từng MapEx decision và AUC theo time/distance;
- backfill `occupied_iou` và `tu` vào `metrics.csv` sau khi run kết thúc.

`mapex_run.py` tự gọi evaluator. Nếu structural GT chưa tồn tại hoặc chưa hợp lệ, run vẫn được lưu bình thường và `evaluation.json` ghi trạng thái `skipped_*`; không sinh metric giả.

## Validation bắt buộc trước khi dùng IoU/TU trong báo cáo

Phải ghi rõ và kiểm tra:

- nguồn mesh/collision;
- transform world → map;
- resolution;
- z-slice nếu rasterize wall mesh;
- collision nào được bao gồm/bỏ qua;
- alignment structural GT với SLAM map;
- building footprint/evaluation mask;
- start pose nằm trong free space.

Generated raster/array lớn để local và không commit.
