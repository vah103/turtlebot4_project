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
- generator: `mapex_lab/scripts/generate_hospital_ground_truth.py`
- trạng thái hiện tại: **chưa generate/validate trên máy chạy thí nghiệm**

NPZ phải nằm trên `hospital_canvas_v1` và có:

- `data`: `-1` = ngoài vùng chấm, `0` = free, `100` = occupied;
- `evaluation_mask`: building footprint dùng để chấm structural metrics;
- `resolution`, `origin_x`, `origin_y`.

Generator chỉ chấp nhận Hospital canonical **1.0x**. Nó rasterize wall collision mesh tại `z=0.30 m`, thêm hai `elevator_blocker`, đóng raster crack theo `roi_v1.yaml`, rồi flood-fill 8-connected từ `(0,0)` trong frame `slam_start_map`.

Chạy:

```bash
cd ~/turtlebot4_project
python3 mapex_lab/scripts/generate_hospital_ground_truth.py
```

Generator tự kiểm tra:

- Hospital scale phải bằng `1.0`;
- wall-slice bounds phải khớp `roi_v1.yaml`;
- ROI phải có đúng `215435` cell;
- SHA-256 của ROI phải là `05d45b7aba66cb6dbb71e0406005f4a3e21875901af3ae72164b17f1d3add8d1`.

Nếu cell-count hoặc SHA không khớp, script trả lỗi và artifact chỉ được xem là **candidate để kiểm tra**, không được dùng làm official GT. Có thể dùng `--allow-spec-mismatch` chỉ để debug/inspect.

Output local:

- `generated/hospital_connected_free_v1.npy`
- `generated/hospital_connected_free_v1_metadata.json`
- `generated/hospital_structural_gt_v1.npz`
- `generated/hospital_structural_gt_v1.pgm`
- `generated/hospital_structural_gt_v1_summary.json`

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
