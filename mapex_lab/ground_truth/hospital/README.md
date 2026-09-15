# Hospital Ground Truth

Đây là nơi đặt config/code nhẹ liên quan đến ground truth của Hospital.

## Canonical exploration ROI

- spec: `roi_v1.yaml`
- ID: `hospital_connected_free_v1`
- generated mask: `generated/hospital_connected_free_v1.npy` (local-only, không commit)

ROI này dùng làm denominator cố định cho Coverage.

### Provenance của ROI v1

ROI v1 được freeze ngày 26/08/2026 bằng pipeline cũ `mapex_hospital_research/scripts/generate_hospital_roi.py`. Pipeline đó dùng `trimesh` để áp COLLADA scene-graph transforms, `cv2.line/fillPoly`, `cv2.dilate`, rồi lấy thành phần free-space 8-connected chứa robot start.

Tại thời điểm freeze ROI v1, hai `elevator_blocker` trong `hospital_aws_flat.sdf` rộng **1.60 m**. Ngày 27/08/2026 world được sửa thành **1.80 m** để chặn khe elevator tốt hơn. Vì vậy world hiện tại không được phép mặc nhiên giả định sẽ có cùng SHA với ROI v1. Nếu cell-count/SHA khác sau khi generator canonical chạy, cần coi đó là geometry revision và quyết định tạo ROI version mới thay vì ép kết quả khớp v1.

Frozen ROI v1 audit:

- denominator: `215435` cells;
- SHA-256: `05d45b7aba66cb6dbb71e0406005f4a3e21875901af3ae72164b17f1d3add8d1`.

## Structural ground truth cho IoU/TU

- spec: `structural_gt_v1.yaml`
- ID: `hospital_structural_gt_v1`
- generated artifact: `generated/hospital_structural_gt_v1.npz` (local-only, không commit)
- generator: `mapex_lab/scripts/generate_hospital_ground_truth.py`
- canonical rasterization core: `mapex_lab/scripts/hospital_ground_truth_core.py`
- trạng thái hiện tại: **chưa validate alignment trên máy chạy thí nghiệm**

NPZ nằm trên `hospital_canvas_v1` và có:

- `data`: `-1` = ngoài vùng chấm, `0` = free, `100` = occupied;
- `evaluation_mask`: vùng dùng để chấm structural metrics;
- `resolution`, `origin_x`, `origin_y`.

Generator chỉ chấp nhận Hospital **1.0x**. Nó dùng lại đúng rasterization semantics của generator ROI v1 lịch sử, nhưng luôn rasterize geometry của **active world hiện tại** thay vì sửa geometry để ép hash cũ.

Chạy:

```bash
cd ~/turtlebot4_project
python3 mapex_lab/scripts/generate_hospital_ground_truth.py
```

Output local:

- `generated/hospital_connected_free_v1.npy`
- `generated/hospital_connected_free_v1_metadata.json`
- `generated/hospital_structural_gt_v1.npz`
- `generated/hospital_structural_gt_v1_preview.png`
- `generated/hospital_structural_gt_v1_summary.json`

Output JSON sẽ báo một trong hai trạng thái chính:

- `ok_frozen_v1_match`: active world tái tạo đúng mask v1;
- `ok_active_world_differs_from_frozen_v1`: rasterization đã đúng pipeline canonical nhưng geometry active world cho mask khác v1. Trường hợp này không được đổi SHA/cell-count của v1 một cách âm thầm; cần version ROI/GT mới hoặc quay lại đúng geometry đã freeze.

`mapex_lab/scripts/evaluate_mapex_run.py` dùng artifact structural GT để tính:

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
