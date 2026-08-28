# Hospital Ground Truth

Đây là nơi đặt config/code nhẹ liên quan đến structural ground truth của Hospital.

Canonical exploration ROI:
- spec: `roi_v1.yaml`
- ID: `hospital_connected_free_v1`
- generator: `../../scripts/generate_hospital_roi.py`
- generated mask: `generated/hospital_connected_free_v1.npy` (local-only, không commit)

Generate một lần trước `nearest_001`:

```bash
python3 mapex_hospital_research/scripts/generate_hospital_roi.py
```

Script tính `denominator_cells`, SHA-256 của mask và tự cập nhật `roi_v1.yaml`, `config/hospital.yaml`, `EXPERIMENT_PROTOCOL.md`.

Generated raster/array lớn nên để local và không commit.

Khi triển khai phải ghi rõ:
- nguồn mesh/collision;
- transform world → map;
- resolution;
- z-slice nếu dùng wall mesh;
- phần collision nào được bao gồm/bỏ qua;
- validation alignment với SLAM map.
