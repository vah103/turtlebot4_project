# Hospital Ground Truth

Đây là nơi đặt config/code nhẹ liên quan đến structural ground truth của Hospital.

Generated raster/array lớn nên để local và không commit nếu kích thước lớn.

Khi triển khai phải ghi rõ:
- nguồn mesh/collision;
- transform world → map;
- resolution;
- z-slice nếu dùng wall mesh;
- phần collision nào được bao gồm/bỏ qua;
- validation alignment với SLAM map.
