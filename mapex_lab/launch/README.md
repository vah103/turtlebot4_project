# Launch

Ba launch chính của `mapex_lab`:

- `stock.launch.py`
  - simulation + SLAM Toolbox + Nav2
  - dùng trực tiếp `config/slam.yaml`
  - không có local scan frontend

- `local.launch.py`
  - simulation + local scan frontend + SLAM Toolbox + Nav2
  - dùng chung `config/slam.yaml`, chỉ override scan topic sang `/scan_local_window` tại runtime

- `submap.launch.py`
  - simulation + segmented submap/ICP frontend + SLAM Toolbox + Nav2
  - dùng `scripts/submap.py`
  - dùng chung `config/slam.yaml`, chỉ override scan topic sang `/scan_submap` tại runtime

Cả ba launch dùng chung `config/nav2.yaml`. New Room là môi trường mặc định của stack hiện tại.
