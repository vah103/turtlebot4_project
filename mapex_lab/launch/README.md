# Launch

Ba launch chính của `mapex_lab`:

- `stock.launch.py`
  - simulation + SLAM Toolbox + Nav2
  - không có local scan frontend

- `local.launch.py`
  - simulation + local scan frontend + SLAM Toolbox + Nav2

- `submap.launch.py`
  - simulation + segmented submap/ICP frontend + SLAM Toolbox + Nav2
  - dùng `scripts/submap.py` và `config/slam_submap.yaml`

New Room là môi trường mặc định của stack hiện tại.
