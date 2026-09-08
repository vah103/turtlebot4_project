# Launch

Các launch chính của `mapex_lab`:

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

- `toolbox.launch.py`
  - conservative SLAM Toolbox A/B baseline
  - adaptive Ceres code tồn tại trong binary nhưng profile này tắt bằng `adaptive_anchor_enabled=false`

- `new_toolbox.launch.py`
  - Adaptive Temporal Anchor V1 diagnostic
  - `scan_buffer_size=30`
  - strict sequential temporal weighting và strong-loop regional release

- `oldmap_toolbox.launch.py`
  - Old-Map-First V2 diagnostic
  - weighted sequential scan matching: recent buffer + nearby trusted historical keyframes, old scan > new scan
  - first pose hard-fixed; local Ceres edges `node_gap<=5` dùng temporal weight `5x -> 1x`
  - loop edges `1x`, temporal release disabled
  - cần rebuild vendored `slam_toolbox` sau khi pull vì V2 có thay đổi C++

Các launch vẫn dùng chung Nav2 override `config/nav2.yaml` khi phù hợp. New Room là môi trường mặc định của stack hiện tại. Các SLAM diagnostic không tự động thay đổi official `hospital_v2` protocol.
