# Scripts

Hiện có:
- `generate_hospital_roi.py`: sinh canonical ROI `hospital_connected_free_v1`, tính denominator + SHA-256 và tự cập nhật protocol/config.
- `new_run.py`: tạo cấu trúc run chuẩn.
- `analyze_all.sh`: chạy tổng hợp cơ bản.

Generate/freeze ROI trước `nearest_001`:

```bash
cd ~/turtlebot4_project
python3 mapex_hospital_research/scripts/generate_hospital_roi.py
```

Nếu thiếu Hospital structural assets, chạy trước:

```bash
bash ros2_ws/src/frontier_exploration/scripts/setup_hospital_world_assets.sh
```

Không dùng `--force` sau khi đã có baseline run. Thay ROI sau đó phải tạo ROI ID/protocol mới và chạy lại baseline.

Planned sau khi Stage 1 chốt protocol:
- `run_nearest.sh`
- `run_mapex.sh`
- wrapper để khởi chạy recorder/logger đồng nhất giữa hai phương pháp.

Không tạo script chạy closed-loop chính thức trước khi chốt `EXPERIMENT_PROTOCOL.md`, để tránh baseline và MapEx dùng điều kiện khác nhau.
