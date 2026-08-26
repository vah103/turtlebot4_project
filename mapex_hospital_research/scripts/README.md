# Scripts

Hiện có:
- `generate_hospital_roi.py`: sinh canonical ROI `hospital_connected_free_v1`, tính denominator + SHA-256 và tự cập nhật protocol/config.
- `research_recorder.py`: recorder dùng chung cho research runs; ghi metadata, `metrics.csv`, `trajectory.csv`, `decisions.csv` trên fixed canvas/ROI.
- `new_run.py`: utility tạo cấu trúc run thủ công.
- `analyze_all.sh`: chạy tổng hợp cơ bản.

ROI `hospital_connected_free_v1` đã được freeze cho `hospital_v1`:
- denominator: `215435`
- SHA-256: `05d45b7aba66cb6dbb71e0406005f4a3e21875901af3ae72164b17f1d3add8d1`

Không dùng `generate_hospital_roi.py --force` sau khi đã bắt đầu baseline. Thay ROI phải tạo ROI ID/protocol mới và chạy lại baseline.

Nearest pilot được chạy bằng launch ở workspace:

```bash
python3 mapex_hospital_research/launch/hospital_nearest.launch.py
```

Mặc định run ID là `nearest_pilot_001`. Launch chạy Hospital + SLAM + Nav2, sau đó bật WFD Nearest và recorder tự động.

Nếu thiếu Hospital structural assets, chạy:

```bash
bash ros2_ws/src/frontier_exploration/scripts/setup_hospital_world_assets.sh
```
