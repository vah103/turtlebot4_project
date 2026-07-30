# Stage 3 — Navigation Benchmark

Thư mục này chứa baseline Nav2 định lượng được hoàn thành trên TurtleBot4 thật ngày 30/07/2026.

## Kết quả chính thức

- Protocol: 3 rounds × 4 fixed goals
- Trials: 12
- Successes: 12
- Success rate: 100%
- Recoveries: 0
- Total travel time: 101.11 s
- Mean travel time: 8.43 s/goal
- Total path length: 20.62 m
- Mean path length: 1.72 m/goal
- Rosbag: 383.514 s, 43,542 messages

## Cấu trúc cần version

```text
config/
  benchmark_goals.yaml
  lab_map.yaml
  lab_map.pgm
scripts/
  navigation_benchmark.py
runs/
  stage3_20260730_official_02/
    baseline.sha256
    benchmark_goals.yaml
    benchmark_results_20260730_151159.csv
    benchmark_terminal.log
    lab_map.yaml
    lab_map.pgm
    rosbag_info.txt
    rosbag/
      metadata.yaml
    rosbag_terminal.log
    summary.txt
```

## Không đưa trực tiếp vào Git thường

Các payload rosbag lớn bị loại bởi `.gitignore`:

```text
*.mcap
*.db3
```

Khi cần lưu lâu dài, dùng Git LFS hoặc kho lưu trữ ngoài Git. `rosbag_info.txt` và `metadata.yaml` vẫn nên được commit để giữ metadata của phiên chạy.

## Chạy benchmark

```bash
cd ~/turtlebot4_project/stage3_navigation_benchmark
python3 scripts/navigation_benchmark.py \
  --goals config/benchmark_goals.yaml
```

Chỉ chạy khi robot thật ở khu vực an toàn, có người vận hành tại lab, Localization và Nav2 đã active.

## Kiểm tra bằng chứng

```bash
ros2 bag info \
  runs/stage3_20260730_official_02/rosbag

cd runs/stage3_20260730_official_02
sha256sum -c baseline.sha256
```

Báo cáo đầy đủ: `../report/2026-07-30.md`.
