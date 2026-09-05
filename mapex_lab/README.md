# MapEx Hospital Research Workspace

Workspace riêng cho lộ trình làm lại từ đầu để tìm bài toán nghiên cứu từ điểm yếu của MapEx trên môi trường Hospital.

## Mục tiêu

Không giả định trước bottleneck. Quy trình nghiên cứu là:

Hospital ổn định → Nearest baseline → MapEx closed-loop → nhiều run → phân tích theo stage → log toàn pipeline → đánh giá từng tầng → oracle/ablation → tìm bottleneck → kiểm tra literature → chọn research gap → proposed method → benchmark.

## Cách một phiên ChatGPT khác tiếp quản

Đọc theo thứ tự:
1. `STATUS.md` — đang làm đến đâu, việc tiếp theo là gì.
2. `ROADMAP.md` — toàn bộ lộ trình nghiên cứu.
3. `references/README.md` — index tài liệu tham chiếu.
4. `references/MAPEX_PAPER.md` — paper MapEx gốc và pipeline cần bám theo.
5. `EXPERIMENT_PROTOCOL.md` — các điều kiện phải giữ cố định giữa các run.
6. `docs/DATA_SCHEMA.md` — dữ liệu mỗi run phải lưu như thế nào.
7. `docs/TEACHER_REPORT.md` — cách chuyển kết quả đã xác minh thành báo cáo cho giảng viên.
8. Sau đó mới đọc code liên quan trong `src/`, `scripts/`, `analysis/`.

Không cần dựa vào báo cáo cũ để giả định rằng ranking, uncertainty hay visibility là bottleneck.

## Phụ thuộc hiện có trong repo

Workspace này có thể tái sử dụng ROS2 package hiện có tại:

`ros2_ws/src/frontier_exploration/`

Các launch/file cũ chỉ được coi là dependency. Code nghiên cứu mới và kết quả mới phải nằm trong workspace này.

## Báo cáo cho giảng viên

- Repo này là **source of evidence**: code, config, run data, summary, figure, log và quyết định kỹ thuật.
- Google Docs **`TurtleBot4`**, tab **`Báo cáo nghiên cứu MapEx Hospital`**, là bản **teacher-facing**.
- Chỉ đưa kết quả đã xác minh vào báo cáo chính; pilot/debug vẫn lưu trong repo.
- Quy tắc chi tiết: `docs/TEACHER_REPORT.md`.

## Cấu trúc

```text
mapex_lab/
├── AGENTS.md
├── README.md
├── ROADMAP.md
├── STATUS.md
├── EXPERIMENT_PROTOCOL.md
├── .gitignore
├── references/
│   ├── README.md
│   ├── MAPEX_PAPER.md
│   ├── RESEARCH_ROADMAP.md
│   └── RELATED_WORK.md
├── config/
│   ├── nav2.yaml
│   ├── slam.yaml
│   └── mapex.yaml
├── launch/
│   ├── stock.launch.py
│   ├── local.launch.py
│   └── submap.launch.py
├── src/
├── scripts/
├── analysis/
├── ground_truth/
├── experiments/
├── results/
└── docs/
    ├── DATA_SCHEMA.md
    └── TEACHER_REPORT.md
```

`slam.yaml` là source of truth chung cho cả ba launch. `local.launch.py` và `submap.launch.py` chỉ tạo bản runtime tạm thời để đổi `scan_topic` cho frontend tương ứng; không giữ thêm file SLAM config riêng.

## Quy tắc dữ liệu

- Code, config, bảng summary nhỏ và figure quan trọng: commit lên GitHub.
- Dữ liệu nặng như rosbag, ảnh hàng nghìn frame, `.npy` lớn, log simulation dài: không commit.
- Mỗi run phải có metadata và metric riêng.
- Không sửa điều kiện thí nghiệm giữa Nearest và MapEx nếu chưa ghi rõ trong `EXPERIMENT_PROTOCOL.md`.
