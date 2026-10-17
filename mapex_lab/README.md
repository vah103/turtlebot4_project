# MapEx Hospital Research Workspace

Workspace riêng cho lộ trình làm lại từ đầu để tìm bài toán nghiên cứu từ điểm yếu của MapEx trên môi trường Hospital.

## Tên chuẩn các native 2D run (cập nhật 2026-10-17)

- **[RUN_REGISTRY.md](RUN_REGISTRY.md)**: bảng tên mới ↔ tên run cũ, trạng thái hoàn thành, nguồn Git exact-ref và giới hạn so sánh.
- **[RUN_REGISTRY.csv](RUN_REGISTRY.csv)**: bản tra cứu có thể đọc bằng script; `run_id` mới là tên hiển thị chính thức cho báo cáo mới.
- Ví dụ: New Room Run 04 → **`NR-RN-01`**, Run 05 → **`NR-MX-01`**, Hospital Run 10 → **`HP-RN-01`**, Run 11 → **`HP-MX-01`**.
- Hai “Run 09” không trùng: G2-R4 → **`NR-G2R4-PRED-01`**; G2-R5 → **`NR-G2R5-MCTS-01`**.
- **Không đổi tên dữ liệu/thư mục gốc** hoặc ID của các thí nghiệm đóng băng; muốn truy dữ liệu, tra `source_ref`/`source_path` trong registry.

## Chạy thí nghiệm nhanh

Từ root repo, chạy:

```bash
./mapex_lab/run
```

Runner tương tác sẽ hỏi lần lượt:

1. Chạy `NF` hay `MapEx`.
2. Có lưu record hay không.
3. Nếu có record, nhập tên run.

Runner tự source ROS 2 Jazzy và `ros2_ws/install/setup.bash`, khởi động `launch/slam.launch.py`, chờ `/map` và `/navigate_to_pose` sẵn sàng rồi mới chạy thuật toán.

- NF không record → `scripts/nf_basic.py`
- MapEx không record → `scripts/mapex.py`
- NF có record → `scripts/nf_run.py --runtime-profile slam`
- MapEx có record → `scripts/mapex_run.py --runtime-profile slam`

Tên run được kiểm tra để không ghi đè thư mục đã tồn tại. `Ctrl+C` một lần sẽ dừng runner và yêu cầu `slam.launch.py` tắt simulation / SLAM / Nav2.

## Mục tiêu

Không giả định trước bottleneck. Quy trình nghiên cứu là:

Hospital ổn định → Nearest baseline → MapEx closed-loop → nhiều run → phân tích theo stage → log toàn pipeline → đánh giá từng tầng → oracle/ablation → tìm bottleneck → kiểm tra literature → chọn research gap → proposed method → benchmark.

## Cách một phiên ChatGPT khác tiếp quản

Đọc theo thứ tự:
1. `STATUS.md` — đang làm đến đâu, việc tiếp theo là gì.
2. `RUN_REGISTRY.md` — tên chuẩn các run native 2D và cách truy lại tên cũ.
3. `ROADMAP.md` — toàn bộ lộ trình nghiên cứu.
4. `references/README.md` — index tài liệu tham chiếu.
5. `references/MAPEX_PAPER.md` — paper MapEx gốc và pipeline cần bám theo.
6. `EXPERIMENT_PROTOCOL.md` — các điều kiện phải giữ cố định giữa các run.
7. `docs/DATA_SCHEMA.md` — dữ liệu mỗi run phải lưu như thế nào.
8. `docs/TEACHER_REPORT.md` — cách chuyển kết quả đã xác minh thành báo cáo cho giảng viên.
9. Sau đó mới đọc code liên quan trong `src/`, `scripts/`, `analysis/`.

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
├── run
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
│   ├── slam.launch.py
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

`slam.yaml` là source of truth chung cho các launch sử dụng file cấu hình này. Một số launch chẩn đoán mới hơn có thể merge/override tham số trực tiếp ở runtime.

## Quy tắc dữ liệu

- Code, config, bảng summary nhỏ và figure quan trọng: commit lên GitHub.
- Dữ liệu nặng như rosbag, ảnh hàng nghìn frame, `.npy` lớn, log simulation dài: không commit.
- Mỗi run phải có metadata và metric riêng.
- Không sửa điều kiện thí nghiệm giữa Nearest và MapEx nếu chưa ghi rõ trong `EXPERIMENT_PROTOCOL.md`.