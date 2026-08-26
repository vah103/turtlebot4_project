# MapEx Hospital Research Workspace

Workspace riêng cho lộ trình làm lại từ đầu để tìm bài toán nghiên cứu từ điểm yếu của MapEx trên môi trường Hospital.

## Mục tiêu

Không giả định trước bottleneck. Quy trình nghiên cứu là:

Hospital ổn định → Nearest baseline → MapEx closed-loop → nhiều run → phân tích theo stage → log toàn pipeline → đánh giá từng tầng → oracle/ablation → tìm bottleneck → kiểm tra literature → chọn research gap → proposed method → benchmark.

## Cách một phiên ChatGPT khác tiếp quản

Đọc theo thứ tự:
1. `STATUS.md` — đang làm đến đâu, việc tiếp theo là gì.
2. `ROADMAP.md` — toàn bộ lộ trình nghiên cứu.
3. `EXPERIMENT_PROTOCOL.md` — các điều kiện phải giữ cố định giữa các run.
4. `docs/DATA_SCHEMA.md` — dữ liệu mỗi run phải lưu như thế nào.
5. Sau đó mới đọc code liên quan trong `src/`, `scripts/`, `analysis/`.

Không cần dựa vào báo cáo cũ để giả định rằng ranking, uncertainty hay visibility là bottleneck.

## Phụ thuộc hiện có trong repo

Workspace này có thể tái sử dụng ROS2 package hiện có tại:

`ros2_ws/src/frontier_exploration/`

Các launch/file cũ chỉ được coi là dependency. Code nghiên cứu mới và kết quả mới phải nằm trong workspace này.

## Cấu trúc

```text
mapex_hospital_research/
├── README.md
├── ROADMAP.md
├── STATUS.md
├── EXPERIMENT_PROTOCOL.md
├── .gitignore
├── config/
├── launch/
├── src/
├── scripts/
├── analysis/
├── ground_truth/
├── experiments/
├── results/
└── docs/
```

## Quy tắc dữ liệu

- Code, config, bảng summary nhỏ và figure quan trọng: commit lên GitHub.
- Dữ liệu nặng như rosbag, ảnh hàng nghìn frame, `.npy` lớn, log simulation dài: không commit.
- Mỗi run phải có metadata và metric riêng.
- Không sửa điều kiện thí nghiệm giữa Nearest và MapEx nếu chưa ghi rõ trong `EXPERIMENT_PROTOCOL.md`.
