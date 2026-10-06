# TurtleBot4 MapEx Research

Repository này hiện được tổ chức quanh **MapEx / early-stopping research**.

## Phần đang dùng

- `mapex_lab/` — workspace nghiên cứu chính: experiment, analysis, ground truth, results, figures, protocol và tài liệu MapEx.
- `run` + `.run_core` — runner hiện tại.
- `ros2_ws/src/frontier_exploration/` — runtime/simulation dependency mà `mapex_lab/launch/slam.launch.py` còn gọi trực tiếp.
- `slam_toolbox/` — SLAM source dùng bởi ROS 2 workspace.
- `scripts/` — helper build/environment còn giữ ở root.
- `third_party/` — external source references/submodules; không phải nơi lưu research result hiện tại.

Chạy experiment từ root:

```bash
./run
```

Hoặc:

```bash
./mapex_lab/run
```

## Historical TurtleBot4 foundation

Các artifact của giai đoạn TurtleBot4 trước khi trọng tâm chuyển sang MapEx đã được gom vào:

```text
archive/legacy_project/
```

Bao gồm metadata Joy cũ, PROJECT_STATUS cũ, Stage 3 navigation benchmark, Stage 4 evidence, map/report/docs/data cũ và CI validator tương ứng.

Archive được giữ để audit/provenance; **không dùng archive như trạng thái nghiên cứu hiện tại**.

## Lưu ý provenance nghiên cứu

Một số experiment MapEx mới hơn được freeze ở exact Git commit/ref thay vì merge toàn bộ vào `main`. Việc dọn root này chỉ thay đổi cách tổ chức phần legacy; không thay đổi các exact frozen experiment commits đã tồn tại.

Khi làm việc với MapEx, bắt đầu từ:

1. `mapex_lab/README.md`
2. `mapex_lab/STATUS.md`
3. `mapex_lab/EXPERIMENT_PROTOCOL.md`
4. exact experiment ref được task/method/result hiện tại chỉ định

## Safety

Tuân thủ `AGENTS.md`. Không gửi lệnh chuyển động tới robot thật nếu chưa có phê duyệt rõ ràng của người vận hành.
