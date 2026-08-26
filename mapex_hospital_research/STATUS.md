# STATUS

## Current stage

**Stage 2 — MapEx Nearest baseline (pilot + logging validation)**

## Done

- Tạo workspace `mapex_hospital_research/`.
- Chốt lộ trình nghiên cứu theo hướng: failure → bottleneck → oracle → research gap.
- Chốt semantic của `known_fraction` và `coverage` trong `docs/DATA_SCHEMA.md`.
- Xác nhận `hospital_flat_stack.launch.py` chạy ổn định simulation + SLAM + Nav2.
- Chốt fixed logging canvas `hospital_canvas_v1`: `0.05 m`, `1504 x 2123`, origin `(-25.6, -60.1)`.
- Chốt canonical evaluation ROI `hospital_connected_free_v1`; denominator `215435`; SHA-256 `05d45b7aba66cb6dbb71e0406005f4a3e21875901af3ae72164b17f1d3add8d1`.
- Gán protocol `hospital_v1`.
- Xác minh official MapEx source tại `castacks/MapEx` commit `53636bd1c79153acc3c74a532837d78c926bae5e`.
- Bỏ WFD tự viết khỏi research Nearest baseline.
- Port official MapEx `nearest` policy: 8-neighbour frontier, 8-connected region, region `>10`, representative gần arithmetic mean nhất, Euclidean score, no-path -> thử next candidate.
- Chỉ thay execution layer: `pyastar2d A*` -> Nav2 `ComputePathToPose` + `NavigateToPose`; không dùng standoff/goal shifting.
- Chốt đúng semantic `cur_pose_dist_threshold_m=1.0`: **rank toàn bộ frontier trước; candidate đang được chọn nếu <1 m thì reject; sau đó thử candidate có rank tiếp theo; rồi mới Nav2 path validation**.
- `nearest_pilot_001` INVALID: không selected frontier, distance `0.00 m`, coverage `0.0053473`, false completion và recorder closed-file crash.
- Thêm `scripts/mapex_nearest_ros_hospital.py` để audit frontier, RViz markers và startup failure semantics.
- Thêm `scripts/mapex_nearest_ros_research.py` để lưu exact frozen policy state tại mỗi decision:
  - raw OccupancyGrid `.npz`;
  - exact fixed-canvas `.npz`;
  - map-frame robot pose;
  - full candidate set/ranking;
  - `<1m` rejection status;
  - Nav2 path validation status;
  - planner timing;
  - selected path length;
  - execution success/failure.
- Thêm `scripts/exploration_manager_research.py`: navigation behaviour giữ nguyên, chỉ publish detailed execution result (`success/rejected/timeout/stall/status`).
- Nâng `scripts/research_recorder_safe.py`:
  - callback no-op sau finalize;
  - link `decisions.csv` với `policy_decision_id`;
  - thêm `coverage`, map-frame pose, navigation detail/failure reason;
  - periodic raw + fixed-canvas compressed snapshots mỗi `10 s` + final map để tính IoU/TU offline;
  - lưu git dirty state và SHA-256 của code/config quan trọng.
- `hospital_nearest.launch.py` hiện chạy replay-safe policy + manager + recorder bằng một lệnh; default pilot mới: `nearest_pilot_005`.

## In progress

- Validate syntax/runtime của bộ logging mới trên Ubuntu.
- Chạy `nearest_pilot_005` như **pilot cuối trước official runs**.
- Xác minh robot vẫn di chuyển bình thường và dữ liệu được sinh đầy đủ mà không ảnh hưởng policy/navigation.

## Next actions

1. Pull code mới trên Ubuntu.
2. Xóa folder pilot dở nếu muốn dùng lại run ID cũ; khuyến nghị giữ `nearest_pilot_005` mới để tránh overwrite.
3. `py_compile` toàn bộ runtime mới.
4. Chạy `nearest_pilot_005`.
5. Sau vài decision, kiểm tra tối thiểu:
   - `policy_decisions.csv`;
   - `candidates.csv`;
   - `decisions/policy_decision_*/decision.json`;
   - `observed_map_raw.npz` + `observed_map_canvas.npz`;
   - `decisions.csv` có `policy_decision_id`, `coverage`, map-frame pose;
   - `snapshots.csv` + periodic/final map;
   - failure reason nếu goal fail.
6. Chỉ khi pilot này pass mới bắt đầu `nearest_001 ... nearest_010` official.
7. Khi Nearest ổn định, port MapEx `visvarprob` closed-loop trên cùng frontier generation + execution/logging protocol.

## Latest result

Robot ở pilot gần nhất đã có thể chạy/khám phá, nhưng run đó bị dừng để bổ sung logging trước khi benchmark chính thức. Chưa dùng bất kỳ pilot nào làm kết quả benchmark.

## Important decisions

- Baseline `nearest` phải dùng decision policy từ official MapEx code.
- Không gọi ROS adapter là “MapEx code chạy nguyên xi”: policy được port, simulator A* execution được thay bằng Nav2.
- Rule `1.0 m` không phải frontier-detection filter và không được áp dụng trước ranking; nó là post-ranking locked-frontier validity rejection.
- Exact decision map phải do policy process lưu từ `decision_map` đã freeze; recorder không được lấy một `latest_map` muộn hơn rồi gọi đó là decision snapshot.
- Nearest và MapEx sau này phải chia sẻ cùng frontier-generation semantics và execution adapter; khác biệt nghiên cứu chính nằm ở score/ranking/prediction pipeline.
- `known_fraction` là debugging/progress proxy; `coverage` trên frozen ROI là exploration metric chính.
- ROI denominator/hash không được đổi trong `hospital_v1`.
- Pilot/debug không được trình bày như official benchmark result.
- Dữ liệu nặng `.npz`, raw maps, logs giữ local; không commit lên GitHub.

## Handoff rule

Mỗi khi hoàn thành một bước lớn, cập nhật file này trước khi kết thúc phiên làm việc. Nếu milestone có kết quả đủ tin cậy, đồng thời cập nhật teacher-facing report theo `docs/TEACHER_REPORT.md`.
