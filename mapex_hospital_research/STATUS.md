# STATUS

## Current stage

**Stage 2 — MapEx Nearest baseline (pilot + logging validation)**

## Done

- Tạo workspace `mapex_hospital_research/` và chốt research workflow failure → bottleneck → oracle → research gap.
- Hospital simulation + SLAM + Nav2 ổn định.
- Fixed canvas `hospital_canvas_v1`: `0.05 m`, `1504 x 2123`, origin `(-25.6, -60.1)`.
- Frozen ROI `hospital_connected_free_v1`: denominator `215435`; SHA-256 `05d45b7aba66cb6dbb71e0406005f4a3e21875901af3ae72164b17f1d3add8d1`.
- Official MapEx source pinned tại commit `53636bd1c79153acc3c74a532837d78c926bae5e`.
- Nearest frontier generation/ranking port theo MapEx: 8-neighbour frontier, 8-connected regions, region `>10`, representative gần arithmetic mean nhất, Euclidean ranking.
- ROS execution adapter: Nav2 `ComputePathToPose` + `NavigateToPose`; không shift/standoff goal.
- `nearest_pilot_001` INVALID: robot không di chuyển, false completion và recorder closed-file crash.
- Replay-safe logging đã có:
  - exact frozen decision raw map + fixed-canvas map;
  - map-frame pose;
  - full candidate/rank/status;
  - Nav2 path result/timing;
  - selected path length;
  - detailed navigation failure reason;
  - periodic map mỗi 10 s + final map;
  - provenance/config hashes.
- `nearest_pilot_005` xác nhận MapEx original `1.0 m` locked-frontier rejection gây **startup deadlock trên Hospital**:
  - frontier cells `534`;
  - regions `24`;
  - large regions `1`;
  - ranked representative `1`;
  - representative distance `0.469 m`;
  - `>=1m_valid=0`;
  - candidate duy nhất bị reject trước Nav2, robot không thể bắt đầu.
- Protocol decision: `hospital_v1` **không enforce MapEx 1 m rejection**. Candidate `<1m` vẫn giữ rank và được gửi sang Nav2 path validation. Cờ `below_1m` vẫn được log.
- Thêm `scripts/mapex_nearest_ros_hospital_adapted.py` và chuyển Hospital launch sang wrapper này.
- Adaptation `<1m allowed` phải dùng giống nhau cho Nearest, full MapEx Hospital và proposed method để fairness.
- Default pilot mới: `nearest_pilot_006`.

## In progress

- Chạy `nearest_pilot_006` để xác nhận candidate đầu tiên `0.469 m` (hoặc tương tự) thực sự được gửi sang Nav2 và robot bắt đầu di chuyển.
- Validate replay-safe logging và termination sau khi bỏ 1 m rejection.

## Next actions

1. Pull code mới trên Ubuntu.
2. Xóa run dở `nearest_pilot_005` nếu không cần giữ local.
3. Chạy `scripts/preflight_nearest.py`; phải thấy `Hospital below-1m bypass: ACTIVE`.
4. Chạy `hospital_nearest.launch.py` (`nearest_pilot_006`).
5. Trong log phải thấy `Hospital adaptation allowing ranked frontier below 1 m` thay vì `Rejecting ... by 1.0 m validity rule`.
6. Sau vài decision chạy `validate_nearest_run.py --run-id nearest_pilot_006 --allow-running`.
7. Chỉ khi robot motion + validator pass mới bắt đầu `nearest_001 ... nearest_010` official.
8. Khi Nearest ổn định, port MapEx `visvarprob` dùng cùng frontier generation, below-1m adaptation, Nav2 execution và logging protocol.

## Latest result

`nearest_pilot_005` không phải benchmark; nó là diagnostic pilot chứng minh rule 1 m gốc không tương thích với startup frontier geometry của Hospital. Không có lỗi Nav2 ở decision đầu vì candidate bị reject trước khi planner được gọi.

## Important decisions

- Không gọi Hospital implementation là MapEx code chạy nguyên xi; frontier semantics/ranking được port, simulator execution được adapt sang Nav2.
- Base/reference adapter vẫn giữ documented MapEx original 1 m semantics để đối chiếu.
- Hospital benchmark runtime bỏ 1 m rejection; đây là protocol adaptation có bằng chứng pilot và phải giống nhau giữa methods.
- Exact decision map phải do policy process lưu từ frozen `decision_map`.
- `coverage` trên frozen ROI là exploration metric chính; `known_fraction` là progress/debug proxy.
- ROI denominator/hash không được đổi trong `hospital_v1`.
- Pilot/debug không được trình bày như official benchmark result.
- Dữ liệu nặng giữ local, không commit lên GitHub.

## Handoff rule

Mỗi khi hoàn thành một bước lớn, cập nhật file này trước khi kết thúc phiên làm việc.
