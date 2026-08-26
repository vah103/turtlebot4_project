# STATUS

## Current stage

**Stage 2 — MapEx Nearest baseline (pilot + logging validation)**

## Done

- Tạo workspace `mapex_hospital_research/`.
- Chốt lộ trình nghiên cứu theo hướng: failure → bottleneck → oracle → research gap.
- Chốt semantic của `known_fraction` và `coverage` trong `docs/DATA_SCHEMA.md`.
- Xác nhận `hospital_flat_stack.launch.py` chạy ổn định simulation + SLAM + Nav2.
- Chốt fixed logging canvas `hospital_canvas_v1`: `0.05 m`, `1504 x 2123`, origin `(-25.6, -60.1)`.
- Chốt canonical evaluation ROI `hospital_connected_free_v1` là 8-connected structural free space chứa robot start, loại obstacle/disconnected/outside/padding.
- Generate/freeze ROI trên Ubuntu:
  - denominator `215435` cells
  - SHA-256 `05d45b7aba66cb6dbb71e0406005f4a3e21875901af3ae72164b17f1d3add8d1`
  - `1040` unique wall segments
  - `2` elevator blockers
- Gán protocol `hospital_v1`.
- Xác minh official MapEx source tại `castacks/MapEx` commit `53636bd1c79153acc3c74a532837d78c926bae5e`.
- Bỏ WFD tự viết khỏi research Nearest baseline trước khi có official run.
- Port official MapEx `nearest` policy sang `scripts/mapex_nearest_ros.py`:
  - free cell kề unknown theo 8-neighbour;
  - frontier region 8-connected;
  - chỉ giữ region `> 10` cells;
  - representative = frontier cell gần arithmetic mean của region nhất;
  - score = Euclidean distance tới robot;
  - `cur_pose_dist_threshold_m = 1.0`;
  - no-path candidate -> thử candidate có cost thấp tiếp theo.
- Chỉ thay execution layer của MapEx: `pyastar2d A*` -> Nav2 `ComputePathToPose`, rồi TurtleBot4 `NavigateToPose`; không dùng standoff/goal shifting.
- `launch/hospital_nearest.launch.py` hiện chạy MapEx-nearest adapter + existing exploration manager; không còn include `frontier_autonomy.launch.py`.
- Recorder ghi policy source repo/commit và số frontier candidates của mỗi decision.
- Chốt per-goal hard timeout `180 s`, stall timeout `30 s` và common stabilized completion wrapper `5 cycles / 2 s / 10 s idle / 20 s startup grace`.

## In progress

- Chạy lại `nearest_pilot_001` với **MapEx official nearest policy** để xác nhận ROS adapter, Nav2 path validation, recorder và termination.
- Kiểm tra rằng frontier center/selection thực tế khớp policy đã pin trước khi chạy official runs.

## Next actions

1. Pull code mới trên Ubuntu.
2. Syntax-check `scripts/mapex_nearest_ros.py` và `launch/hospital_nearest.launch.py`.
3. Chạy `nearest_pilot_001`.
4. Kiểm tra terminal + `metadata.json`, `metrics.csv`, `trajectory.csv`, `decisions.csv`.
5. Nếu pilot hợp lệ, chạy minimum 5, target 10 official Nearest runs.
6. Sau đó port MapEx `visvarprob` closed-loop trên **cùng frontier-generation policy và hospital_v1**.

## Latest result

Nearest research implementation đã được thay từ WFD custom sang official MapEx nearest frontier policy. Run WFD trước đó bị dừng/xóa và không được dùng làm kết quả. Chưa có official Nearest benchmark run.

## Important decisions

- Baseline `nearest` phải có decision policy từ official MapEx code, không dùng WFD tự viết để tuyên bố so sánh với MapEx.
- Không gọi ROS adapter là “MapEx code chạy nguyên xi”: frontier policy/ranking được port trực tiếp, nhưng simulator A* execution được thay bằng Nav2 để chạy TurtleBot4 Hospital.
- Nearest và MapEx sau này phải chia sẻ cùng MapEx frontier-generation semantics; khác biệt nghiên cứu chính nằm ở score/ranking.
- Không dùng kết luận của báo cáo cũ làm giả định ban đầu.
- MapEx phải được chạy closed-loop trước khi kết luận failure.
- `known_fraction` là progress/debugging proxy trên fixed logging canvas, denominator cố định.
- `coverage` là exploration metric chính trên frozen ROI `hospital_connected_free_v1`.
- ROI denominator `215435` và SHA-256 ở trên không được thay đổi trong `hospital_v1`.
- Stage chính dùng absolute exploration state; không normalize final state riêng của từng run thành 100%.
- Tại cùng coverage stage, coverage chỉ dùng để căn chỉnh trạng thái; so time/distance-to-stage, goal outcomes và diagnostic metrics.
- `hospital_v1` không dùng fixed time/distance budget làm controller termination; common budget có thể được chọn offline sau pilot nếu cần cho secondary analysis.
- Dữ liệu nặng không commit lên GitHub.
- Google Docs teacher report chỉ nhận milestone/kết quả đã xác minh; pilot/debug không được trình bày như kết quả chính thức.

## Handoff rule

Mỗi khi hoàn thành một bước lớn, cập nhật file này trước khi kết thúc phiên làm việc. Nếu milestone có kết quả đủ tin cậy, đồng thời cập nhật Google Docs teacher report theo `docs/TEACHER_REPORT.md`.
