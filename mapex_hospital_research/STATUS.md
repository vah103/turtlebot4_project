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
- Generate/freeze ROI trên Ubuntu: denominator `215435`, SHA-256 `05d45b7aba66cb6dbb71e0406005f4a3e21875901af3ae72164b17f1d3add8d1`, `1040` unique wall segments, `2` elevator blockers.
- Gán protocol `hospital_v1`.
- Xác minh official MapEx source tại `castacks/MapEx` commit `53636bd1c79153acc3c74a532837d78c926bae5e`.
- Bỏ WFD tự viết khỏi research Nearest baseline trước official runs.
- Port official MapEx `nearest` policy sang `scripts/mapex_nearest_ros.py`: 8-neighbour frontier, 8-connected region, region `>10`, center gần arithmetic mean nhất, Euclidean score, `cur_pose_dist_threshold_m=1.0`, no-path -> thử next candidate.
- Chỉ thay execution layer: `pyastar2d A*` -> Nav2 `ComputePathToPose` + `NavigateToPose`; không dùng WFD standoff/goal shifting.
- `nearest_pilot_001` với MapEx-nearest adapter đã chạy nhưng **INVALID**:
  - adapter khởi động đúng;
  - không frontier goal nào được chọn;
  - robot distance `0.00 m`;
  - coverage chỉ `0.0053473`;
  - wrapper cũ gán nhầm trạng thái này thành `exploration_complete`;
  - recorder sau finalize vẫn nhận `/odom` và crash với `ValueError: I/O operation on closed file`.
- Thêm `scripts/mapex_nearest_ros_hospital.py`:
  - giữ nguyên pinned MapEx policy;
  - log audit `free/unknown/frontier_cells/region_count/max_region/large_regions/centers>=1m/usable_candidates`;
  - startup không có usable frontier được ghi là **policy failure**, không phải successful completion.
- Thêm `scripts/research_recorder_safe.py` để mọi callback no-op sau finalize và ghi `/exploration_failed` vào metadata.
- `hospital_nearest.launch.py` chuyển sang hai safe/audit wrapper trên; default pilot tiếp theo là `nearest_pilot_002`.

## In progress

- Chạy `nearest_pilot_002` để xác định chính xác adapter đang bị chặn ở bước nào: raw frontier, threshold region `>10`, threshold distance `1 m`, hay Nav2 path validation.
- Chưa thay bất kỳ policy parameter MapEx nào cho tới khi diagnostic xác nhận nguyên nhân.

## Next actions

1. Pull code mới trên Ubuntu.
2. Syntax-check `mapex_nearest_ros.py`, `mapex_nearest_ros_hospital.py`, `research_recorder_safe.py`, `hospital_nearest.launch.py`.
3. Chạy `nearest_pilot_002`.
4. Đọc dòng `MapEx frontier audit:` và nếu có `MapEx-nearest startup policy failure:` để xác định incompatibility chính xác.
5. Chỉ sau khi pilot có ít nhất một selected frontier + robot motion mới xét official Nearest runs.
6. Khi Nearest ổn định, port MapEx `visvarprob` closed-loop trên cùng frontier-generation semantics và `hospital_v1`.

## Latest result

`nearest_pilot_001` không phải kết quả benchmark: robot không di chuyển và không có selected frontier. Runtime đã được sửa để lần kế tiếp phân biệt rõ policy failure với true exploration completion và không còn recorder closed-file crash.

## Important decisions

- Baseline `nearest` phải có decision policy từ official MapEx code, không dùng WFD tự viết để tuyên bố so sánh với MapEx.
- Không gọi ROS adapter là “MapEx code chạy nguyên xi”: frontier policy/ranking được port trực tiếp, simulator A* execution được thay bằng Nav2.
- Không tự ý nới `region_size>10` hay `1.0 m` sau pilot lỗi; phải diagnostic trước và nếu cần adaptation thì ghi rõ đó là ROS/Hospital adaptation dùng giống nhau cho Nearest và MapEx.
- Nearest và MapEx sau này phải chia sẻ cùng frontier-generation semantics; khác biệt nghiên cứu chính nằm ở score/ranking.
- `known_fraction` là progress/debugging proxy trên fixed canvas; `coverage` là exploration metric chính trên frozen ROI.
- ROI denominator `215435` và SHA-256 ở trên không được thay đổi trong `hospital_v1`.
- Pilot/debug không được trình bày như official benchmark result hay teacher-facing result.
- Dữ liệu nặng không commit lên GitHub.

## Handoff rule

Mỗi khi hoàn thành một bước lớn, cập nhật file này trước khi kết thúc phiên làm việc. Nếu milestone có kết quả đủ tin cậy, đồng thời cập nhật Google Docs teacher report theo `docs/TEACHER_REPORT.md`.
