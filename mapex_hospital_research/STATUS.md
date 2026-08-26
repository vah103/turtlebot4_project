# STATUS

## Current stage

**Stage 2 — MapEx Nearest baseline (final pilot before official runs)**

## Done

- Hospital simulation + SLAM + Nav2 ổn định.
- Fixed canvas `hospital_canvas_v1`: `0.05 m`, `1504 x 2123`, origin `(-25.6,-60.1)`.
- Frozen ROI `hospital_connected_free_v1`: denominator `215435`, SHA-256 `05d45b7aba66cb6dbb71e0406005f4a3e21875901af3ae72164b17f1d3add8d1`.
- Official MapEx source pinned: `53636bd1c79153acc3c74a532837d78c926bae5e`.
- Nearest frontier generation/ranking port: 8-neighbour frontier, 8-connected regions, region `>10`, representative gần arithmetic mean nhất, Euclidean ranking.
- Nav2 `ComputePathToPose` + `NavigateToPose` thay simulator pyastar execution; không shift/standoff goal.
- `nearest_pilot_005` chứng minh original MapEx 1 m rejection gây startup deadlock trên Hospital (candidate duy nhất `0.469 m`).
- `hospital_v1` bỏ 1 m rejection cho mọi Hospital methods; `below_1m` chỉ là diagnostic flag.
- Replay-safe logging đã có exact decision raw/canvas map, map-frame pose, full candidates/ranks/status, Nav2 path timing/outcome, periodic maps 10 s + final, navigation failure reason.
- Thêm final official-run policy wrapper `scripts/mapex_nearest_ros_official.py`:
  - benchmark clock bắt đầu **trước candidate computation đầu tiên**;
  - lưu exact exhausted state khi `candidate_count=0`;
  - thêm stable `candidate_id`;
  - publish selected candidate ID để join trực tiếp `decisions.csv ↔ candidates.csv`.
- Terminal Nav2 state được ghi trước completion window với `nav2_*_count` và `terminal_reason`.
- Phát hiện execution-adapter bug trong `nearest_pilot_007`: `exploration_manager` dùng `path.poses[-1]` làm NavigateToPose goal. Planner tolerance `0.5 m` có thể snap endpoint gần robot, làm Nav2 báo success dù chưa tới exact frontier.
- Đã sửa `scripts/exploration_manager_research.py`:
  - `ComputePathToPose` path chỉ là reachability evidence;
  - manager chờ cả validated path + `/frontier_selected`;
  - `NavigateToPose` dùng **exact MapEx frontier center**;
  - planner endpoint và offset tới frontier chỉ được log để audit.
- Đã override `planner_server.GridBased.tolerance: 0.0` trong `nav2_hospital_override.yaml`, để cả path validation và planner bên trong NavigateToPose không snap goal tới điểm cách frontier 0.5 m.
- `research_recorder_official.py` tách `planner_endpoint_*` khỏi `goal_*`; `goal_source=exact_frontier_center`.
- Validator giờ FAIL official run nếu execution goal không trùng exact frontier center.
- Preflight kiểm tra cả **installed** Hospital Nav2 override có `tolerance: 0.0`; nếu stale phải rebuild `frontier_exploration`.
- Thêm final recorder wrapper `scripts/research_recorder_official.py`:
  - đồng bộ `t=0` từ policy pre-compute timestamp;
  - hash cả source research code và installed runtime files thực tế;
  - ghi rõ simulator seed policy là Gazebo default intentionally uncontrolled + repeated-run statistics.
- Validator được siết: snapshot integrity, candidate join, exact-frontier execution goal, no `PENDING` sau termination, failure reason, terminal Nav2 audit, metrics time/distance sanity, coverage bounds, provenance, clean git cho official runs.
- `DATA_SCHEMA.md` đã đồng bộ Hospital below-1m semantics, candidate IDs, exact execution goal và planner-endpoint audit.
- `ROADMAP.md` đã bỏ wording custom WFD, chuyển sang shared MapEx frontier candidate set.
- `EXPERIMENT_PROTOCOL.md` đã chốt benchmark clock, seed policy, exact exhausted state, exact-frontier execution semantics và provenance.

## In progress

- Chạy `nearest_pilot_008` như final runtime check sau exact-frontier + zero-planner-tolerance fix.
- Xác nhận robot thực sự di chuyển tới selected frontier, không còn vòng lặp success tại tolerance-snapped planner endpoint.
- Xác nhận strict validator có dòng `exact frontier execution goal: OK`.

## Next actions

1. Pull code trên Ubuntu.
2. Giữ `nearest_pilot_007` cũ làm diagnostic evidence; không dùng cho benchmark.
3. Rebuild `frontier_exploration` để installed `nav2_hospital_override.yaml` nhận `tolerance: 0.0`.
4. Chạy `scripts/preflight_nearest.py`; phải PASS toàn bộ invariant, gồm installed Nav2 exact-planner tolerance.
5. Chạy `hospital_nearest.launch.py run_id:=nearest_pilot_008`.
6. Sau vài decision, chạy validator `--run-id nearest_pilot_008 --allow-running`.
7. Kiểm tra log: `Sending autonomous frontier goal` phải trùng exact selected frontier; planner endpoint được log riêng.
8. Để pilot kết thúc tự nhiên nếu có thể; chạy validator cuối không có `--allow-running`.
9. Chỉ khi pilot_008 PASS mới khóa code và chạy `nearest_001 ... nearest_010`.
10. Khi Nearest official hoàn thành, port full MapEx `visvarprob` dùng cùng frontier generation, below-1m adaptation, exact-frontier execution, Nav2, benchmark clock và logging schema.

## Important decisions

- Hospital implementation không được gọi là MapEx chạy nguyên xi: frontier semantics/ranking được port, execution adapt sang Nav2.
- Base/reference adapter giữ documented original 1 m semantics để đối chiếu; Hospital benchmark runtime không enforce rule này.
- Nearest, MapEx và proposed method phải chia sẻ cùng Hospital adaptation, exact-frontier execution semantics và benchmark-clock definition.
- `ComputePathToPose` path endpoint không phải execution goal; exact `/frontier_selected` center mới là NavigateToPose goal.
- Hospital planner tolerance dùng `0.0 m`; candidate không lập được path tới exact frontier thì thử candidate rank tiếp theo.
- `t=0` là first policy decision before computation, không phải first selected path.
- Exact no-candidate termination state phải được lưu như policy decision.
- Official run phải clean git và metadata phải hash installed runtime files thực tế.
- Simulator seed intentionally uncontrolled; variability xử lý bằng repeated runs.
- `coverage` trên frozen ROI là exploration metric chính; `known_fraction` là progress/debug proxy.
- Pilot/debug không được dùng như official benchmark result.

## Latest result

`nearest_pilot_005` là diagnostic pilot cho incompatibility của MapEx 1 m rule. `nearest_pilot_007` phát hiện execution bug do tolerance-snapped planner endpoint bị dùng thay exact frontier; run đó không hợp lệ cho benchmark. `nearest_pilot_008` là final runtime check cho exact-frontier execution + planner tolerance `0.0` trước official Nearest runs.
