# STATUS

## Current stage

**Stage 2 — MapEx Nearest baseline (mapping-stability pilot before official runs)**

## Done

- Fixed canvas `hospital_canvas_v1`: `0.05 m`, `1504 x 2123`, origin `(-25.6,-60.1)`.
- Frozen ROI `hospital_connected_free_v1`: denominator `215435`, SHA-256 `05d45b7aba66cb6dbb71e0406005f4a3e21875901af3ae72164b17f1d3add8d1`.
- Official MapEx source pinned: `53636bd1c79153acc3c74a532837d78c926bae5e`.
- Nearest frontier generation/ranking port: 8-neighbour frontier, 8-connected regions, region `>10`, representative gần arithmetic mean nhất, Euclidean ranking.
- `nearest_pilot_005` chứng minh original MapEx 1 m rejection gây startup deadlock trên Hospital; `hospital_v1` bỏ 1 m rejection và chỉ log `below_1m` diagnostic.
- Exact replay logging: frozen decision maps, map-frame pose, full candidate/rank/status, candidate IDs, planner timing/outcomes, periodic/final maps, detailed navigation result.
- Benchmark clock bắt đầu trước candidate computation đầu tiên.
- Terminal Nav2 state được ghi trước completion window với `nav2_*_count` và `terminal_reason`.
- `nearest_pilot_007` phát hiện execution bug: planner endpoint tolerance-snapped bị dùng làm NavigateToPose goal.
- Đã sửa execution adapter:
  - `ComputePathToPose` path chỉ là reachability evidence;
  - manager chờ validated path + `/frontier_selected`;
  - `NavigateToPose` dùng exact MapEx frontier center;
  - planner endpoint chỉ log audit.
- Đã override `planner_server.GridBased.tolerance: 0.0` để candidate phải planner-reachable tới exact frontier.
- Recorder tách `planner_endpoint_*` khỏi actual `goal_*`; validator yêu cầu `goal_source=exact_frontier_center` và goal trùng frontier.
- `nearest_pilot_008` live validator PASS exact-frontier execution, candidate linkage, snapshots và provenance.
- `nearest_pilot_008` đồng thời lộ ra **long-run occupancy-map warping**: wall/hành lang bị méo dù TF chain đứng yên ổn định khi robot dừng. Run này chỉ dùng diagnostic, không benchmark.
- Đã chốt mapping-stability profile trước official runs:
  - Nav2 max linear speed `0.45 m/s` (từ `0.75 m/s`);
  - SLAM `minimum_travel_distance=0.10 m` (từ `0.20 m`);
  - SLAM `minimum_travel_heading=0.10 rad` (từ `0.20 rad`);
  - giữ nguyên matcher/loop-closure thresholds để không nới false-closure acceptance trong hành lang lặp lại.
- Preflight kiểm tra cả **source và installed** Nav2/SLAM profile, bao gồm exact planner tolerance, speed `0.45`, SLAM spacing `0.10/0.10`.
- Protocol/config đã đồng bộ mapping-stability profile; pilot mặc định chuyển sang `nearest_pilot_009`.

## In progress

- Chạy `nearest_pilot_009` để kiểm tra map geometry sau profile ổn định mới.
- Xác nhận exact-frontier execution vẫn PASS.
- Xác nhận wall/corridor trên RViz không còn warp tăng dần trong long run.

## Next actions

1. Dừng `nearest_pilot_008`; giữ directory làm diagnostic evidence.
2. Pull code trên Ubuntu.
3. Rebuild `frontier_exploration` để installed Nav2 + SLAM configs nhận profile mới.
4. Chạy `scripts/preflight_nearest.py`; phải PASS cả:
   - installed Nav2 exact-planner tolerance;
   - installed mapping speed `0.45 m/s`;
   - installed SLAM keyframe spacing `0.10 m / 0.10 rad`.
5. Chạy default `hospital_nearest.launch.py` → `nearest_pilot_009`.
6. Sau vài goal, chạy validator `--run-id nearest_pilot_009 --allow-running`.
7. Quan sát RViz trong long run; nếu wall geometry vẫn thẳng và validator PASS thì để pilot kết thúc tự nhiên.
8. Chỉ sau pilot_009 ổn định mới khóa code và chạy official `nearest_001 ... nearest_005` trước; nếu run 001 có bất thường thì dừng batch để điều tra, không lãng phí 4 run tiếp.
9. Khi Nearest official hoàn thành, port full MapEx `visvarprob` dùng cùng frontier generation, below-1m adaptation, exact-frontier execution, mapping profile, Nav2, benchmark clock và logging schema.

## Important decisions

- Hospital implementation không được gọi là MapEx chạy nguyên xi: frontier semantics/ranking được port, execution adapt sang Nav2.
- Base/reference adapter giữ documented original 1 m semantics để đối chiếu; Hospital benchmark runtime không enforce rule này.
- Nearest, MapEx và proposed method phải dùng cùng Hospital adaptation, exact-frontier execution, mapping-stability profile và benchmark-clock definition.
- `ComputePathToPose` path endpoint không phải execution goal; exact `/frontier_selected` center mới là NavigateToPose goal.
- Hospital planner tolerance dùng `0.0 m`.
- Hospital mapping speed dùng `0.45 m/s`; SLAM keyframe spacing `0.10 m / 0.10 rad`.
- Không nới loop-closure thresholds chỉ để làm map nhìn thẳng hơn.
- `t=0` là first policy decision before computation.
- Official run phải clean git và metadata phải hash installed runtime files thực tế.
- Simulator seed intentionally uncontrolled; variability xử lý bằng repeated runs.
- `coverage` trên frozen ROI là exploration metric chính; `known_fraction` là progress/debug proxy.
- Pilot/debug không được dùng như official benchmark result.

## Latest result

`nearest_pilot_005`: diagnostic 1 m deadlock.  
`nearest_pilot_007`: diagnostic tolerance-snapped execution-goal bug.  
`nearest_pilot_008`: exact-frontier execution/logging PASS nhưng long-run map bị warp.  
`nearest_pilot_009`: mapping-stability pilot với `0.45 m/s` + `0.10 m / 0.10 rad`, cần chạy trước official Nearest runs.
