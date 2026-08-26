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
- Thêm final recorder wrapper `scripts/research_recorder_official.py`:
  - đồng bộ `t=0` từ policy pre-compute timestamp;
  - hash cả source research code và installed runtime files thực tế;
  - ghi rõ simulator seed policy là Gazebo default intentionally uncontrolled + repeated-run statistics.
- Validator được siết: snapshot integrity, candidate join, no `PENDING` sau termination, failure reason, metrics time/distance sanity, coverage bounds, provenance, clean git cho official runs.
- `DATA_SCHEMA.md` đã đồng bộ Hospital below-1m semantics và candidate IDs.
- `ROADMAP.md` đã bỏ wording custom WFD, chuyển sang shared MapEx frontier candidate set.
- `EXPERIMENT_PROTOCOL.md` đã chốt benchmark clock, seed policy, exact exhausted state và provenance.

## In progress

- Chạy `nearest_pilot_007` như pilot cuối.
- Xác nhận robot motion với below-1m candidate, pre-compute clock, exact decision logging và strict validator.

## Next actions

1. Pull code trên Ubuntu.
2. Xóa local `nearest_pilot_007` nếu đã tồn tại do thử dở.
3. Chạy `scripts/preflight_nearest.py`; phải PASS toàn bộ invariant.
4. Chạy `hospital_nearest.launch.py` (default `nearest_pilot_007`).
5. Sau vài decision, chạy validator với `--allow-running`.
6. Để pilot kết thúc tự nhiên nếu có thể; chạy validator lần cuối không có `--allow-running`.
7. Chỉ khi pilot_007 PASS mới khóa code và chạy `nearest_001 ... nearest_010`.
8. Khi Nearest official hoàn thành, port full MapEx `visvarprob` dùng cùng frontier generation, below-1m adaptation, Nav2, benchmark clock và logging schema.

## Important decisions

- Hospital implementation không được gọi là MapEx chạy nguyên xi: frontier semantics/ranking được port, execution adapt sang Nav2.
- Base/reference adapter giữ documented original 1 m semantics để đối chiếu; Hospital benchmark runtime không enforce rule này.
- Nearest, MapEx và proposed method phải chia sẻ cùng Hospital adaptation và benchmark-clock definition.
- `t=0` là first policy decision before computation, không phải first selected path.
- Exact no-candidate termination state phải được lưu như policy decision.
- Official run phải clean git và metadata phải hash installed runtime files thực tế.
- Simulator seed intentionally uncontrolled; variability xử lý bằng repeated runs.
- `coverage` trên frozen ROI là exploration metric chính; `known_fraction` là progress/debug proxy.
- Pilot/debug không được dùng như official benchmark result.

## Latest result

`nearest_pilot_005` là diagnostic pilot cho incompatibility của MapEx 1 m rule. `nearest_pilot_007` là pilot cuối của schema/protocol hiện tại; chưa có official Nearest run.
