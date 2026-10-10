# Experiment Protocol

## New Room R003 paper500 supplement — USER-approved 2026-09-23

The current New Room `new_room_mapex_paper500_v1` protocol is defined in
[docs/R003_PAPER500_PROTOCOL_V1.md](docs/R003_PAPER500_PROTOCOL_V1.md). It uses
500 adapted 0.30 m odometry-progress steps (150.0 m), common k=10 support and
the unchanged approved GT/valid-space/evaluation semantics. The earlier
paper1000 document is historical. Official runtime remains gated on independent
review of the exact implementation SHA.


Mọi phương pháp so sánh phải dùng cùng protocol, trừ khi thay đổi đó chính là biến thí nghiệm và được ghi rõ.

## Protocol identity

- Protocol version: `hospital_v2`
- Fixed canvas ID: `hospital_canvas_v1`
- Evaluation ROI ID: `hospital_connected_free_v1`

`hospital_v2` dùng runtime SLAM/policy grid `0.10 m/cell` để Nearest Frontier, MapEx frontier và MapEx prediction làm việc trên cùng độ phân giải. Fixed logging canvas vẫn ở `0.05 m/cell`; runtime grid và evaluation canvas là hai khái niệm độc lập.

## Environment

- World: `Hospital flat` (`hospital_aws_flat.sdf`)
- Simulator: Gazebo / TurtleBot4 simulation stack
- Robot: TurtleBot4
- Spawn: `x=0.0 m`, `y=12.0 m`, `yaw=-1.57 rad`

### Simulator seed policy

Gazebo launch hiện không expose một seed cố định. Vì vậy:

- simulator seed: **intentionally uncontrolled**;
- không giả vờ các run có cùng random seed;
- variability được xử lý bằng repeated runs (`5` tối thiểu, mục tiêu `10` mỗi method);
- metadata ghi `sim_seed_policy=intentionally_uncontrolled_gazebo_default_multiple_run_statistics`.

Nếu sau này thêm fixed seed thì đó là thay đổi protocol và baseline liên quan phải được xem xét chạy lại.

## Mapping / SLAM

- SLAM: `slam_toolbox`
- Runtime resolution: `0.10 m/cell` đối với `hospital_v2`
- Map frame: `map`
- Mapping speed cap mục tiêu: `0.45 m/s`
- `minimum_travel_distance = 0.10 m`
- `minimum_travel_heading = 0.10 rad`
- `minimum_time_interval = 0.15 s`
- LiDAR max range dùng bởi SLAM: `20 m`

Khi A/B một biến SLAM, các tham số còn lại phải được giữ giống nhau để tránh confound.

### Fixed logging canvas

- ID: `hospital_canvas_v1`
- Resolution: `0.05 m/cell`
- Width: `1504`
- Height: `2123`
- Origin: `(-25.6, -60.1) m`

Với map axis-aligned `0.10 m`, mỗi runtime cell được reproject bằng nearest-neighbour area preservation thành khối `2 x 2` cell trên fixed canvas `0.05 m`. Raw OccupancyGrid vẫn được lưu nguyên resolution để replay/audit.

Không crop theo bounding box động và không normalize final known fraction riêng từng run thành 100%.

### Canonical evaluation ROI

- ROI: `hospital_connected_free_v1`
- Denominator: `215435` cells
- Mask SHA-256: `05d45b7aba66cb6dbb71e0406005f4a3e21875901af3ae72164b17f1d3add8d1`
- Bounds SLAM-start: `x=[-0.572445,24.588833]`, `y=[-35.091079,21.044604]`
- Structural source: Hospital wall collision mesh + flat elevator blockers
- Connectivity: 8-connected free component containing start `(0,0)`

```text
coverage(t) = known cells inside hospital_connected_free_v1 / 215435
```

ROI/canvas không được đổi giữa Nearest, MapEx và proposed method trong cùng protocol.

## Navigation

Nav2 là execution layer chung cho Nearest và MapEx. Các method phải dùng cùng Nav2 configuration khi so sánh trực tiếp.

### Position-only frontier goal semantics

Frontier là target vị trí `(x,y)`; policy không tối ưu terminal yaw.

Yêu cầu:

- exact frontier `x/y` là execution position;
- không thay frontier bằng nearest-safe-cell hoặc planner endpoint;
- quaternion trong `NavigateToPose` chỉ là neutral orientation seed;
- effective Nav2 goal checker phải không biến terminal yaw thành biến policy khác giữa các method.

Quãng đường thực tế lấy từ odometry/trajectory, không suy từ một global path duy nhất vì Nav2 có thể replan.

## Frontier policy shared by Nearest and MapEx

Frontier generation bám theo public MapEx implementation (`castacks/MapEx`, reference commit `53636bd1c79153acc3c74a532837d78c926bae5e`):

- runtime policy grid: `0.10 m/cell` trong `hospital_v2`;
- free: occupancy `0`;
- unknown: occupancy `<0`;
- frontier cell: free cell kề unknown trong 8-neighbourhood;
- frontier regions: 8-connected;
- giữ region khi `size > 10`;
- representative: frontier cell gần arithmetic mean `(row,col)` của region nhất;
- không dùng custom WFD reachable-BFS, segment split, standoff hoặc nearest-safe-cell substitution.

### Shared 1 m frontier-distance preference + near-frontier fallback

Public MapEx có `cur_pose_dist_threshold_m = 1`. Trong ROS/TurtleBot4 adaptation, hard rejection toàn bộ candidate `<1 m` có thể làm exploration đứng ngay khi frontier hiện tại chỉ có các representative gần robot. Vì vậy Nearest và MapEx dùng **cùng một luật eligibility**:

```text
raw frontier representatives
→ nếu tồn tại ít nhất 1 representative có distance >= 1.0 m:
     chỉ nhóm >= 1.0 m được đưa vào ordinary policy selection
→ nếu tất cả representative hiện tại đều < 1.0 m:
     bật near-frontier fallback và cho phép xét toàn bộ nhóm gần
→ sau đó mới áp dụng execution/planner suppression
→ policy chọn candidate tốt nhất trong tập còn selectable
```

Các điểm cần giữ đúng:

- `1.0 m` là **preference threshold**, không phải điều kiện completion;
- `all frontiers <1m` **không** đồng nghĩa exploration complete;
- fallback chỉ bật khi toàn bộ raw representative hiện tại đều dưới ngưỡng, không phải vì các candidate xa đang bị suppression;
- luật này phải giống nhau giữa Nearest, MapEx và proposed method nếu muốn so sánh policy công bằng;
- recorder phải log `below_1m`, `distance_eligible` và `near_frontier_fallback` để đo ảnh hưởng thực tế của adaptation.

Đây là ROS2 execution adaptation; không mô tả là byte-for-byte reproduction của public simulator.

### Nearest scoring

```text
cost(frontier_i) = EuclideanDistance(current_pose, frontier_i)
selected = argmin(cost) trong tập selectable
```

### MapEx scoring

MapEx giữ nguyên core scoring:

```text
P1,P2,P3 = LaMa ensemble predictions
mean_map = mean(P1,P2,P3)
variance_map = variance(P1,P2,P3)
visibility(f) = probabilistic predicted visibility
IG(f) = sum variance over predicted-visible AND currently-unknown cells
score(f) = IG(f) / EuclideanDistance(robot, f)
selected = argmax(score) trong tập selectable
```

Luật 1 m/fallback chỉ thay **candidate eligibility**; không thay công thức `IG/d`.

## ROS/TurtleBot4 execution adapter

Public MapEx simulator dùng A* (`pyastar2d`). ROS2 implementation hiện dùng Nav2:

```text
frontier generation
→ method-specific ranking/scoring
→ shared 1 m preference / all-near fallback
→ shared suppression rules
→ NavigateToPose(exact frontier x/y)
→ /plan được lưu làm navigation diagnostic và path-guided recovery evidence
```

Không bắt buộc `ComputePathToPose` trước **mọi** ordinary frontier. `ComputePathToPose` được dùng cho terminal planner revalidation khi không còn normally selectable candidate sau planner-blocking suppression.

**Execution-goal invariant:** goal thực thi phải giữ exact frontier `x/y`; planner/path endpoint không được dùng để thay frontier center.

Recorder giữ riêng:

```text
frontier x/y       = exact selected representative
goal target x/y    = actual NavigateToPose position
plan endpoint x/y  = last pose của /plan, diagnostic only
plan endpoint error = distance(endpoint, exact frontier), diagnostic only
```

`plans.csv.path_length_m` là chiều dài path được Nav2 publish tại thời điểm đó, **không** phải total executed trajectory. Quãng đường robot thực lấy từ `trajectory.csv` / `metrics.csv.distance_m`.

## Planner revalidation and navigation failures

Một planner/navigation failure không được coi là bằng chứng vĩnh viễn trong hệ ROS online.

- `NavigateToPose` main-goal error `206 = GOAL_OCCUPIED` và `208 = NO_VALID_PATH` là planner-blocking failures. Frontier và candidate rất gần nó bị suppress khỏi ordinary selection để exploration chuyển sang frontier khác.
- Suppression `206/208` không phải blacklist vĩnh viễn. Khi không còn normally selectable candidate, eligible representative set được `ComputePathToPose` revalidate lại.
- Candidate planner-reachable trở lại phải được bỏ planner suppression và exploration tiếp tục.
- Stable non-empty candidate set cần repeated exhausted revalidation sweeps trước completion.
- Nếu map/frontier set thay đổi, completion verification reset.
- `105 FAILED_TO_MAKE_PROGRESS` là execution/controller failure: cho path-guided recovery có giới hạn rồi temporary cooldown; nó không tự trở thành permanent planner-unreachable evidence.
- Failed temporary subgoal không tự biến main frontier thành permanent blacklist.

## Benchmark clock

`time_s = 0` bắt đầu tại **first policy decision before computation**: sau khi map/TF/Nav2 ready nhưng ngay trước candidate computation đầu tiên.

- Nearest vì vậy tính cả frontier-generation/ranking cost đầu tiên.
- MapEx tính cả LaMa prediction + visibility + IG/scoring đầu tiên.
- Recorder lưu timestamp này trong `metadata.json` dưới `exploration_start_sim_s`.

Không yêu cầu một topic riêng để định nghĩa clock; source of truth là timestamp được recorder freeze khi decision đầu tiên bắt đầu.

## Required replay logging before official runs

### Exact policy decision state

Mỗi policy decision phải lưu đủ để replay lựa chọn:

- raw OccupancyGrid + width/height/resolution/origin/frame/timestamp;
- fixed-canvas reprojection;
- map-frame robot pose dùng cho ranking;
- candidate computation time;
- full frontier representative set;
- raw rank / policy rank;
- `below_1m`, `distance_eligible`, `near_frontier_fallback`;
- execution/planner suppression state;
- selected candidate và decision outcome.

Khóa join chung:

```text
policy_decision_id + candidate_id
```

### Nearest replay invariant

Từ `observed_map_raw.npz` + robot pose phải có thể tái tạo:

```text
frontier cells
→ regions >10
→ representatives
→ distance
→ 1m preference/fallback
→ suppression
→ nearest selectable candidate
```

### MapEx replay extension

MapEx lưu thêm:

- G1/G2/G3 predictions (khi prediction retention bật);
- ensemble mean / variance;
- Information Gain;
- score `IG/d`;
- visible-unknown cell count;
- prediction/scoring timing.

### Periodic/final map retention

Recorder lưu raw + fixed-canvas snapshot khoảng mỗi `10 s` và một final snapshot; `snapshots.csv` là index chung cho hai method.

### Effective runtime provenance

Mỗi run phải lưu/hash tối thiểu:

- policy + recorder source;
- active launch/profile;
- relevant SLAM/frontend source;
- base installed Nav2 params;
- project Nav2 override;
- `runtime_nav2_merged.yaml`;
- world/ground-truth source khi áp dụng;
- simulator seed policy;
- benchmark clock source;
- termination reason.

Official runs nên bắt đầu từ clean git worktree.

## Exploration termination

Completion của shared execution layer phải bảo thủ:

- không complete khi main/subgoal còn active;
- không complete chỉ vì remaining frontiers `<1 m`;
- zero frontier regions cần stable repeated evidence;
- nếu regions còn nhưng normally selectable set bị planner-blocking suppression, dùng repeated planner revalidation;
- stable `5` exhausted sweeps;
- sweep spacing tối thiểu `2 s`;
- idle tối thiểu `10 s`;
- startup grace `20 s`;
- execution cooldown do `105` không phải terminal evidence.

## Metrics required per run

- `known_fraction` vs time/distance;
- `coverage` vs time/distance;
- total odometry distance/time;
- frontier goals attempted/succeeded/failed;
- success rate;
- termination reason;
- policy computation timing;
- fallback count/fraction;
- raw maps đủ để tính occupied IoU/TU offline.

Paper-level metrics cần nhớ: Coverage, occupied-class IoU và Topological Understanding (TU).

Validator nên tách **data/protocol integrity** khỏi **experiment health** và phát hiện pathology kiểu "goal SUCCEEDED nhưng robot gần như không di chuyển".

## Repetition

- Nearest target: `10` runs
- MapEx target: `10` runs
- Minimum preliminary: `5` per method
- pilot/debug không tính vào official benchmark.

Historical Hospital pilots trước shared 1 m fallback có thể giữ làm diagnostic nhưng không trộn với official runs của protocol mới mà không ghi rõ version.

## Fair-comparison rule

Không đổi giữa Nearest và MapEx mà không ghi rõ lý do: spawn, sensor, SLAM runtime resolution/profile, Nav2, timeout, stopping condition, fixed canvas, ROI, frontier-generation semantics, **1 m preference + all-near fallback**, position-only goal semantics, suppression/revalidation, benchmark-clock definition hoặc resource budget.

Khác biệt policy chủ yếu cần giữ đúng là:

```text
Nearest: chọn min Euclidean distance
MapEx:   chọn max IG / Euclidean distance
```

Execution adapter và recorder phải được chia sẻ tối đa để khác biệt kết quả không đến từ hạ tầng thí nghiệm.

## MX071 R2 - separate offline 2D pilot (2026-10-09)

This section describes the accepted MX071 R2 pilot and does not alter the ROS/Hospital protocol above. Authority is the [exact R2 design](https://github.com/vah103/chat-gpt/blob/e2987a65e23df7cd04fc60c19505492d771ce801/company/projects/mapex/MAPEX_MULTI_DIRECTION_2D_PILOT_DESIGN_R2_20261009.md); implementation/deployment details and named adaptations are in [pilots/mx071_r2/README.md](pilots/mx071_r2/README.md).

Stage 1 is fixed to four KTH map IDs, two deterministic starts per map, eight PIPE_ALIGNED sources, and targets 20/40/60/80 m. Only targets 20/60 m permit five first-goal requests each: at most 16 branch points and 80 logical requests before dedup/NA. Coincident target slots use one complete snapshot; exact execution aliases do not inflate N. Stage 2 is CLOSED.

Unknown-blocked ALIGNED_SHARED planning is distinct from CODE_REFERENCE's audited native helper behavior. All policies share the source sensor, four-connected A*, full immutable paths, deterministic candidate ordering, the explicitly named one-cell/scan-every-move controller, and common PIPE_ALIGNED continuation. Native scoring retains `path[2::3]` without endpoint addition and the sampled-count denominator. Matched controls add/deduplicate the endpoint and use full path metres. Goal tolerance remains Euclidean strictly less than 1 m. The distance budget is 100 m / 1000 successful moves; no GT-IoU termination or near-goal fallback is added.

P0/P1/P4 share observed state, candidate paths, frozen uncertainty, and source-render schedule. P0 online decisions never receive GT. GT is permitted only for environment/start/evaluation and explicit offline oracle diagnostics. MAPEX_ALIGNED retains the native probabilistic mean-map endpoint renderer. H4 is oracle headroom; declare EXECUTION_SENSOR_CONTRACT_GAP when scored-route and actual first-goal sensor contracts differ. H7 is restricted to distinct tested actions with the sealed remaining-budget support; optional directions remain unevaluated/deferred.

Coverage uses a fixed initially reachable GT-valid free component. Q integrates incremental coverage against a common remaining travel budget, with early-terminal carry-forward and infrastructure outcomes missing. Aggregate checkpoint means within start, then start means within map, then equal map means. Positive direction screening requires all four maps, delta Q >= 0.01, at least three positive map means, at least six non-equivalent map/start units across at least three maps, and no increase in behavioral failures. Preserve NA and structural zeros.

Models use exact checkpoint identities, channel 0, NumPy mean, unbiased K=3 Torch variance, and the upstream transform with actual LaMa padding cropped back; no resize. DELL CPU members run sequentially. Precollection resource V3 permits 5120 MiB per-worker RSS with unchanged 256 MiB minimum available memory, 512 MiB disk headroom, and 1200 s per-member timeout. Source hashes, models, code, maps, starts, adaptations and gates are sealed before collection.

The first map block's report-writing ASCII error was recovered by process-local UTF-8 locale settings. The initial resume kept the Python implementation unchanged; a subsequent recorded RESUME_FIX1 adds only a completed-selector-group guard before repeated offline diagnostics. Both original and amended execution manifests are retained. Four focused checks PASS and all numerical contracts remain unchanged. Completed-data hashes and failure phase are preserved in the recovery ledger. The map-ID cohort remains TRAIN_OVERLAP_UNVERIFIED; partial or engineering data is not an accepted scientific result.


## 2026-10-10 — Proposed MapEx follow-up / separate PIPE audit

USER selected all seven open questions (#1–#6, #8); #7 remains dropped. Current bounded proposal: `pilots/mx071_followup/PLAN_R1.md`, following IR1 R0 REVISE; historical PLAN_R0.md remains unchanged. R1's direct #2 channel is G1/G2/G3 mean/variance and downstream visvarprob state, with G auxiliary and fixed weights. Acquisition proposes separate NATIVE_SCORE_EPOCH and active-lock OBSERVATION_CONTROL_STATE strata with outcome-independent nomination and labelled shadow rescoring. The 100m adaptation uses LAST_OBSERVED_STATE_AT_OR_BEFORE_BUDGET for both arms' primary Q and retains crossing endpoint scans separately. PIPE_AUDIT_R1.md separates effective native control-step limits (resolved time_step, not mission_time alone at the exact source pin) from our 100m distance-prefix contract. These are unimplemented proposal semantics, not a complete method/runtime ACCEPT. Direct MapEx utility still requires source states, full-state replay and common MapEx continuation, sealed controls/reuse/failure/zeros, a new branch ceiling and model/resource/storage/time gates. Existing eight-step stand-in evidence is unchanged; no R1 model or fixture rerun occurred. The old 80-request ceiling, accepted R2 method and MX072/COM1 scope do not transfer.


## 2026-10-10 — MX071 follow-up R2 executable-method candidate (review pending)

Current proposed seven-question contract: [PLAN_R2](pilots/mx071_followup/PLAN_R2.md), [question controls](pilots/mx071_followup/QUESTION_CONTRACTS_R2.md), [resource/preflight](pilots/mx071_followup/RESOURCE_AND_PREFLIGHT_R2.md), [machine constants](pilots/mx071_followup/METHOD_CONTRACT_R2.json). USER directly authorized drafting and IR handback, not runtime. R1 bounded proposal ACCEPT is historical and preserved, not a complete R2 execution grant.

Eight native visvarprob KTH/DELL source acquisitions; original source weighted four-connected planner with distance_transform=True, effective unknownFalse, controller index3, mission_time1000, native pad/ray quirks. Both R1 nominee strata and four targets/windows retained. Stage B fixed S1=20m/S2=60m source/observation nominees, one per stratum/start; R>=20 support; no outcome-driven replacement.

All seven have single-factor/matched controls and independent native MapEx continuation: aggregation, immediate observation-conditioned inference, topology with dose controls, matched unknown-only visibility oracle, one-time locked-goal refresh and compute sham, fixed-cluster/expanded-pool views and limited short/future action set. #7 dropped.192 total replay requests=176 scientific+16 full KEEP seals before dedup; eight sources additional. No inherited80 ceiling.

Primary100m LAST_OBSERVED_STATE_AT_OR_BEFORE_BUDGET, endpoint-chord distance, original remaining source-step budget, exact endpoint-held Q and separate actual crossing sensitivity. Actual planned arc/measured policy/research compute/declared virtual-time clock remain separate. Full-state/P19 suffix replay, lossless decoder and all failure/zero/NA/tie/technical-censor rules are binding in the candidate. Positive mechanism/action is not a causal or online-policy win.

FULL_7 and STAGED_CORE are explicitly costed alternatives; no scope narrowing or execution is selected. Current DELL disk fails admission and real-model/resource/source/codec readiness is not certified. Return full R2 to IR1 before implementation/preflight authority and separately authorized acquisition/branch execution. PIPE_AUDIT_R1 stays independent with0 scientific requests; MX072/New Room/COM1 untouched. This section is a proposal record, not canonical adoption or a scientific result.

## 2026-10-10 — R2 bounded corrections pending IR1 focused closure

IR1 full R2 review0038e276 at PR42 head36027f2 returned REVISE only MX071-R2-1 and MX071-R2-2; the large architecture passed. See pilots/mx071_followup/REVIEW_RESPONSE_R2.md and corrected R2 bundle.

#3 preserves size/error/direction/distance/L1 dose and adds pre-treatment exact native candidate-wise free-prefix/first-blocker/polygon-hit counts, full visible-unknown/origin bits, frozen variance bins and candidate-wise weighted opportunity. One deterministic or seeded construction; no redraw/bin widening/alternate patch. Missing exposure match -> CONTROL_SUPPORT_NA; native→TOPO is oracle headroom and topology specificity INCONCLUSIVE. No treated score/visibility or branch Q selects controls.

#8 keeps the four actions and G10/Q10/Q_R but reports NATIVE_ACTION_HAS_TESTED_LONG_HORIZON_REGRET / TESTED_HORIZON_DISAGREEMENT_INVOLVING_NATIVE_ACTION. G10 alignment is descriptive; source predicted variance/distance does not optimize realized G10. Causal source short-horizon attribution would require a separately reviewed scorer intervention, absent here.

Preserved:176 scientific+16 KEEP=192 replay requests,8 shared sources, S/O/full-state/own-scan/common continuation, native-step/100m/Q/failure/NA/tie rules, all P01–P19 and FULL_7/STAGED_CORE original numerical limits. Exposure records <=64MiB/S and <=2GiB across32 slots inside existing4GiB diagnostic bucket; <=8192 extra baseline renders within original600s/slot/worker RSS, no additional models, branch or quota. P08/P13/P15 need future technical verification; none was run.

Status: focused closure/regression pending with same IR1 successor04; no self-issued CLOSED/ACCEPT. Static document/arithmetic checks only; genuine model/replay/RAM/time/storage feasibility remains uncertified and current DELL disk gate fails. R1/R0/PIPE/Python/readiness/completed evidence and original R2 verification preserved. No model/source/simulation/probe/batch/READY/START/merge, DELL cleanup/deletion, COM1/MX072 change or staff/task reallocation. Previous proposal/review checkpoints remain historical.

