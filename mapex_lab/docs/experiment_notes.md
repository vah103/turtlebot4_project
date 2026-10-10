# Experiment Notes

## 2026-09-22 — TU goal-domain differs from original MapEx

- Original MapEx TU samples 100 goal cells directly from `valid_space == 1` for each map/start condition, stores those coordinates, and reuses the same goal set across methods/timesteps.
- The public implementation does not additionally require the sampled goal cell to be GT-free at the sampling step. Because `valid_space` and occupancy are downsampled with different rules, some valid-space cells can overlap GT-occupied cells; such goals are effectively predisposed to TU failure.
- This project intentionally uses a different goal domain for New Room / Hospital TU: 100 deterministic goals are sampled from the connected structural GT-free ROI (fixed seed), so every goal is a physically traversable free-space location in the scored environment.
- Consequence: this project's TU is **MapEx-style but not byte-for-byte equivalent** to the original paper implementation. Absolute TU values should not be compared directly with the paper without noting this goal-domain difference.
- Rationale: restricting goals to connected GT-free space is more meaningful for navigation because the metric then asks whether predicted topology supports valid robot destinations, rather than allowing goals that may lie inside walls/obstacles due to raster/downsampling semantics.
- Fairness within this project is preserved because the same frozen goal set is reused across compared methods on the same environment/evaluation profile.
- No metric change is made by this note; it documents the current intentional adaptation and its interpretation.

## 2026-09-22 — MapEx fixed-horizon vs project completion mismatch

- Observation: the original MapEx paper evaluates exploration under a fixed budget of **1000 timesteps**. The reported Coverage curves therefore measure how much of the environment is observed within that fixed horizon, not the final Coverage after exhaustive exploration.
- Current project behavior: New Room / Hospital runs are generally allowed to continue until the shared exploration-completion condition (or an explicit early-stop rule) is reached. Final Coverage values from these runs therefore represent near-completion/completion behavior rather than Coverage@1000.
- Consequence: directly comparing paper MapEx final plotted Coverage (about 82–83% at the 1000-timestep horizon) with this project's near-completion values (often close to 100%) is misleading because the stopping horizon is different.
- Important interpretation: the paper's lower final Coverage does **not** by itself mean MapEx cannot explore the remaining area; it mainly reflects the fixed evaluation budget. Conversely, this project's near-100% final Coverage does **not** demonstrate a better exploration policy unless compared at a matched budget/horizon.
- Reporting rule: distinguish at least **Coverage@fixed-budget** from **final/completion Coverage**. Do not present them as directly comparable metrics without matching the exploration horizon.
- No protocol change yet: this note records the mismatch only. Any future reproduction study should either evaluate this project at the same 1000-timestep horizon or explicitly justify another matched time/distance budget.

## 2026-09-22 — Coverage GT/ROI semantics differ from original MapEx

- Observation: the current project Coverage denominator is based on connected-free ROI masks such as `hospital_connected_free_v1` / `new_room_connected_free_v2`, i.e. reachable/free-space support only.
- Original MapEx behavior: the public MapEx evaluator computes Coverage inside `valid_space.npy`, and counts a cell as covered when the observed map is no longer unknown. The MapEx `valid_space` mask is not equivalent to free-space-only occupancy GT; it can include both free and occupied cells inside the valid evaluation region.
- Practical meaning: our current Coverage asks approximately "how much of the reachable free-space ROI has become known", while original MapEx asks approximately "how much of the valid environment region has become known".
- Consequence: current absolute Coverage values are **not metric-semantics-equivalent to original MapEx Coverage** and should not be described as an exact reproduction of the paper metric without qualification.
- Scope: this note does **not** invalidate within-project Nearest-vs-MapEx comparisons when both methods use the same frozen ROI; it affects cross-paper comparability and interpretation of the absolute Coverage number.
- No protocol change yet: do not replace the existing ROI automatically. If exact MapEx-style metric reproduction is required, define and validate a separate MapEx-style valid-space mask for New Room/Hospital, version the metric/ROI, and decide whether historical values need backfill.
- Evidence checked: original `castacks/MapEx` coverage code uses `known & valid_space` divided by `valid_space`; inspection of an original KTH test-map pair confirms `valid_space` is not identical to free-only occupancy support.
- Next action: USER/WORK decides whether to keep current connected-free Coverage as the project metric, add a separate MapEx-style Coverage metric, or migrate under a new protocol/metric version.

## 2026-09-22 — Canonicalize reviewed R002 GT-semantics diagnostic

- What changed: synced the independently ACCEPTED R002 evaluator and toy tests into technical `main` byte-identically from accepted implementation `feb94eaa9c1ba5aa4f9993792dae454bc4edcd60`; updated D1 STATUS/README/research log with the reviewed one-run `mpx_001` evidence.
- Validation: pre-score compile PASS; 12/12 unit tests PASS; `mpx_001` exit code 0 with 35/35 decisions, Gate-P support parity True, topology-invalid 0; independent H010 code review verdict ACCEPT.
- Scientific scope: R002 remains a post-hoc diagnostic. Reference A / negative Gate P2 are preserved. One-run evidence is mixed/negative late and does not rescue direct D1.
- No methodology changes: threshold 0.5, B tolerances, one-to-one matching order, C0 0.05 m / 4-connectivity, topology domain and seed rule remain frozen.
- No cohort expansion: `mpx_002...mpx_010` were not run; further expansion requires explicit USER/WORK authorization.
- Raw first-score artifacts remain local/untracked; canonical repo stores accepted evaluator/tests plus reviewed summary/reproducibility notes.
- Next action: USER verifies fetched `origin/main` and local status without overwriting unrelated work; then close R002-SYNC if verification passes.

## 2026-09-21 — Fix Gate-P2 fractional origin reprojection

- What changed: removed the exact integer-lattice origin assertion in `analysis/d1/d1_gate_p.py`; P2 now uses the same nearest-cell rounding reprojection as the canonical MapEx evaluator.
- Why: the first `mpx_001 --reference both` smoke test failed because SLAM raw-map origins can have fractional 0.05 m canvas offsets.
- Observation: this was an analyzer alignment bug, not a run-data provenance failure.
- Audit: P2 records x/y origin-rounding residuals; for the original failing decision they are approximately +0.01523 m and -0.02458 m.
- Affected runs: offline Gate-P2 analysis only; no raw run changed.
- Does baseline need rerun?: no.
- Next action: rerun the one-run smoke test, then full 10-run P1+P2 if clean.

## 2026-09-21 — Match Gate-P predictions to exact raw maps

- What changed: Gate-P prediction loading now hard-checks shape, resolution, origin, source map timestamp and G1/G2/G3/mean member identity against the exact decision raw map.
- Why: shape-only validation could miss a stale/wrong-decision prediction with identical map extent.
- Environment policy: metadata/prediction environment mismatch is WARN-only; GT ID/canvas/resolution and prediction/raw spatial/source identity remain hard-fail.
- Affected runs: no raw runs changed; offline analyzer only.
- Does baseline need rerun?: no.
- Next action: smoke test `mpx_001 --reference both --no-figures`, then run the full cohort if clean.

## 2026-09-21 — Harden D1 Gate P2 provenance before cohort run

- What changed: added structural-GT provenance validation and expanded P1-vs-P2 class-specific/support comparison in `analysis/d1/d1_gate_p.py`.
- Provenance policy: missing legacy metadata warns; present contradiction in environment/GT ID/GT file/canvas/resolution/runtime resolution fails.
- Comparison policy: overall + last-10 now include free/occupied precision/recall/IoU in addition to accuracy/macro-IoU/MAE.
- Support policy: compare P1/P2 primarily by evaluated area and unknown-area fraction because their cell resolutions differ.
- Threshold unchanged: Gate P uses `<0.5 free`, `>=0.5 occupied`; legacy evaluator boundary semantics are documented separately.
- Affected runs: no raw runs changed; offline analyzer only.
- Does baseline need rerun?: no.
- Next action: smoke test then execute `--reference both` on `mpx_001...mpx_010`.

## 2026-09-21 — D1 Gate P2 structural-GT reference added

- What changed: extended `analysis/d1/d1_gate_p.py` with `later_observed|structural_gt|both` modes; default is `both`.
- Why: Gate P1 is right-censored because it only evaluates `Unknown_t ∩ EventuallyObserved`; P2 must evaluate the broader valid structural-GT unknown region.
- Structural target domain: `Unknown_t ∩ structural_gt.evaluation_mask`.
- Alignment: runtime 0.10 m cells are nearest-neighbour expanded to the canonical 0.05 m structural-GT canvas using the existing evaluation-grid convention.
- Important methodological decision: connected-free ROI is not the P2 classification mask because it contains free-space support and would discard occupied targets.
- Affected runs: no raw runs changed; offline analysis only.
- Does baseline need rerun?: no.
- Observation: code implemented; P2 numerical execution is still pending on local NPZ artifacts.
- Next action: run Gate P with `--reference both` on `mpx_001...mpx_010` and compare P1 vs P2 before Gate U.

## 2026-09-09 — All-training evaluation moved offline

- Restored three-model online config/worker/bridge/recorder; removed alltrain startup, saving and metadata requirements.
- Added `predict_alltrain_offline.py`: loads only the whole-training predictor, consumes saved raw decision maps, publishes complete hash-linked manifest, and preserves original run records.
- Evaluator uses completed offline predictions for primary IoU/TU with secondary ensemble-mean metrics; incomplete/stale manifests are rejected.
- Four ROS-free tests pass. Real inference remains pending absent model weights/environment. No robot motion or formal research result was produced.
- The earlier four-model online implementation is superseded; alltrain no longer adds decision time or GPU memory during exploration.

Ghi ngắn gọn các quyết định hoặc sự cố có thể ảnh hưởng kết quả.

## Template

### YYYY-MM-DD — <run or change>

- What changed:
- Why:
- Affected runs:
- Does baseline need rerun?:
- Observation:
- Next action:

## 2026-09-07 — Adaptive Temporal Anchor V1 after hard-chain diagnostic

- What changed:
  - Added `launch/toolbox_adaptive.launch.py` as a clean A/B variant of the previously stable `toolbox.launch.py`; scan cadence, Karto scan matching, near-chain graph construction, conservative loop-closure settings, whole-graph Ceres optimization and Nav2 remain unchanged.
  - Integrated Adaptive Anchor V1 into the vendored `slam_toolbox/solvers/ceres_solver.cpp`, default disabled. `toolbox.launch.py` explicitly sets `adaptive_anchor_enabled=false`; `toolbox_adaptive.launch.py` enables it.
  - Only strict sequential graph edges (`node_gap<=1`) receive `w(n)=1+2*exp(-n/50)`, i.e. `3x -> 1x`. Long-gap edges are never strengthened.
  - V1 intentionally does not apply a second covariance confidence multiplier because Karto covariance already forms the Ceres information matrix.
  - `LinkInfo` does not expose edge origin/type to `ScanSolver`, so V1 uses `node_gap>=30` only as a conservative loop-evidence candidate fallback. Evidence is captured at `AddConstraint()` before Karto loop correction can optimize away the initial residual.
  - Strong release evidence requires at least 3 recent independent long-gap observations with consistent correction and squared normalized/Mahalanobis residual `>=9.0`. Multiple edges attached to essentially the same new node cannot count as independent votes.
  - Strong evidence releases only the related trajectory interval from factor `1.0 -> 0.5`, then Ceres solves the whole graph. Stage 2 recomputes current residuals for the same evidence edges; persistent strong conflict releases that interval to factor `0.0`, returning its sequential edges to ordinary Toolbox `1x`, followed by another whole-graph solve. Release is one-way for the mapping session.
  - Added `src/slam/adaptive_anchor_v1.md` with the exact algorithm, limitations and runtime validation checklist.
- Why: the strict hard-chain test was not satisfactory because it removed too much of Toolbox's ability to distribute correction. The new diagnostic keeps Toolbox's normal mechanisms and adds only a reversible soft temporal preference, with a later-evidence escape path so an incorrect early trajectory is not protected indefinitely.
- Affected runs: Adaptive Anchor V1 diagnostic runs only. Existing Toolbox/Nearest/MapEx data are not reclassified by this change.
- Does baseline need rerun?: no official `hospital_v2` protocol change has been made. For an Adaptive-vs-Toolbox diagnostic, both variants should use the same newly built solver binary, with only `adaptive_anchor_enabled` differing.
- Observation: implementation is committed but compile/runtime validation on the TurtleBot4 Jazzy machine is still pending. The explicit edge-type limitation remains: long-gap evidence is a fallback heuristic because Karto's `LinkInfo` API does not identify loop origin.
- Next action: pull, rebuild `slam_toolbox`, verify `toolbox.launch.py` logs `enabled=false`, verify `toolbox_adaptive.launch.py` logs `enabled=true`, then run ordinary `scripts/nf_basic.py` and inspect stage-1/stage-2 release logs before judging map quality.

## 2026-08-29 — Treat GOAL_OCCUPIED (206) like NO_VALID_PATH (208)

- What changed:
  - `scripts/nf_basic.py` now classifies main-goal `206 = GOAL_OCCUPIED` and `208 = NO_VALID_PATH` as planner-blocking failures.
  - Either code clears the current main frontier, suppresses candidates within `0.10 m` during ordinary selection, and lets exploration choose another frontier instead of retrying the same `(x,y)` forever.
  - Suppression remains reversible: when ordinary candidates are exhausted, terminal `ComputePathToPose` revalidation checks the full eligible set again and can restore a reachable frontier.
  - Controller/execution failures such as `105 FAILED_TO_MAKE_PROGRESS` still use the existing path-guided recovery path when a usable main `/plan` exists.
- Why: `nearest_004` reached frontier `(11.38,7.82)`, then `NavigateToPose` repeatedly returned `206` with no usable `/plan`. The previous generic failure branch kept the same main frontier pending, so the 1 Hz exploration loop resent it indefinitely until manual Ctrl+C.
- Affected runs: `nearest_004` attempt stopped at coverage `0.854499037`, distance `159.13 m` is invalid/incomplete and should be replaced. `nearest_001`–`nearest_003` predate this bugfix; `nearest_003` recorded one `206` but still completed.
- Does baseline need rerun?: `nearest_004` yes, from scratch. For a strictly identical-code formal batch, earlier runs should be treated as pre-fix provenance; current local-window runs are still debug-profile data relative to formal `hospital_v2`.
- Observation: code fix committed as `6c81982`; runtime validation is pending.
- Next action: rerun `nearest_004`; if `206` occurs, verify the immediate sequence is `Main frontier abandoned after GOAL_OCCUPIED (206)` followed by selection of another frontier, with no repeated same-goal loop.

## 2026-08-28 — Event-driven global replanning for `nf_basic.py`

- What changed:
  - Replaced the debug BT's periodic `RateController hz=1.0` global replanning with a plan-once-then-follow sequence.
  - `ComputePathToPose` now runs once when a NavigateToPose attempt starts; map updates alone no longer regenerate the global path while `FollowPath` is progressing.
  - If `ComputePathToPose` or `FollowPath` fails, a `RecoveryNode` waits `0.25 s` and retries the whole plan/follow sequence once, which triggers one fresh global plan using the latest map.
  - If that event-triggered retry also fails, NavigateToPose returns failure to `nf_basic.py`, so the existing path-guided intermediate-goal fallback remains available.
- Why: Cartographer map updates can change the global costmap frequently; periodic 1 Hz replanning makes the displayed/global path jump even when the current path is still usable. The new BT replans only when execution actually fails.
- Affected runs: debug `nf_basic.py` execution only; no official `hospital_v2` run affected.
- Does baseline need rerun?: no official baseline. If this execution policy is later adopted for benchmark runs, it must be protocol-versioned and shared across Nearest/MapEx/proposed methods.
- Observation: implementation committed; runtime behavior still needs validation.
- Next action: run `carto.launch.py + nf_basic.py`, verify `/plan` is normally published once per navigation attempt, then place/observe a newly discovered obstacle on the path and confirm a fresh plan appears only after planner/controller failure.

## 2026-08-28 — Revert Cartographer smoothness tuning after no improvement

- What changed:
  - Restored `TRAJECTORY_BUILDER_2D.use_online_correlative_scan_matching = true`.
  - Restored Cartographer OccupancyGrid `publish_period_sec` from `3.0` back to `1.0`.
- Why: disabling online correlative scan matching and slowing `/map` publication did not remove the visible robot jerk, so those Cartographer changes are no longer justified as performance fixes.
- Affected runs: Cartographer debug only; no official `hospital_v2` run affected.
- Does baseline need rerun?: no. Cartographer remains a debug mapping alternative.
- Observation: final `/cmd_vel` Twist values were changing smoothly, while the robot still appeared jerky. Together with the low wall-time `/scan` and `/odom` rates, the current working hypothesis is Gazebo/physics real-time performance rather than Cartographer local-matching load.
- Next action: keep the restored accuracy-first Cartographer config fixed and diagnose Gazebo real-time factor / headless rendering separately.

## 2026-08-28 — Disable online correlative scan matching for smoothness A/B test

- What changed: `TRAJECTORY_BUILDER_2D.use_online_correlative_scan_matching` changed from `true` to `false` in `config/cartographer_hospital_2d.lua`.
- Why: the Cartographer debug stack navigates correctly but the simulated robot appears visibly jerky; measured wall-time topic rates were about `/cmd_vel=14 Hz`, `/odom=14 Hz`, `/scan=2.6 Hz`, suggesting the simulator is running well below real time. This A/B isolates the relatively expensive online correlative local matcher without changing frontier logic.
- Affected runs: Cartographer debug only; no official `hospital_v2` run affected.
- Does baseline need rerun?: no. Cartographer is still a debug mapping alternative and has not been promoted to the official protocol.
- Observation: runtime result pending. Wheel odometry, IMU fusion, Ceres scan matching, submaps, pose-graph optimization and loop closure remain enabled.
- Next action: rerun the same Cartographer + `nf_basic.py` setup, compare visual smoothness and `/scan`/`/odom` wall-time rates, then revisit old corridors to check whether map alignment remains acceptable.

## 2026-08-28 — Fix Cartographer IMU sensor-frame TF mismatch

- What changed:
  - First `carto.launch.py` runtime showed Cartographer repeatedly failing to transform IMU messages because `/imu.header.frame_id` is `turtlebot4/imu_link/imu`, while robot TF exposes the physical sensor link as `imu_link`.
  - Added an identity static transform `imu_link -> turtlebot4/imu_link/imu` in `launch/carto.launch.py`. The IMU sensor has zero extra pose relative to `imu_link`, so this reconnects the Gazebo sensor-scoped message frame to the real robot TF tree without disabling IMU fusion.
- Why: Cartographer started and loaded its trajectory successfully, but the missing IMU source frame prevented normal sensor processing, so `map` was never created and Nav2 subsequently timed out waiting for `map -> base_link`.
- Affected runs: Cartographer debug only; no official `hospital_v2` run affected.
- Does baseline need rerun?: no official baseline. The failed Cartographer launch is diagnostic only.
- Observation: runtime error was a TF integration problem, not Cartographer package/config loading failure.
- Next action: pull and rerun `carto.launch.py`; verify the IMU TF warning disappears, `/map` is published at 0.10 m/cell, and `map -> odom -> base_link` is available before running `nf_basic.py`.

## 2026-08-28 — Add Cartographer 2D debug mapping alternative

- What changed:
  - Added `config/cartographer_hospital_2d.lua` and `launch/carto.launch.py` as a replacement mapping stack for debug validation while retaining the same Hospital simulation and stock-style Nav2 debug profile.
  - Cartographer consumes `/scan`, `/odom`, and `/imu`; tracking frame is `imu_link`, Gazebo keeps publishing `odom -> base_link`, and Cartographer publishes the loop-closed `map -> odom` relation.
  - Local Cartographer submaps use `0.05 m/cell`; the ROS `/map` OccupancyGrid exposed to Nav2 and `nf_basic.py` remains `0.10 m/cell`.
  - Loop closure remains enabled through Cartographer's pose graph. Online correlative scan matching is enabled and pose-graph optimization is requested every 60 nodes for an accuracy-first first test.
  - LiDAR used by Cartographer is limited to `12 m` rather than the full simulated 20 m to reduce long-range ambiguous corridor structure during scan matching.
- Why: the latest long exploration run returned to previously mapped space with visible map misalignment, after which MPPI repeatedly failed with `PATIENCE_EXCEEDED`; the working hypothesis is accumulated SLAM drift/map inconsistency rather than a frontier-ranking failure.
- Affected runs: debug Cartographer runs only. `hospital_v2` official protocol still names `slam_toolbox` until Cartographer is runtime-validated and deliberately adopted as a new protocol version.
- Does baseline need rerun?: not yet because no official Cartographer protocol has been adopted. If Cartographer becomes the official mapping stack, Nearest/MapEx/proposed-method official runs must all use it and the protocol version must change.
- Observation: code/config implemented; runtime validation is still pending.
- Next action: install Jazzy Cartographer packages, run `carto.launch.py + nf_basic.py`, verify `/map`, `map -> odom`, `/imu`, and `/odom`, then drive/explore far enough to revisit old corridors and compare map overlap against the slam_toolbox run.

## 2026-08-28 — Robust conservative completion guard for `nf_basic.py`

- What changed:
  - `nf_basic.py` no longer prints completion after a single empty frontier scan.
  - COMPLETE now requires zero frontier regions larger than 10 cells across `5` distinct `/map` generations, with terminal sweeps spaced by at least `2 s`, navigation idle for at least `10 s`, and node age at least `20 s`.
  - Any reappearance of a large frontier resets terminal verification immediately.
  - If large frontier regions still exist but every representative is inside the debug `MIN_DISTANCE_THRESHOLD=0.5 m`, the node publishes `BLOCKED_BY_MIN_DISTANCE` and explicitly does **not** claim exploration completion.
  - A pending main frontier or recovery subgoal also prevents completion; this keeps the no-blacklist/same-main-goal recovery semantics conservative.
  - Added transient-local `/frontier_exploration_complete` (`std_msgs/Bool`) and `/frontier_exploration_status` (`std_msgs/String` JSON) so terminal state is machine-readable and visible to late subscribers while the node remains alive.
  - After COMPLETE, markers/path are cleared and the node permanently stops issuing new navigation goals.
- Why: a single transient SLAM map with no detected frontier can create a false completion, while the debug 0.5 m filter can hide still-existing frontier regions. The completion signal should prefer non-termination over falsely declaring the map explored.
- Affected runs: debug `stock.launch.py + nf_basic.py` only; not current `hospital_v2` official baseline semantics.
- Does baseline need rerun?: no official benchmark uses this debug file. If adopted later, completion semantics must be protocol-versioned and shared across compared methods.
- Observation: implementation committed; runtime terminal validation pending.
- Next action: run until the end, verify `VERIFYING_COMPLETE` progresses across fresh map updates, then confirm exactly one `COMPLETE` and `/frontier_exploration_complete: true`, with no later goal dispatch.

## 2026-08-28 — Debug path-guided intermediate-goal recovery for `nf_basic.py`

- What changed:
  - Added `behavior_trees/navigate_to_pose_subgoal.xml`, a lightweight NavigateToPose tree that keeps 1 Hz global replanning but omits the stock Nav2 recovery loop so planner/controller failure returns to `nf_basic.py` quickly.
  - `nf_basic.py` now keeps the selected frontier as a persistent **main goal**, stores only the latest `/plan` produced while navigating to that main goal, and does not blacklist or switch frontier after execution failure.
  - When the main NavigateToPose fails and a usable main-goal path exists, `nf_basic.py` picks a temporary subgoal on that path. Target distance is half the remaining path, capped at `0.70 m`, normally at least `0.25 m`, while trying to keep `0.15 m` separation from the main frontier.
  - If the temporary subgoal succeeds, the exact same frontier is sent again as the main goal so Nav2 replans from the new robot pose. If the subgoal also fails, the same main frontier remains selected and is retried; no alternate frontier/blacklist is introduced.
  - Ctrl-C shutdown now checks `rclpy.ok()` before calling `rclpy.shutdown()` to avoid the previous double-shutdown RCLError.
- Why: the stock BT was observed to report `Failed to make progress` repeatedly while internally cycling through recovery actions for a long time. The test hypothesis is that a short path-aligned motion can move the robot out of a local controller deadlock without changing Nearest Frontier ranking.
- Affected runs: debug `stock.launch.py + nf_basic.py` only. This is not yet `hospital_v2` official benchmark execution semantics.
- Does baseline need rerun?: no official run exists under this debug recovery. If this recovery is adopted for the benchmark, it must become a protocol-versioned shared execution adapter and be applied to Nearest, MapEx and the proposed method before official comparisons.
- Observation: implementation is committed but not yet runtime-validated.
- Next action: reproduce the previously stuck frontier, verify log sequence `Main goal failed -> Path-guided recovery -> Subgoal reached -> retry same main frontier`, and check that the robot actually changes pose before replanning.

## 2026-08-27 — Switch Hospital runtime map/policy grid to 0.10 m while preserving evaluation grid

- What changed:
  - Protocol tăng từ `hospital_v1` lên `hospital_v2`.
  - `hospital_slam.yaml` và `hospital_slam_no_loop.yaml` đổi OccupancyGrid runtime resolution `0.05 -> 0.10 m/cell`.
  - Nearest và MapEx config được pin runtime frontier/prediction resolution `0.10 m/cell`.
  - `hospital_canvas_v1` vẫn giữ `0.05 m`, `1504 x 2123`, origin `(-25.6,-60.1)`; `hospital_connected_free_v1`, denominator `215435` và ROI SHA không đổi.
  - Generic `exploration_recorder.py` và legacy `mapex_nearest_full.py` giờ chấp nhận source map có resolution là integer multiple của `0.05`; map `0.10` được nearest-neighbour expand thành `2 x 2` cell trước khi paste lên fixed canvas.
- Why: MapEx gốc làm policy/prediction trên khoảng `0.10 m/pixel`. Dùng runtime SLAM `0.10` làm Nearest frontier, MapEx frontier và LaMa prediction chung một grid, loại bỏ bước downsample `0.05 -> 0.10` trong `control_tb4_mapex.py` và tránh candidate-set mismatch giữa Nearest/MapEx.
- Affected runs: mọi run mới sau thay đổi phải ghi `protocol_version=hospital_v2`. Các pilot v1 chỉ giữ làm diagnostic/history và không được trộn thống kê với v2.
- Does baseline need rerun?: có đối với mọi comparison chính thức. Chưa có official baseline hợp lệ nên chưa mất benchmark chính thức; cần chạy lại Nearest dưới v2 trước khi so full MapEx.
- Observation: evaluation ROI không cần regenerate vì runtime grid và evaluation grid được tách rõ. Raw map vẫn lưu 0.10; metric dùng reprojected fixed canvas 0.05.
- Next action: rebuild runtime, xác nhận `/map.info.resolution=0.10`, chạy Nearest v2 pilot và verify raw snapshot resolution 0.10 + fixed canvas shape `(2123,1504)` + coverage hợp lệ.

## 2026-08-26 — Finalize Nearest official-run schema before benchmark

- What changed:
  - Thêm `mapex_nearest_ros_official.py`: benchmark clock bắt đầu trước candidate computation đầu tiên, lưu exact exhausted/no-candidate decision, thêm stable `candidate_id` và selected-candidate topic.
  - Thêm `research_recorder_official.py`: đồng bộ `t=0` theo policy timestamp, hash source + installed runtime files, ghi rõ simulator seed policy.
  - Siết `validate_nearest_run.py`: kiểm tra metrics/snapshot integrity, candidate join, PENDING/failure reason, provenance và clean git cho official runs.
  - Đồng bộ `EXPERIMENT_PROTOCOL.md`, `DATA_SCHEMA.md`, `ROADMAP.md`, `config/nearest.yaml` với Hospital below-1m adaptation.
  - Raw `experiments/**` được ignore toàn bộ để pilot output không làm git worktree dirty.
- Why: tránh bias thời gian decision đầu, tránh mất exact termination state, tránh join candidate bằng suy đoán và tránh phải rerun vì thiếu provenance/data.
- Simulator seed: intentionally uncontrolled Gazebo default; dùng repeated-run statistics thay vì giả định fixed seed.
- Affected runs: mọi pilot trước `nearest_pilot_007` không có toàn bộ final schema. Không pilot nào dùng làm official benchmark.
- Does baseline need rerun?: chưa có official baseline nên không mất kết quả chính thức.
- Next action: chạy `nearest_pilot_007`; preflight + live validator + final validator đều PASS thì khóa code và bắt đầu `nearest_001...nearest_010`.

## 2026-08-26 — Disable MapEx 1 m rejection for Hospital

- What changed: thêm `scripts/mapex_nearest_ros_hospital_adapted.py` và chuyển Hospital runtime sang semantics này. Hospital vẫn detect/cluster/represent/rank frontier theo MapEx, nhưng candidate `<1 m` không còn bị reject chỉ vì khoảng cách; nó được gửi sang Nav2 `ComputePathToPose` như candidate bình thường.
- Why: `nearest_pilot_005` chứng minh startup deadlock do rule gốc: `534` frontier cells, `24` regions, chỉ `1` large region `>10`, chỉ `1` representative ở `0.469 m`, candidate duy nhất bị reject trước Nav2.
- Affected runs: `nearest_pilot_005` invalid cho benchmark; dùng làm evidence cho protocol adaptation.
- Does baseline need rerun?: chưa có official run nên chưa mất benchmark nào. Tất cả official Nearest và full MapEx Hospital sau này phải dùng cùng below-1m adaptation để đảm bảo fairness.
- Observation: vấn đề nằm trước Nav2; candidate 0.469 m chưa từng được gửi planner trong pilot_005.

## 2026-08-26 — Replay-safe logging before official Nearest runs

- What changed:
  - Exact frozen `decision_map` được policy process lưu trực tiếp, gồm raw OccupancyGrid + fixed-canvas copy.
  - Lưu map-frame robot pose dùng cho ranking, full candidate set/rank, `below_1m` flag, Nav2 path status, planner timing, selected path length và execution result.
  - Recorder lưu periodic raw/canvas snapshots mỗi 10 s + final map để tính occupied IoU/TU offline.
  - `decisions.csv` thêm `policy_decision_id`, coverage, map-frame pose, navigation detail/failure reason.
  - Metadata thêm git dirty state và SHA-256 code/config quan trọng.
- Why: tránh phải rerun Nearest chỉ vì thiếu snapshot/candidate/failure/computation data khi chuyển sang stage analysis, oracle/ablation hoặc offline IoU/TU.
- Affected runs: pilot cũ trước schema này không dùng làm official benchmark.
- Does baseline need rerun?: chưa có official baseline.

## 2026-08-26 — Replace custom WFD Nearest with official MapEx Nearest policy

- What changed: research Nearest baseline không còn dùng `frontier_autonomy.launch.py` / WFD custom. Thêm `scripts/mapex_nearest_ros.py` port policy `nearest` từ `castacks/MapEx` commit `53636bd1c79153acc3c74a532837d78c926bae5e`.
- Why: mục tiêu là so Hospital MapEx với chính nearest-frontier baseline semantics mà implementation MapEx công bố, tránh thay đổi frontier generator làm confound kết quả.
- Official policy pinned: free cell kề unknown theo 8-neighbour; frontier regions 8-connected; giữ region khi size `>10`; representative là frontier cell gần arithmetic mean nhất; score bằng Euclidean distance; MapEx gốc có `cur_pose_dist_threshold_m=1.0`; no A* path thì reselect next-lowest cost frontier.
- ROS adaptation: Hospital thay simulator execution bằng Nav2 `ComputePathToPose` + `NavigateToPose`. Goal vẫn là đúng frontier-center, không dùng WFD standoff/goal shifting. Sau pilot_005, Hospital disable riêng 1 m rejection như note phía trên.
- Affected runs: WFD pilot chạy dở trước thay đổi này bị dừng/xóa và không được tính.
- Does baseline need rerun?: chưa có official benchmark.

## 2026-08-26 — Freeze Hospital ROI v1

- What changed: generate và freeze `hospital_connected_free_v1`.
- Denominator: `215435` cells.
- Mask SHA-256: `05d45b7aba66cb6dbb71e0406005f4a3e21875901af3ae72164b17f1d3add8d1`.
- Structural audit: `1040` unique wall segments, `2` elevator blockers.
- Why: coverage cần một denominator cố định giống hệt giữa Nearest và MapEx.
- Affected runs: toàn bộ run dùng protocol `hospital_v1`.
- Does baseline need rerun?: chưa có official baseline run. Sau khi baseline bắt đầu, mọi thay đổi ROI yêu cầu protocol/ROI ID mới và baseline rerun.

## 2026-08-26 — Freeze Hospital canvas and ROI definition

- What changed: chốt `hospital_canvas_v1` và định nghĩa `hospital_connected_free_v1`.
- Why: coverage cần denominator cố định, không phụ thuộc crop/map extent của từng run.
- Affected runs: toàn bộ Nearest/MapEx run mới trong lộ trình `mapex_hospital_research`.
- Does baseline need rerun?: chưa; benchmark mới chưa bắt đầu.
- Observation: full Hospital stack đã được xác nhận chạy bình thường; fixed canvas là 0.05 m, 1504 x 2123, origin (-25.6, -60.1).

## Initial note

Workspace mới được tạo để làm lại nghiên cứu từ đầu. Chưa có run official mới và chưa chốt bottleneck.
# 2026-09-23 — R004 offline prediction-quality execution

R004 V3 initially could not score the full future-observed target universe
because historical predictions have dynamic geometric support. CODEX stopped
before scoring; V4 was then independently accepted to report conditional
accuracy on scoreable support and expose unsupported coverage separately.

The final V3+V4 evaluator ran on all ten historical MapEx runs with no
exclusion or final-snapshot fallback. Results remain pending independent review
and must not be treated as a scientific prediction-quality verdict yet. Fixed
total-support and class-composition sensitivities had no eligible final-bin
decisions under the preregistered sample sizes; the evaluator reports this
without relaxing the rule.

## 2026-10-09 - MX071 R2 DELL deployment and report-write recovery

- User chose DELL for MX071; only the accepted offline 2D Stage 1 is executed.
- Built isolated Shapely 1.8.0, pyastar2d 1.0.2 (official source pin), and PIPE range_libc against the existing LaMa environment. System libstdc++ is preloaded per process; no system/environment overwrite.
- Completed source parity, real-model complete-state five-move replay, 15 scientific contract checks, exact loading of all three generators, and full-size CPU probes. Largest-map G1 peak RSS was 4725.40 MiB with 631.70 MiB available; the three-member first-map inference sum was 158.25 s. Resource guard V3 was amended and tested before collection.
- First map `50052750` finished both sources at 100 m, producing 20 logical first-goal requests. Six snapshots cover seven target slots; one target remains NA. H1/H4/H7 support is incomplete with one map, so no positive direction claim is made.
- The batch then exited at `direction_screening.md` writing because Python 3.6 selected ASCII and the heading contained an em dash. All source final states, snapshots, model outputs, branch outcomes, and JSON/CSV summaries had already been saved.
- Recovery recorded protected file hashes and kept the same sealed Python files. Resume uses `LC_ALL=C.UTF-8 LANG=C.UTF-8 PYTHONIOENCODING=utf-8`; completed sources and requests are skipped. The hash check after resume confirmed the protected numerical data unchanged.
- Stage 2 remains closed. No replacement maps, threshold adjustment, robot run, or independent result verdict. This execution is MX071, separate from MX072.
- Caveats: five-move replay is a technical fixture; map IDs are not verified building identities; interrupted physical branches restart from the original snapshot; complete cumulative cost telemetry across resumes is not claimed.

### Same-session follow-up: completed-request resume guard

The initial UTF-8 resume recomputed offline diagnostics for completed first-map snapshot groups before skipping their recorded selector outcomes. That process was stopped while recomputing diagnostics; no source or physical branch was interrupted. Added a three-line guard that skips only when all five selector contracts are recorded. The missing-selector negative case still enters execution. Four focused checks passed in 0.16 s after imports, without additional numerical rollouts; protected completed states/results remain unchanged. The original manifest and implementation commit `b008d997a3332c758347a09cc12c5cbe594cd487` are preserved, and the amended manifest binds RESUME_FIX1 with unchanged model/map/controller/scoring/budget fields. Active log: `/home/dell/mx071_dell_20261009/logs/stage1_resume_fix1.log`.


## 2026-10-10 — Correct MapEx/PIPE question provenance and open follow-up

The original eight brainstormed omissions concern MapEx (Agent1 journal §17), including #8; verification of strong PIPE is a separate priority (§16/§17.5). MX071 R2 answers a subset on PIPE_ALIGNED source snapshots and common PIPE continuation. USER now requests a direct MapEx rerun and PIPE investigation, and selected all open questions; #7 remains dropped.

Preparation branch `mx071-mapex-followup-pipe-audit-20261010` preserves the old sealed runner/data. The direct-source harness comes from technical commit 8bf7b96f388298361a1c0dd6c32d52041ffcb814, with imports/temp-prefix only changed. On DELL: pinned used-source bytes and G/G1/G2/G3 hashes match; preprocessing retains native unshifted candidate coordinates on padded tensors; runtime confirms unknown-as-occupied request is effectively False. Two four-step stand-in source repetitions are identical. This is bounded source repeatability, not universal branch parity or actual scientific acquisition.

R0 records per-question evidence requirements. #2/#3 diagnostics alone do not identify policy benefit; #5 needs a commitment intervention and #6 an explicitly expanded pose pool. Full seven-question collection, real-model/resource preflight and native complete-state forks remain unimplemented/unsealed. No full preflight PASS, scientific READY, independent verdict, online improvement, teacher-facing result or COM1 modification.

## 2026-10-10 — IR1 R0 REVISE and bounded R1 corrections

IR1 review 63cd2ce00166284ad298d6b9fafd4b33c058a3c3 / return 46758a76ae0c827afec5a962ddf116aea0cbb629 requests four corrections. Maker agrees: G/all-train alone is not MapEx visvarprob's final prediction state; native scoring epochs miss active-lock fresh observations; the 100m discrete crossing rule was ambiguous; and PIPE source-step and distance-prefix budgets were conflated.

Current PLAN_R1.md and PIPE_AUDIT_R1.md fix those proposal semantics, including native-versus-shadow labels, two nomination strata, actual last post-scan pose/map at budget and common endpoint-held Q, and separate configured/effective PIPE limits. Exact PIPE explore.py uses category/log_iou-derived time_step as its loop bound; mission_time alone is not the effective-limit certificate. REVIEW_RESPONSE_R1.md maps the finite closure request and preserves PASS dimensions.

This is a document correction, not an executable seven-question seal. R0 files, Python and both original readiness/completed-science evidence are retained unchanged; no fixture/model rerun, new source/branch, COM1 change or teacher-facing scientific result. Request IR1 focused R0-1..4 closure/regression; full interventions/replay/ceiling/resource/storage/time gates remain open.


## 2026-10-10 — USER direct R2 drafting; full seven-question causal package

IR1 independently ACCEPTED bounded R1 semantics at5f9a60339ebd4e96ffcf115e8f9ad520ab66f405, review5477923365. USER then published all-seven causal requirements and directly instructed this maker session to complete design and send it back to IR. This authoring authorization overrides waiting for PM pickup for the bounded draft only; no role self-bind, runtime, staff transfer or merge.

R2 defines one shared native acquisition, complete-state replay/common continuation, paired actual action contracts for1–6/8, costed outcomes and positive/negative/insufficient-support gates. #5 includes KEEP/FRESH_ONCE/computation sham; #6 separates within-cluster views, expanded pool and cheap geometry. #8 separates unique G10 reversal, tied-prefix ambiguity and whether native actually chose the short winner. Topology/visibility/future-best oracles remain non-online.

New request ceiling176 scientific+16 KEEP=192 before dedup, eight sources additional. Fresh-model/resource preflight and full P19 native suffix replay remain unperformed. A lossless CAS/verified decoder is specified; no quantization/crop/snapshot thinning/candidate or failed-run omission. Full48GiB/staged32GiB storage options are proposed, not selected. Read-only DELL facts: i3-7100U/8GB class, about6.6GiB free; current disk gates fail and time/RSS feasibility is uncertified. Scenario bounds are not measured ETA.

Read pinned git objects, not a model/simulator import. Newly explicit source facts: configs/base.yaml has mission_time1000 and distance_transform=True; helper ignores requested unknownAsOcc and passesFalse. get_free_points resets accum_hit_prob at each cell, so retain source per-cell0.8 behavior rather than replacing it with a paper-derived accumulation rule. Capture G auxiliary before later batch-dict mutation and preserve its actual RGB/BGR extraction. These are fidelity facts, not demonstrated weaknesses or new PIPE hypotheses.

PLAN_R1/PIPE_AUDIT_R1/R0/Python/readiness/old completed evidence are preserved. Only design documents/JSON and status/reference metadata change. No new source, branch, fixture, inference, teacher-facing result, MX072/COM1 change, independent ACCEPT, READY or START.
