# Experiment Notes

Ghi ngắn gọn các quyết định hoặc sự cố có thể ảnh hưởng kết quả.

## Template

### YYYY-MM-DD — <run or change>

- What changed:
- Why:
- Affected runs:
- Does baseline need rerun?:
- Observation:
- Next action:

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
