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
- Why: `nearest_pilot_005` chứng minh startup deadlock do rule gốc: `534` frontier cells, `24` regions, chỉ `1` large region `>10`, chỉ `1` representative, distance `0.469 m`; `>=1m_valid=0`; candidate duy nhất bị reject trước Nav2 và robot không thể bắt đầu exploration.
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
