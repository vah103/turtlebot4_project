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
