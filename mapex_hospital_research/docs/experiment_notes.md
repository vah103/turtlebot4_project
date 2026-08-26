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

## 2026-08-26 — Replay-safe logging before official Nearest runs

- What changed:
  - `hospital_nearest.launch.py` chuyển sang `mapex_nearest_ros_research.py`, `exploration_manager_research.py`, `research_recorder_safe.py`.
  - Exact frozen `decision_map` được policy process lưu trực tiếp, gồm raw OccupancyGrid + fixed-canvas copy.
  - Lưu map-frame robot pose dùng cho ranking, full candidate set/rank, `<1 m` rejection, Nav2 path status, planner timing, selected path length và execution result.
  - Recorder lưu periodic raw/canvas snapshots mỗi 10 s + final map để tính occupied IoU/TU offline.
  - `decisions.csv` thêm `policy_decision_id`, coverage, map-frame pose, navigation detail/failure reason.
  - Metadata thêm git dirty state và SHA-256 code/config quan trọng.
- Why: tránh phải rerun Nearest chỉ vì thiếu snapshot/candidate/failure/computation data khi chuyển sang stage analysis, oracle/ablation hoặc offline IoU/TU.
- 1 m semantic: rank toàn bộ frontier trước; selected candidate nếu `<1 m` thì reject; thử candidate rank tiếp theo; sau đó mới Nav2 path validation. Không dùng 1 m làm detection filter trước ranking.
- Affected runs: mọi pilot cũ trước `nearest_pilot_005` không có full replay-safe schema và không dùng làm official benchmark.
- Does baseline need rerun?: chưa có official baseline, nên không mất kết quả chính thức. Chỉ cần chạy một pilot cuối `nearest_pilot_005` để validate logger trước `nearest_001`.
- Observation: robot ở pilot gần nhất đã chạy được; run bị dừng chủ động để hoàn thiện logger trước benchmark.
- Next action: pull, py_compile, chạy `nearest_pilot_005`, kiểm tra output files; pass thì bắt đầu official runs.

## 2026-08-26 — Replace custom WFD Nearest with official MapEx Nearest policy

- What changed: research Nearest baseline không còn dùng `frontier_autonomy.launch.py` / WFD custom. Thêm `scripts/mapex_nearest_ros.py` port policy `nearest` từ `castacks/MapEx` commit `53636bd1c79153acc3c74a532837d78c926bae5e`.
- Why: mục tiêu là so Hospital MapEx với chính nearest-frontier baseline semantics mà implementation MapEx công bố, tránh thay đổi frontier generator làm confound kết quả.
- Official policy pinned: free cell kề unknown theo 8-neighbour; frontier regions 8-connected; giữ region khi size `>10`; representative là frontier cell gần arithmetic mean của region nhất; score bằng Euclidean distance; `cur_pose_dist_threshold_m=1.0`; no A* path thì reselect next-lowest cost frontier.
- ROS adaptation: MapEx gốc dùng `pyastar2d` grid A*; Hospital thay execution bằng Nav2 `ComputePathToPose` + `NavigateToPose`. Goal vẫn là đúng frontier-center, không dùng WFD standoff/goal shifting.
- Affected runs: WFD pilot chạy dở trước thay đổi này bị dừng/xóa và không được tính. Chưa có official Nearest run nên không cần rerun kết quả hợp lệ nào.
- Does baseline need rerun?: pilot phải chạy lại từ đầu với MapEx-nearest adapter; official benchmark chưa bắt đầu.
- Observation: frontier-generation semantics bây giờ có thể dùng chung cho Nearest và MapEx; khác biệt chính sau này nằm ở score/ranking.
- Next action: pull, syntax-check và chạy pilot mới.

## 2026-08-26 — Freeze Hospital ROI v1

- What changed: generate và freeze `hospital_connected_free_v1`.
- Denominator: `215435` cells.
- Mask SHA-256: `05d45b7aba66cb6dbb71e0406005f4a3e21875901af3ae72164b17f1d3add8d1`.
- Structural audit: `1040` unique wall segments, `2` elevator blockers.
- Why: coverage cần một denominator cố định giống hệt giữa Nearest và MapEx.
- Affected runs: toàn bộ run dùng protocol `hospital_v1`.
- Does baseline need rerun?: chưa có official baseline run. Sau khi baseline bắt đầu, mọi thay đổi ROI yêu cầu protocol/ROI ID mới và baseline rerun.
- Observation: ROI generator dùng COLLADA unit/up-axis normalization + unique plane-intersection segments trước rasterization, sau đó lấy 8-connected free component chứa spawn.
- Next action: chạy pilot với research recorder để xác nhận metrics/alignment/termination trước official Nearest runs.

## 2026-08-26 — Freeze Hospital canvas and ROI definition

- What changed: chốt `hospital_canvas_v1` và định nghĩa `hospital_connected_free_v1`.
- Why: coverage cần denominator cố định, không phụ thuộc crop/map extent của từng run.
- Affected runs: toàn bộ Nearest/MapEx run mới trong lộ trình `mapex_hospital_research`.
- Does baseline need rerun?: chưa; benchmark mới chưa bắt đầu. Nếu ROI thay đổi sau baseline thì phải chạy lại baseline.
- Observation: full Hospital stack đã được xác nhận chạy bình thường; fixed canvas hiện có là 0.05 m, 1504 x 2123, origin (-25.6, -60.1).
- ROI rule: structural obstacles = wall mesh + flat elevator blockers; rasterize ở z=0.30 m, thickness 2 cells, close one-cell cracks; trong Hospital bounds lấy 8-connected free component chứa robot start `(0,0)` ở SLAM-start frame.

## Initial note

Workspace mới được tạo để làm lại nghiên cứu từ đầu. Chưa có run official mới và chưa chốt bottleneck.
