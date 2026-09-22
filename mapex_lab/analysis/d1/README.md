# Direction 1 Analysis

Thư mục dành riêng cho phân tích và phát triển Direction 1 (D1) — Uncertainty-Aware Predicted Map Completeness.

## Research log

- `D1_RESEARCH_LOG.md` — nhật ký nghiên cứu D1 theo thời gian: quyết định, protocol, kết quả, limitation và next action.

## Phase 0

- `d1_gate_p.py` — Gate P: prediction fidelity, gồm P1 later-observed và P2 structural GT.
- `d1_gate_u.py` — Gate U: uncertainty informativeness (chưa triển khai).
- `d1_gate_r.py` — Gate R: remaining-area informativeness (chưa triển khai).

Code trong thư mục này phục vụ phân tích offline và không thay đổi MapEx runtime baseline.

## Gate P

Cohort New Room mặc định:

```text
mpx_001 ... mpx_010
```

Prediction được đánh giá riêng cho:

```text
mean
G1
G2
G3
```

Classification convention của D1:

```text
prediction < 0.5   → predicted free
prediction >= 0.5  → predicted occupied
```

Đây là **Gate-P convention**. Evaluator legacy dùng `prediction > 0.5` cho occupied; khác biệt chỉ tại đúng boundary `0.5`.

### P1 — later_observed

Domain:

```text
Unknown_t ∩ EventuallyObserved
```

Với mỗi cell unknown tại decision `t`, script tìm policy-decision đầu tiên sau `t` nơi cell trở thành known và dùng class đầu tiên đó làm target.

Final raw map chỉ là diagnostic phụ để tính `first_vs_final_class_agreement`.

### P2 — structural_gt

Domain:

```text
Unknown_t ∩ StructuralGT.evaluation_mask
```

P2 dùng:

```text
ground_truth/new_room/generated/new_room_structural_gt_v2.npz
```

mặc định.

Runtime map/prediction `0.10 m` được nearest-neighbour expand lên canonical structural-GT canvas `0.05 m` theo đúng semantics của evaluator hiện có trong repo.

Origin của raw SLAM map **không bắt buộc** nằm chính xác trên lattice `0.05 m`. P2 dùng cùng rule với evaluator canonical:

```text
canvas_index = int(round((runtime_origin - canvas_origin) / 0.05))
```

và ghi residual lượng tử hóa origin theo x/y để audit. Không hard-fail chỉ vì origin lệch fractional-cell.

P2 **không dùng** `new_room_connected_free_v2.npy` làm classification mask vì connected-free ROI chỉ biểu diễn free-space ROI và sẽ loại mất occupied class. P2 cần giữ cả free và occupied target.

## R002 — Task-aligned GT semantics diagnostic

R002 is a reviewed **post-hoc diagnostic/reformulation** after the negative Gate P2 result. It preserves Reference A / Gate P2 unchanged and adds two task-aligned diagnostics:

```text
A = existing structural-solid Gate P2 reference
B = occupancy-surface structural diagnostic
C0 = point-connected remaining-free diagnostic
```

Implementation:

```text
analysis/d1/r002_gt_semantics.py
analysis/d1/test_r002_gt_semantics.py
```

Accepted implementation source:

```text
feb94eaa9c1ba5aa4f9993792dae454bc4edcd60
```

### Frozen R002 semantics

Shared scoring universe:

```text
U_t = Unknown_t ∩ StructuralGT.evaluation_mask
```

Reference B:

```text
F_conn = new_room_connected_free_v2
GT_surface = StructuralOccupied ∩ Adjacent8(F_conn)
B_GT_t = GT_surface ∩ U_t
B_PRED_t = PredictedOccupiedBoundary_t ∩ U_t
```

Boundary matching is frozen as maximum-cardinality one-to-one, then minimum total Euclidean distance, then lexicographic `(GT_row, GT_col, Pred_row, Pred_col)` tie-break.

Fixed tolerances: primary `0.10 m`; sensitivities `0.05 m` and exact.

Reference C0:

```text
C0_TopologyDomain_t =
StructuralGT.evaluation_mask ∩ DecisionMapSupport_t

C0_GT_positive_t =
U_t ∩ new_room_connected_free_v2

C0_PRED_positive_t =
U_t ∩ PredictedFree_t ∩ PredPointConnectedFree_t
```

C0 uses canonical `0.05 m` topology, 4-connectivity, the exact projected robot cell only when observed-known-free, otherwise nearest observed-known-free seed within Euclidean `0.10 m`; no valid seed is recorded as `topology_invalid`.

Prediction threshold remains:

```text
prediction < 0.5  -> predicted free
prediction >= 0.5 -> predicted occupied
```

### First reviewed execution

Only `mpx_001` was authorized and executed.

Validation: Python compile PASS; 12/12 unit tests PASS; run exit code 0; 35/35 decisions; Gate-P / DecisionMapSupport parity = True; topology-invalid = 0; preregistered visual decisions = 6 / 18 / 29.

One-run summary: overall A free IoU ≈ 0.5001; overall B F1 @ 0.10 m ≈ 0.4178; overall C0 free IoU ≈ 0.5065; late C0 free IoU ≈ 0.0445; last-10 C0 free IoU ≈ 0.0335; last-10 B F1 @ 0.10 m ≈ 0.3725.

Independent implementation review verdict: **ACCEPT**.

This ACCEPT means implementation fidelity/evidence integrity passed review. It does **not** mean R002 scientifically rescues D1.

### Reproduce the approved one-run evaluator

```bash
python3 mapex_lab/analysis/d1/r002_gt_semantics.py \
  --run mpx_001 \
  --output-dir mapex_lab/analysis/d1/results/r002_mpx_001
```

Do not automatically run `mpx_002...mpx_010`. No threshold, tolerance, connectivity, seed or matching rule may be retuned from the `mpx_001` result while retaining that run as confirmatory data.

### Provenance guard cho P2

Trước khi chấm structural GT, script kiểm tra theo từng run:

- environment tương thích với GT ID;
- structural_ground_truth_id / structural_ground_truth_file;
- fixed_canvas_id / fixed_canvas_resolution_m;
- runtime_map_resolution_m so với exact decision raw maps;
- runtime resolution phải là integer multiple của GT resolution.

Field legacy bị thiếu → WARN. Field tồn tại nhưng mâu thuẫn → FAIL.

Riêng `metadata.environment` chỉ là provenance mô tả: missing/mismatch → `WARN`, không hard-fail.

Prediction NPZ được ghép với exact raw map của decision bằng hard integrity checks:

```text
source_height / source_width
resolution
origin_x / origin_y
source_map_stamp_s
member = G1 / G2 / G3 / mean
```

Chỉ khi toàn bộ spatial/source identity khớp thì prediction mới được dùng. Prediction `environment` mismatch chỉ WARN.

## Cách chạy

### Phân tích chính — P1 + P2

```bash
python3 mapex_lab/analysis/d1/d1_gate_p.py
```

Tương đương:

```bash
python3 mapex_lab/analysis/d1/d1_gate_p.py --reference both
```

### Chỉ P1

```bash
python3 mapex_lab/analysis/d1/d1_gate_p.py \
  --reference later_observed
```

### Chỉ P2

```bash
python3 mapex_lab/analysis/d1/d1_gate_p.py \
  --reference structural_gt
```

Có thể override GT:

```bash
python3 mapex_lab/analysis/d1/d1_gate_p.py \
  --reference structural_gt \
  --ground-truth /path/to/structural_gt.npz
```

Smoke test một run:

```bash
python3 mapex_lab/analysis/d1/d1_gate_p.py \
  --runs mpx_001 \
  --reference both \
  --no-figures
```

## Output

Mặc định:

```text
mapex_lab/analysis/d1/results/gate_p/
├── gate_p_decisions.csv
├── gate_p_runs.csv
├── gate_p_summary.json
└── gate_p_plots/
```

Khi dùng `--reference both`:

- `gate_p_decisions.csv`: hai row/reference cho mỗi decision, cột `reference` là `later_observed` hoặc `structural_gt`;
- `gate_p_runs.csv`: hai row/reference cho mỗi run;
- `gate_p_summary.json`: kết quả tách dưới `references.later_observed` và `references.structural_gt`, cộng block `p1_vs_p2`;
- `p1_vs_p2` so P2-minus-P1 cho overall và last-10: accuracy, macro IoU, MAE, free/occupied precision/recall/IoU; đồng thời lưu support P1/P2 theo toàn cohort và từng run;
- plot có prefix theo reference, ví dụ `later_observed_...` và `structural_gt_...`.

Cả P1 và P2 đều có:

- accuracy;
- free precision / recall;
- occupied precision / recall;
- free IoU / occupied IoU;
- macro IoU;
- MAE;
- early / mid / late;
- last-N, mặc định last 10;
- mean / G1 / G2 / G3.

Ngoài raw cell support, script ghi `evaluated_area_m2` và `reference_fraction_of_unknown_area`. Điều này quan trọng vì P1 chấm trên runtime cells `0.10 m`, còn P2 chấm trên canonical GT cells `0.05 m`; không so trực tiếp raw cell count giữa P1 và P2.

## P1 limitation

P1 chỉ chứng minh fidelity trên:

```text
Unknown_t ∩ EventuallyObserved
```

Nó không đánh giá vùng baseline không bao giờ reveal. Late-stage P1 vì vậy bị right-censoring mạnh.

## Vai trò của P2

P2 kiểm tra xem kết luận P1 có còn giữ khi bỏ selection bias `EventuallyObserved` hay không.

Mục tiêu sau khi chạy `both`:

```text
P1 vs P2
→ xem mpx_008 như case study
→ mean vs G1/G2/G3
→ nếu nhất quán → Gate U
```

Structural GT, final map và future observations chỉ là **offline evaluation targets**, không được dùng làm runtime feature của D1.
