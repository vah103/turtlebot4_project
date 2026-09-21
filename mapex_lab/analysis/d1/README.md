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

P2 **không dùng** `new_room_connected_free_v2.npy` làm classification mask vì connected-free ROI chỉ biểu diễn free-space ROI và sẽ loại mất occupied class. P2 cần giữ cả free và occupied target.

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
