# Direction 1 Analysis

Thư mục dành riêng cho phân tích và phát triển Direction 1 (D1) — Uncertainty-Aware Predicted Map Completeness.

## Phase 0

- `d1_gate_p.py` — Gate P: prediction fidelity.
- `d1_gate_u.py` — Gate U: uncertainty informativeness (chưa triển khai).
- `d1_gate_r.py` — Gate R: remaining-area informativeness (chưa triển khai).

Code trong thư mục này phục vụ phân tích offline và không thay đổi MapEx runtime baseline.

## Gate P hiện tại

`d1_gate_p.py` dùng cohort baseline New Room mặc định:

```text
mpx_001 ... mpx_010
```

Với mỗi decision, script:

1. đọc exact observed raw map tại decision đó;
2. lấy các cell còn unknown;
3. crop G1/G2/G3/mean prediction về đúng runtime grid;
4. chuyển cell center qua world coordinates;
5. tra các cell đó trong final raw map của cùng run;
6. chỉ giữ cell đã trở thành known về cuối run;
7. tính prediction fidelity cho G1/G2/G3 và ensemble mean.

Primary classification threshold:

```text
predicted free     : prediction < 0.5
predicted occupied : prediction >= 0.5
```

### Chạy mặc định

Từ repo root:

```bash
python3 mapex_lab/analysis/d1/d1_gate_p.py
```

Hoặc chỉ chạy một số run:

```bash
python3 mapex_lab/analysis/d1/d1_gate_p.py \
  --runs mpx_001 mpx_002 mpx_003
```

Bỏ figure nếu chỉ muốn CSV/JSON:

```bash
python3 mapex_lab/analysis/d1/d1_gate_p.py --no-figures
```

## Output

Mặc định ghi vào:

```text
mapex_lab/analysis/d1/results/gate_p/
├── gate_p_decisions.csv
├── gate_p_runs.csv
├── gate_p_summary.json
└── figures/
    ├── accuracy_vs_decision_progress.png
    ├── macro_iou_vs_decision_progress.png
    ├── mae_vs_decision_progress.png
    └── late_stage_accuracy_by_run.png
```

### gate_p_decisions.csv

Một dòng cho mỗi MapEx decision, gồm:

- số cell unknown;
- số cell có later-observed target;
- free/occupied class support;
- accuracy;
- free precision/recall;
- occupied precision/recall;
- free/occupied IoU;
- macro IoU;
- MAE;
- các metric riêng cho mean, G1, G2, G3;
- progress/stage và last-N marker.

### gate_p_runs.csv

Aggregate theo run, gồm micro metrics và decision-macro metrics, cộng riêng late-stage metrics.

### gate_p_summary.json

Tổng hợp toàn cohort, stage early/mid/late, run-macro mean/std và cảnh báo provenance như metadata run-id mismatch.

## Reference semantics

Gate P hiện dùng:

```text
unknown at decision t
AND
known in final raw map of the same run
```

làm `later-observed` offline target.

Final/future map chỉ dùng để đánh giá fidelity offline, không được dùng làm feature của D1 runtime.
