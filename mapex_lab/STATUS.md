# mapex_lab status

_Last synchronized with `main`: 2026-09-17._

## Current focus

The active thesis direction is **Way2: frontier-utility-based early stopping for MapEx**.

- Way1 is closed.
- Way2 is still offline/development only; online MapEx has not yet been changed to stop early.
- `G` is frozen as raw `information_gain`.
- Historical replay `C` is frozen as Euclidean `distance_m`.
- `K=2` is the current fixed debounce guard.
- `lambda` is **not yet frozen**.

Core rule:

```text
V_t(f) = G_t(f) - lambda * C_t(f)
R_t    = max_f G_t(f)/C_t(f)
STOP when R_t <= lambda for K=2 distinct decision states
```

All current 17 runs are development/diagnostic data only:

- New Room: `mpx_001...mpx_015`
- Hospital: `hpx_001...hpx_002`

They must not later be presented as independent Way2 validation.

---

# Way1 — CLOSED

Frozen rule:

```text
unknown_variance_p95 <= 0.23, K=1
```

Prospective New Room validation `mpx_011...mpx_015`:

- trigger: **5/5**
- mean time saved: **24.37%**
- mean distance saved: **25.36%**
- mean IoU loss vs analyzer final reference: **-0.00201**
- worst IoU loss: **+0.000053**

Hospital cross-environment diagnostic:

- `hpx_001`: no trigger
- `hpx_002`: no trigger
- P95 remains near `1/3`

Conclusion: Way1 works in New Room but does not transfer cleanly to Hospital without retuning, so it is closed as the primary direction.

---

# Way2 — OPEN

## 1. Offline replay — COMPLETED

Replay baseline:

```text
G = information_gain
C = distance_m
score = G/C
candidate set = selectable == 1
```

Audit result over all evaluable decisions:

```text
score_mismatch = 0
selection_mismatch = 0
```

Therefore the offline replay reproduces current MapEx frontier ranking exactly.

## 2. Gain G — FROZEN

Compared:

```text
G_raw  = information_gain
G_norm = information_gain / visible_unknown_cells
```

Key result:

- normalized gain changes the best frontier heavily; agreement with raw MapEx ranking is only about **30%** of decisions.
- raw `G/C` already shows compatible post-early/late scale between New Room and Hospital.
- raw `G/C` decreases strongly toward the end of exploration in both environments.

Frozen definition:

```text
G_t(f) = information_gain(f)
```

Do not divide by `visible_unknown_cells`.
Do not use startup maxima to choose `lambda`.

## 3. Travel cost C — FROZEN FOR HISTORICAL REPLAY

Cost audit:

| Environment | Euclidean candidate coverage | Planner selected-goal coverage | Planner all-candidate upper bound | Median planner/Euclidean |
|---|---:|---:|---:|---:|
| New Room | 100.0% | 98.1% | 5.5% | 1.413 |
| Hospital | 100.0% | 89.6% | 6.7% | 1.479 |

Interpretation:

- `distance_m` exists for every selectable candidate.
- `plans.csv` provides planner path only after a frontier becomes the active selected goal.
- planner path therefore cannot be used fairly to rerank all historical candidates.

Frozen historical replay definition:

```text
C_t(f) = distance_m(f)
```

A future planner-cost variant would require `ComputePathToPose` for every selectable frontier at decision time and must be evaluated separately.

## 4. Lambda sensitivity — COMPLETED, NOT FROZEN

Script:

```text
scripts/analyze_way2_lambda.py
```

Sensitivity grid:

```text
lambda = 0.1, 0.2, 0.3, 0.4, 0.5, 0.75, 1.0, 1.5, 2.0
K = 2
```

Environment-level results:

| lambda | New Room trigger | NR median time saved | NR median distance saved | NR rebound runs | Hospital trigger | Hospital median time saved | Hospital median distance saved | Hospital rebound runs |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 0.1 | 4/15 | 14.0% | 8.3% | 0.0% | 2/2 | 8.7% | 7.8% | 100.0% |
| 0.2 | 10/15 | 16.8% | 9.8% | 20.0% | 2/2 | 13.1% | 12.4% | 50.0% |
| 0.3 | 13/15 | 17.9% | 14.7% | 23.1% | 2/2 | 13.1% | 12.4% | 50.0% |
| 0.4 | 14/15 | 19.1% | 15.4% | 14.3% | 2/2 | 14.4% | 13.7% | 50.0% |
| 0.5 | 14/15 | 19.1% | 15.4% | 14.3% | 2/2 | 15.0% | 14.1% | 50.0% |
| 0.75 | 14/15 | 22.5% | 16.6% | 28.6% | 2/2 | 29.9% | 29.9% | 50.0% |
| 1.0 | 15/15 | 28.8% | 24.3% | 33.3% | 2/2 | 33.1% | 33.8% | 50.0% |
| 1.5 | 15/15 | 34.7% | 31.6% | 33.3% | 2/2 | 41.8% | 43.9% | 50.0% |
| 2.0 | 15/15 | 38.6% | 37.0% | 40.0% | 2/2 | 42.6% | 44.7% | 50.0% |

Current interpretation:

- very low `lambda` (`0.1–0.2`) is conservative and often fails to trigger in New Room.
- high `lambda` (`>=0.75`) gives larger savings but more rebound/premature-stop risk.
- `lambda = 0.4–0.5` is currently the most useful region for deeper inspection: New Room has moderate savings and comparatively low rebound.
- Hospital still has rebound in **1/2 runs** throughout `lambda=0.2...2.0`, so the final lambda must not be frozen from the aggregate table alone.

`lambda` remains **NOT FROZEN**.

## 5. Noise guard

Current candidate guard:

```text
K = 2 distinct non-positive decision states
```

`K=2` is a debounce guard, not a parameter to sweep for maximum savings.

## 6. Evaluation caveat

Current `iou_loss_vs_final` uses the reconstructed map at the last analyzed policy decision as its reference, not necessarily the actual final observed SLAM map.

TU is weakly discriminative in the current dataset and should not be the primary stopping signal.

---

# Current next actions

1. Inspect per-run rebound details for `lambda=0.4` and `0.5`, especially `hpx_001` and `hpx_002`.
2. For every rebound, measure how far `R_t` rises above `lambda` and how long after the hypothetical stop it occurs.
3. Check stop timing and map-quality loss for the candidate lambda region.
4. Freeze one lambda only after this failure-mode analysis.
5. Backtest the frozen complete rule on the 17 development runs.
6. Integrate the frozen rule into online MapEx.
7. Collect new independent New Room + Hospital validation runs with no environment-specific retuning.

## Analysis scripts

- `scripts/analyze_way2_utility.py`
- `scripts/audit_way2_gain.py`
- `scripts/compare_way2_gain_definitions.py`
- `scripts/audit_way2_cost.py`
- `scripts/analyze_way2_lambda.py`
