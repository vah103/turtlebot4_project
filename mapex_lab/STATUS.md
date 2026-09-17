# mapex_lab status

_Last synchronized with `main`: 2026-09-17._

## Current focus

The active thesis direction is **Way2: frontier-utility-based early stopping for MapEx**.

- Way1 is closed.
- Way2 candidate rule is now **FROZEN FOR PROSPECTIVE VALIDATION**.
- Online Way2 integration is implemented in `scripts/mapex_way2.py`; baseline `scripts/mapex.py` remains unchanged.
- Online sanity testing is still pending; no prospective validation run has been collected yet.
- `G` is frozen as raw `information_gain`.
- Historical replay `C` is frozen as Euclidean `distance_m`.
- The 17 existing runs remain development/diagnostic data only and must not be used as independent validation.
- Teacher-facing Google Doc tab **“Triển khai Way2”** is synchronized through the frozen rule, development backtest, and prospective-validation next steps.

Frozen candidate rule for the next validation stage:

```text
For each evaluable decision t over selectable frontiers f:

R_t = max_f information_gain(f) / distance_m(f)
U_t = max_f visible_unknown_cells(f) / distance_m(f)

base_valid_t := (R_t <= 0.30) AND (U_t <= 10.0)

Persistence / confirmation:
- require 2 consecutive evaluable base-valid states;
- at the 2nd state, if selectable candidate_count <= 1: STOP;
- otherwise require a 3rd consecutive base-valid state, then STOP.
```

The rule is frozen as a **candidate validation rule**, not claimed as a validated final method yet.

Development backtest for the frozen candidate:

```text
New Room: trigger 11/15, bad observed-IoU-loss runs 0/11
  median time saved     = 8.21%
  median distance saved = 6.29%
  worst observed IoU loss = 0.001605

Hospital: trigger 2/2, bad observed-IoU-loss runs 0/2
  median time saved     = 3.38%
  median distance saved = 2.09%
  worst observed IoU loss = 0.003713
```

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

# Way2 — CANDIDATE RULE FROZEN FOR VALIDATION

## 1. Offline replay — COMPLETED

Replay baseline:

```text
G = information_gain
C = distance_m
score = G/C
candidate set = selectable == 1
```

Audit over all evaluable historical decisions:

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

Normalized gain changes the best frontier heavily; agreement with raw MapEx is only about 30% of decisions. Raw `G/C` already has compatible post-early/late scale between New Room and Hospital and decreases toward the end of exploration.

Frozen:

```text
G_t(f) = information_gain(f)
```

`visible_unknown_cells` is not used to replace G for frontier ranking. It is used only as a separate completion guard.

## 3. Travel cost C — FROZEN FOR HISTORICAL REPLAY

Cost audit:

| Environment | Euclidean candidate coverage | Planner selected-goal coverage | Planner all-candidate upper bound | Median planner/Euclidean |
|---|---:|---:|---:|---:|
| New Room | 100.0% | 98.1% | 5.5% | 1.413 |
| Hospital | 100.0% | 89.6% | 6.7% | 1.479 |

Frozen historical replay definition:

```text
C_t(f) = distance_m(f)
```

Planner path cannot be used fairly for historical all-candidate replay because old recordings only contain planner paths for selected goals.

## 4. Simple lambda/K rule — REJECTED

Initial sensitivity explored lambda from 0.05 to 2.0 and persistence guards K=2/3/4 plus trailing-mean smoothing.

Important negative result:

> **R_t = max(G/C) with only one fixed lambda and a fixed K-consecutive debounce is not a sufficient cross-environment completion criterion.**

Reasons:

- high lambda increases premature-stop/rebound risk;
- low lambda becomes too conservative and often does not trigger;
- increasing K removes some New Room rebounds but does not solve Hospital cleanly;
- smoothing reduces trigger coverage and still does not solve the key Hospital failure.

## 5. Map-quality audit — CRITICAL RESULT

Hospital exposed a failure that rebound analysis alone missed.

`hpx_001` has low `R_t` around decisions 43–45 but the observed SLAM map still gains important occupied structure later.

Observed-only examples:

```text
lambda=0.4, K=2: stop d44, observed IoU 0.366621 -> final 0.398084, loss 0.031463
lambda=0.4, K=3: stop d45, observed IoU 0.369641 -> final 0.398084, loss 0.028442
lambda=0.5, K=2: stop d43, observed IoU 0.366833 -> final 0.398084, loss 0.031251
```

Therefore the apparent failure is real structural loss, not merely prediction-reference instability.

## 6. Secondary completion guard — COMPLETED

Audited online-available signals over selectable frontiers:

```text
Umax_t  = max visible_unknown_cells
Udmax_t = max visible_unknown_cells / distance_m
```

`Udmax_t` was more useful cross-environment than raw `Umax_t`.

The completion condition was expanded to:

```text
R_t <= lambda
AND
Udmax_t <= threshold
```

This keeps the original MapEx frontier ranking unchanged; `Udmax_t` is only a stopping safety guard.

## 7. Remaining New Room failure and adaptive confirmation

With `lambda=0.3/0.4`, `Udmax<=10`, fixed K=2:

- Hospital became safe 2/2;
- New Room had one near-threshold failure: `mpx_009`, stop d26, observed IoU loss 0.010516.

Focused trajectory:

```text
mpx_009 d26: observed loss = 0.010516
mpx_009 d27: observed loss = -0.000272
```

One additional confirming decision removes the failure.

A blanket fixed K=3 is too conservative because it loses Hospital trigger coverage. This motivated the adaptive persistence rule based on the number of remaining selectable frontiers.

## 8. Adaptive candidate-count confirmation — COMPLETED

Rule tested after two consecutive base-valid states:

```text
if candidate_count <= cutoff:
    STOP
else:
    require one additional base-valid decision
```

Sensitivity:

- cutoff=0 behaves like fixed K=3: safe but Hospital trigger falls to 1/2.
- cutoff=1: New Room 11/15, Hospital 2/2, zero observed-IoU violations >0.01.
- cutoff>=2 immediately restores the `mpx_009` d26 failure.

`cutoff=1` also has a direct structural interpretation: only one selectable frontier remains, whereas >=2 means multiple exploration alternatives still remain.

Frozen candidate cutoff:

```text
candidate_count <= 1
```

## 9. Local robustness sweep — COMPLETED

Adaptive rule neighborhood tested:

```text
lambda = 0.25, 0.30, 0.35, 0.40
Udmax threshold = 7.5, 10.0, 12.5, 15.0
candidate_count cutoff = 1
```

Key result: zero-bad cross-environment behavior is not isolated to one exact parameter pair.

A robust plateau exists around:

```text
lambda = 0.25 ... 0.40
Udmax threshold = 10.0 ... 12.5
```

Examples with Hospital 2/2 and zero observed-IoU violations:

```text
lambda=0.25, Udmax<=10.0: NR 11/15, H 2/2
lambda=0.30, Udmax<=10.0: NR 11/15, H 2/2
lambda=0.35, Udmax<=10.0: NR 11/15, H 2/2
lambda=0.40, Udmax<=10.0: NR 11/15, H 2/2

lambda=0.25, Udmax<=12.5: NR 12/15, H 2/2
lambda=0.30, Udmax<=12.5: NR 12/15, H 2/2
lambda=0.35, Udmax<=12.5: NR 13/15, H 2/2
lambda=0.40, Udmax<=12.5: NR 13/15, H 2/2
```

Boundary behavior is also coherent:

- `Udmax<=7.5` is too conservative for Hospital (`1/2` trigger).
- `Udmax<=15` allows the `hpx_001` premature stop back in (`observed IoU loss 0.015491`).

Therefore the safe behavior is a local region rather than a single lucky grid point.

## 10. Frozen candidate for prospective validation

Chosen candidate:

```text
lambda = 0.30
Udmax threshold = 10.0
candidate_count cutoff = 1
```

Rationale:

- lies inside the locally robust zero-bad plateau rather than on its aggressive edge;
- more conservative than `lambda=0.4` without losing development trigger coverage at threshold 10;
- threshold 10 has margin from the unsafe threshold 15 boundary;
- zero observed-IoU violations >0.01 on all triggered development runs;
- preserves Hospital 2/2 development trigger coverage;
- does not maximize savings on the development set, reducing parameter-selection pressure toward overfit.

Full stop logic:

```text
base_valid_t =
    (max information_gain/distance_m <= 0.30)
    AND
    (max visible_unknown_cells/distance_m <= 10.0)

After 2 consecutive evaluable base_valid states:
    if selectable candidate_count <= 1:
        STOP
    else:
        require a 3rd consecutive evaluable base_valid state, then STOP
```

This rule is now frozen. Do not retune it using the 17 development runs after prospective validation begins.

## 11. Online integration — IMPLEMENTED, SANITY TEST PENDING

Implementation:

- baseline `scripts/mapex.py` is preserved unchanged;
- new variant `scripts/mapex_way2.py` inherits `MapExExplorer`;
- Way2 is evaluated only after the same MapEx distance preference / near fallback and shared execution/planner suppression have produced the selectable frontier set;
- `R_t`, `U_t`, selectable candidate count, `base_valid`, consecutive valid count, and the Way2 action are logged/published online;
- Way2 does not alter MapEx frontier ranking when the decision is CONTINUE;
- early stop publishes `/frontier_exploration_complete = true` and status reason `way2_early_stop` without issuing another Nav2 goal;
- recovery retries, active-goal map updates, no-frontier terminal handling, and planner revalidation are not counted as Way2 evaluable decisions.

The integration is implementation-complete but has not yet been sanity-tested on the robot. Sanity-test runs may be used only to fix implementation bugs, not to retune the frozen thresholds.

## 12. Evaluation caveats

- The 17 existing runs are development/tuning/audit data only.
- Observed-only occupied IoU is used for quality auditing and is not an online stop input.
- `iou_loss_vs_final` from `early_stopping_analysis.csv` uses prediction-assisted reconstructed maps; observed-only audits were added specifically to check real structural loss.
- TU remains weakly discriminative and is not a primary stop signal.

---

# Current next actions

1. Sanity-test `scripts/mapex_way2.py` online and verify `R_t`, `U_t`, selectable candidate count, `base_valid`, persistence state, CONTINUE, and EARLY STOP logs.
2. Fix implementation bugs only if the sanity test exposes them; do **not** retune `0.30`, `10.0`, cutoff `1`, or the 2/3-decision confirmation rule.
3. Collect **new independent** New Room + Hospital validation runs with the rule frozen and no environment-specific retuning.
4. Compare against the full MapEx baseline using time, distance, observed structural quality, reconstructed quality, and existing exploration metrics.
5. Only after prospective validation decide whether Way2 is accepted or rejected as the thesis stopping rule.

## Analysis scripts

- `scripts/analyze_way2_utility.py`
- `scripts/audit_way2_gain.py`
- `scripts/compare_way2_gain_definitions.py`
- `scripts/audit_way2_cost.py`
- `scripts/analyze_way2_lambda.py`
- `scripts/inspect_way2_lambda_focus.py`
- `scripts/inspect_way2_rebounds.py`
- `scripts/compare_way2_guard_k.py`
- `scripts/compare_way2_guard_smoothing.py`
- `scripts/evaluate_way2_candidate_quality.py`
- `scripts/inspect_way2_quality_failures.py`
- `scripts/inspect_way2_hospital_quality_trajectory.py`
- `scripts/audit_way2_observed_quality.py`
- `scripts/sweep_way2_lambda_quality.py`
- `scripts/audit_way2_visible_unknown_guard.py`
- `scripts/sweep_way2_visible_unknown_guard.py`
- `scripts/inspect_way2_guard_candidates.py`
- `scripts/inspect_way2_mpx009_quality_trajectory.py`
- `scripts/evaluate_way2_adaptive_confirmation.py`
- `scripts/sweep_way2_adaptive_candidate_count.py`
- `scripts/sweep_way2_adaptive_rule_neighborhood.py`
