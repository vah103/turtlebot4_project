# mapex_lab status

_Last synchronized with `main`: 2026-09-17._

## Current focus

The active thesis direction is **Way2: frontier-utility-based early stopping for MapEx**.

- Way1 is closed.
- Way2 is still offline/development only; online MapEx has not yet been changed to stop early.
- `G` is frozen as raw `information_gain`.
- Historical replay `C` is frozen as Euclidean `distance_m`.
- `lambda` and the final persistence/completion guard are **not yet frozen**.
- The simple rule `R_t <= lambda` plus only a K-consecutive-state debounce is currently **not sufficient** as a cross-environment completion criterion.

Core utility signal:

```text
V_t(f) = G_t(f) - lambda * C_t(f)
R_t    = max_f G_t(f)/C_t(f)
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

Do not divide by `visible_unknown_cells` for frontier ranking.
Do not use startup maxima to choose `lambda`.

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

## 4. Lambda sensitivity — COMPLETED, NOT FROZEN

Initial K=2 sensitivity grid:

```text
lambda = 0.1, 0.2, 0.3, 0.4, 0.5, 0.75, 1.0, 1.5, 2.0
```

Main result:

- very low lambda is conservative and often fails to trigger in New Room.
- high lambda gives larger savings but more rebound/premature-stop risk.
- `lambda=0.4–0.5` initially looked like the useful middle region by savings/rebound alone.

However later map-quality audits show that savings/rebound alone are insufficient for selecting lambda.

## 5. Rebound and persistence-guard audit — COMPLETED

At `lambda=0.4` and `0.5`, K=2 had immediate/near-immediate rebounds in:

- `mpx_004`
- `mpx_012`
- `hpx_002`

The rebounds occur within 1–2 later evaluable decisions.

K comparison:

- `K=3` removes New Room rebounds but `hpx_002` still rebounds.
- `K=4` removes observed rebounds but trigger coverage collapses strongly.
- trailing-mean smoothing windows 3/4 do not solve `hpx_002` and reduce coverage.

Conclusion: simply increasing K or smoothing R_t is not a satisfactory general fix.

## 6. Map-quality audit — CRITICAL RESULT

Candidate rules tested:

```text
lambda = 0.4 or 0.5
K = 2 or 3
```

New Room is comparatively well behaved, especially with K=3.

Hospital reveals a different failure mode:

- the run causing large IoU loss is **hpx_001**, not the rebound run `hpx_002`.
- at candidate stops around decisions 43–45, `hpx_001` has coverage about **99.52–99.53%**, but observed structural quality still improves materially later.
- observed-only IoU confirms the loss is real, not just instability in prediction-assisted reconstructed IoU.

Observed-only `hpx_001` examples:

```text
lambda=0.4, K=2: stop d44, observed IoU 0.366621 -> final 0.398084, loss 0.031463
lambda=0.4, K=3: stop d45, observed IoU 0.369641 -> final 0.398084, loss 0.028442
lambda=0.5, K=2: stop d43, observed IoU 0.366833 -> final 0.398084, loss 0.031251
lambda=0.5, K=3: stop d44, observed IoU 0.366621 -> final 0.398084, loss 0.031463
```

The observed map continues to gain important occupied structure even while R_t is already low.

## 7. Low-lambda quality sweep — COMPLETED

Tested:

```text
lambda = 0.05, 0.075, 0.1, 0.15, 0.2, 0.25, 0.3, 0.35, 0.4
K = 2, 3
```

Goal: find one cross-environment rule with useful trigger/savings and observed-only IoU loss <= 0.01.

No simple lambda/K rule met that goal on the current development data.

Key boundary cases:

- `lambda=0.075, K=2` is quality-safe in triggered runs, but triggers only **3/15 New Room** and **1/2 Hospital**; `hpx_001` stops at d51 with essentially no distance saving.
- `lambda=0.1, K=2` triggers **4/15 New Room** and **2/2 Hospital**, but `hpx_001` still has observed-IoU loss **0.015608**.
- larger lambda values trigger more consistently but make `hpx_001` structural loss worse, reaching roughly **0.028–0.031** in the 0.2–0.4 region.

Conclusion:

> **The current one-step uncertainty-weighted G/C signal plus a fixed lambda/K guard is not yet a sufficient cross-environment completion criterion.**

This is a useful negative result. Do not freeze lambda/K from the current sweep.

## 8. Current hypothesis: secondary completion guard — UNDER AUDIT

Potential issue:

`information_gain` is uncertainty-weighted. A frontier may have low uncertainty-weighted gain while still exposing a meaningful amount of currently unknown map structure.

Therefore the next audit keeps the frozen ranking semantics but tests a separate, online-available completion signal based on recorded:

```text
visible_unknown_cells
```

Candidate secondary signals:

```text
Umax_t  = max selectable visible_unknown_cells
Udmax_t = max selectable visible_unknown_cells / distance_m
```

These are being evaluated as a possible **stop guard**, not as a replacement for G used in frontier ranking.

No threshold is frozen yet.

## 9. Evaluation caveats

- `iou_loss_vs_final` from `early_stopping_analysis.csv` uses prediction-assisted reconstructed maps.
- Observed-only IoU audits were added to verify whether losses correspond to actual SLAM structural observations.
- `hpx_001` remains a genuine quality failure under observed-only IoU, so its issue is not explained away by prediction-reference instability.
- TU remains weakly discriminative and should not be the primary stopping signal.

---

# Current next actions

1. Run `scripts/audit_way2_visible_unknown_guard.py` on all 17 development runs.
2. Check whether unsafe stops consistently retain larger `visible_unknown_cells` / `visible_unknown_cells per meter` than safe stops.
3. If there is a stable separation, formulate a secondary online completion guard without changing the existing MapEx frontier ranking.
4. If there is no separation, reconsider the Way2 gain/completion formulation rather than continuing to tune lambda/K.
5. Freeze the full rule only after the formulation is stable.
6. Integrate online only after freeze.
7. Collect new independent New Room + Hospital validation runs with no environment-specific retuning.

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
