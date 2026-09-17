# Way2 threshold selection evidence

_Last updated: 2026-09-17._

This document is the decision trail for the **frozen Way2 early-stopping rule**. It separates development/tuning evidence from post-freeze online runs so that validation data is not used to retune the rule.

## 1. Final frozen rule

For each evaluable decision over the **selectable** MapEx frontier set `F_t`:

```text
R_t = max_f information_gain(f) / distance_m(f)
U_t = max_f visible_unknown_cells(f) / distance_m(f)

base_valid_t := (R_t <= 0.30) AND (U_t <= 10.0)

Persistence / confirmation:
- first consecutive base-valid evaluable decision: CONTINUE;
- second consecutive base-valid evaluable decision:
  - if selectable candidate_count <= 1: STOP;
  - otherwise CONTINUE and require one more confirmation;
- third consecutive base-valid evaluable decision: STOP.
```

Frozen constants:

```text
R threshold                    = 0.30
U threshold                    = 10.0 cells/m
candidate-count cutoff         = 1
second confirmation            = 2
third confirmation             = 3
```

The rule is frozen. Post-freeze online runs must not be used to retune these constants.

---

## 2. Development data and separation from validation

The threshold-selection work used the historical development/diagnostic set:

```text
New Room: 15 runs
Hospital: 2 runs
Total:    17 runs
```

These 17 runs are development/tuning/audit data only.

Post-freeze online Way2 runs are tracked separately in `way2_online_runs.csv` and are not inputs to threshold selection.

---

## 3. Replay validity: reproduce the original MapEx ranking first

Historical replay definition:

```text
G = information_gain
C = distance_m
score = G/C
candidate set = selectable == 1
```

Audit result over evaluable historical decisions:

```text
score_mismatch     = 0
selection_mismatch = 0
```

Therefore the offline Way2 replay reproduces the current MapEx frontier ranking exactly before any stopping logic is added.

Relevant scripts:

- `scripts/analyze_way2_utility.py`
- `scripts/audit_way2_gain.py`
- `scripts/audit_way2_cost.py`

---

## 4. Gain definition G

Definitions compared:

```text
G_raw  = information_gain
G_norm = information_gain / visible_unknown_cells
```

Observed result:

- normalized gain changes the best frontier heavily;
- agreement with raw MapEx is only about **30% of decisions**;
- raw `G/C` has compatible post-early/late scale across New Room and Hospital;
- raw `G/C` decreases toward the end of exploration.

Decision:

```text
G_t(f) = information_gain(f)
```

`visible_unknown_cells` is retained only as a secondary stopping-safety guard, not as a replacement gain for frontier ranking.

Relevant scripts:

- `scripts/audit_way2_gain.py`
- `scripts/compare_way2_gain_definitions.py`

---

## 5. Travel-cost definition C

Historical candidate coverage audit:

| Environment | Euclidean candidate coverage | Planner selected-goal coverage | Planner all-candidate upper bound | Median planner/Euclidean |
|---|---:|---:|---:|---:|
| New Room | 100.0% | 98.1% | 5.5% | 1.413 |
| Hospital | 100.0% | 89.6% | 6.7% | 1.479 |

Historical recordings do not contain planner paths for all candidates, therefore planner cost cannot be reconstructed fairly for every historical frontier.

Decision for historical replay:

```text
C_t(f) = distance_m(f)
```

Relevant script:

- `scripts/audit_way2_cost.py`

---

## 6. Simple lambda/K rule: rejected

Initial sensitivity explored:

```text
lambda: 0.05 ... 2.0
K:      2, 3, 4
plus trailing-mean smoothing variants
```

Result:

- high lambda increases premature-stop/rebound risk;
- low lambda becomes too conservative and frequently does not trigger;
- increasing fixed `K` removes some New Room rebounds but does not solve Hospital cleanly;
- smoothing reduces trigger coverage and does not solve the critical Hospital quality failure.

Conclusion:

> `R_t = max(G/C)` with only a fixed lambda and fixed K-consecutive debounce is not a sufficient cross-environment completion criterion.

Relevant scripts:

- `scripts/analyze_way2_lambda.py`
- `scripts/inspect_way2_lambda_focus.py`
- `scripts/inspect_way2_rebounds.py`
- `scripts/compare_way2_guard_k.py`
- `scripts/compare_way2_guard_smoothing.py`

---

## 7. Structural-quality failure exposed by Hospital

Hospital showed that rebound checks alone can miss real map loss.

For `hpx_001`, low `R_t` around decisions 43--45 occurs while the observed SLAM map still gains important occupied structure later.

Observed-only examples:

| Rule | Stop decision | Observed IoU at stop | Final observed IoU | Observed IoU loss |
|---|---:|---:|---:|---:|
| lambda=0.4, K=2 | 44 | 0.366621 | 0.398084 | 0.031463 |
| lambda=0.4, K=3 | 45 | 0.369641 | 0.398084 | 0.028442 |
| lambda=0.5, K=2 | 43 | 0.366833 | 0.398084 | 0.031251 |

Conclusion: the failure is real structural loss, not merely instability of a prediction-assisted reference.

Relevant scripts:

- `scripts/evaluate_way2_candidate_quality.py`
- `scripts/inspect_way2_quality_failures.py`
- `scripts/inspect_way2_hospital_quality_trajectory.py`
- `scripts/audit_way2_observed_quality.py`

---

## 8. Secondary guard based on remaining visible unknown space

Online-available candidate-level signals audited:

```text
Umax_t  = max visible_unknown_cells
Udmax_t = max visible_unknown_cells / distance_m
```

`Udmax_t` transferred better across New Room and Hospital than raw `Umax_t`.

Expanded base condition:

```text
R_t <= lambda
AND
Udmax_t <= threshold
```

This guard does not alter MapEx frontier ranking; it is only a completion-safety condition.

Relevant scripts:

- `scripts/audit_way2_visible_unknown_guard.py`
- `scripts/sweep_way2_visible_unknown_guard.py`

---

## 9. Fixed K=2 after adding the U guard: one New Room failure remained

With:

```text
lambda = 0.3 or 0.4
Udmax <= 10
fixed K = 2
```

Hospital became safe on both development runs, but New Room retained one near-threshold failure:

```text
mpx_009 stop d26: observed IoU loss = 0.010516
mpx_009 d27:      observed IoU loss = -0.000272
```

One additional confirming decision removes that failure.

A blanket fixed `K=3` is too conservative because it loses Hospital trigger coverage, motivating adaptive confirmation based on the number of remaining selectable candidates.

Relevant scripts:

- `scripts/inspect_way2_guard_candidates.py`
- `scripts/inspect_way2_mpx009_quality_trajectory.py`

---

## 10. Adaptive candidate-count confirmation

Rule tested after two consecutive base-valid states:

```text
if candidate_count <= cutoff:
    STOP
else:
    require one additional base-valid decision
```

Sensitivity result:

| cutoff | New Room | Hospital | Quality result |
|---:|---:|---:|---|
| 0 | behaves like fixed K=3 | 1/2 trigger | safe but too conservative |
| 1 | 11/15 trigger | 2/2 trigger | zero observed-IoU violations > 0.01 |
| >=2 | `mpx_009` d26 failure returns | — | observed IoU loss 0.010516 |

Decision:

```text
candidate_count cutoff = 1
```

Interpretation: when only one selectable frontier remains after the second confirmation, the remaining exploration choice is structurally weak enough to permit early stopping. With two or more alternatives, one more confirmation is required.

Relevant scripts:

- `scripts/evaluate_way2_adaptive_confirmation.py`
- `scripts/sweep_way2_adaptive_candidate_count.py`

---

## 11. Local robustness sweep around the candidate rule

Neighborhood tested:

```text
lambda              = 0.25, 0.30, 0.35, 0.40
Udmax threshold      = 7.5, 10.0, 12.5, 15.0
candidate cutoff     = 1
```

Verified safe plateau examples:

| lambda | Udmax threshold | New Room trigger | Hospital trigger | observed-IoU violations >0.01 |
|---:|---:|---:|---:|---:|
| 0.25 | 10.0 | 11/15 | 2/2 | 0 |
| 0.30 | 10.0 | 11/15 | 2/2 | 0 |
| 0.35 | 10.0 | 11/15 | 2/2 | 0 |
| 0.40 | 10.0 | 11/15 | 2/2 | 0 |
| 0.25 | 12.5 | 12/15 | 2/2 | 0 |
| 0.30 | 12.5 | 12/15 | 2/2 | 0 |
| 0.35 | 12.5 | 13/15 | 2/2 | 0 |
| 0.40 | 12.5 | 13/15 | 2/2 | 0 |

Boundary behavior:

```text
Udmax <= 7.5  -> too conservative for Hospital: 1/2 trigger
Udmax <= 15.0 -> hpx_001 premature stop returns: observed IoU loss = 0.015491
```

Therefore the safe behavior is a local region rather than one isolated parameter pair.

Machine-readable rows currently preserved from this verified neighborhood are in `way2_threshold_neighborhood.csv`.

Relevant script:

- `scripts/sweep_way2_adaptive_rule_neighborhood.py`

---

## 12. Why the frozen point is 0.30 / 10.0 / cutoff 1

Chosen candidate:

```text
lambda = 0.30
Udmax threshold = 10.0
candidate_count cutoff = 1
```

Reasons:

1. It lies inside the zero-bad robustness plateau rather than on an aggressive boundary.
2. `lambda=0.30` is more conservative than 0.40 without losing development trigger coverage at `Udmax=10`.
3. `Udmax=10` has margin from the unsafe `Udmax=15` boundary.
4. Triggered development runs have zero observed-IoU violations above 0.01.
5. Hospital retains 2/2 development trigger coverage.
6. The point does not maximize development-set savings, reducing pressure toward overfitting.

Development backtest at the frozen candidate:

| Environment | Trigger | Bad observed-IoU-loss runs | Median time saved | Median distance saved | Worst observed IoU loss |
|---|---:|---:|---:|---:|---:|
| New Room | 11/15 | 0/11 | 8.21% | 6.29% | 0.001605 |
| Hospital | 2/2 | 0/2 | 3.38% | 2.09% | 0.003713 |

This is development evidence, not an independent validation result.

---

## 13. Online implementation and freeze boundary

Frozen online implementation:

- `scripts/mapex_way2.py`
- `scripts/mapex_way2_run.py`

Important implementation properties:

- baseline `scripts/mapex.py` remains unchanged;
- Way2 is evaluated only after MapEx distance preference / near fallback and execution/planner suppression produce the selectable set;
- non-evaluable states do not count toward or reset consecutive Way2 confirmation unless the implementation explicitly does so;
- a CONTINUE decision uses unchanged MapEx ranking;
- an early stop emits `way2_early_stop` and issues no next Nav2 exploration goal.

Recorder evolution:

- `mpx_w2_001` exposed a recorder-only termination-label bug; the algorithm stopped correctly but its summary originally reported `exploration_complete`;
- the bug was fixed before later prospective collection;
- the recorder now buffers per-evaluable-decision Way2 audit states and writes `way2_checks.csv` at finalization;
- the added audit recording does not change the frozen policy thresholds.

---

## 14. Post-freeze online runs: keep separate from threshold selection

Current pushed Way2 runs are summarized in `way2_online_runs.csv`.

At the time of this update:

```text
New Room:
- mpx_w2_001  integration/sanity; actual Way2 early-stop branch exercised
- mpx_w2_002  normal baseline completion after no selectable frontier remained

Hospital:
- hpx_w2_001  normal completion
- hpx_w2_002  normal completion
- hpx_w2_003  Way2 early stop, third consecutive valid confirmation
```

These runs are evidence about online behavior and future validation performance. They are **not** allowed to change the frozen thresholds above.

---

## 15. Reproducibility / scripts used in the threshold-selection chain

Main analysis scripts currently committed:

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

## 16. Data-retention note

This file records all threshold-selection numerical conclusions currently verified and preserved in the repository/status trail.

Some analysis scripts historically printed larger intermediate sweep tables without every raw row being versioned as an output file. To make the record literally exhaustive, those scripts should be rerun against the preserved development runs and their generated CSV/JSON outputs committed under `mapex_lab/analysis/way2_results/`. New threshold tuning is not permitted during that regeneration; it is only archival/reproducibility work.
