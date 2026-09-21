# MapEx Early-Stopping Program — Implementation Specification

> **Status:** active research program after closing Way1 and Way2.
>
> **Purpose:** this file is the canonical implementation specification for the six new MapEx early-stopping directions. It is deliberately stricter than a brainstorming note. A future implementation must follow the definitions here unless an explicit experiment changes one definition and records that change.
>
> Related literature review: `STOPPING_CRITERIA_RESEARCH.md`.
>
> These are candidate research directions, not novelty claims. Novelty must still be checked against the literature before writing a thesis/paper contribution statement.

---

# 0. Historical boundary: Way1 and Way2 are closed

Way1 and Way2 are **finished historical branches**. Their code, runs and logs stay in the repository as evidence, but they are no longer the active thesis direction and must not be silently retuned into one of the six directions below.

## Way1 — closed

Historical rule:

```text
unknown_variance_p95 <= 0.23
```

Reason for closing: it did not transfer robustly from New Room to Hospital without retuning.

**Direction 1 below is not Way1.** Direction 1 does not stop from global unknown-map variance alone. It first estimates the amount of **predicted reachable remaining free space**, then uses uncertainty only as a safety gate over that remaining region.

## Way2 — closed

Historical rule used frontier-level quantities including:

```text
R_t = max information_gain / distance
U_t = max visible_unknown_cells / distance
+ confirmation logic
```

Reason for closing as the main research branch: the collected outcome was not sufficiently robust/compelling to continue as the primary thesis direction.

**Direction 2 below is not Way2.** Direction 2 is intentionally a simple saturation baseline. It must not inherit Way2 thresholds, candidate-count logic or frozen constants.

## Naming rule

From this point onward use:

```text
D1 ... D6
```

for the six new directions. Do not reuse "Way1" or "Way2" for new implementations.

---

# 1. Shared research objective

The common objective is:

> **Stop before exhaustive exploration when continuing is unlikely to add enough useful map information, while keeping the loss in final map quality within a pre-declared acceptable bound.**

The early-stopping layer decides **whether exploration should continue**. It must not change the original MapEx frontier ranking or Nav2 execution unless a separate experiment explicitly tests such a change.

A successful method is therefore not merely one that stops early. It must show meaningful time/distance saving while satisfying a map-quality constraint.

---

# 2. Verified MapEx runtime contract

The current MapEx implementation in `scripts/mapex.py` already provides the required core signals:

```text
Observed ROS occupancy grid
→ 3 LaMa ensemble predictions
→ ensemble mean map
→ ensemble variance map
→ MapEx frontier extraction
→ probabilistic visibility
→ information_gain(f)
→ distance_m(f)
→ score(f) = information_gain(f) / distance_m(f)
→ selectable frontier filtering
→ choose argmax(score)
→ send Nav2 goal
```

Current fixed runtime facts:

```text
ensemble size          N = 3
runtime map resolution   = 0.10 m/cell
prediction resolution    = 0.10 m/cell
observed free            = 0
observed unknown         = 0.5 in MapEx representation
observed occupied        = 1
frontier connectivity    = 8-neighbour
MapEx score              = IG / Euclidean distance
```

The new stopping code must reuse the current `predictions`, `mean_map`, `variance_map`, `padded_observed`, frontier evaluations and selection filters rather than recomputing a parallel version with different semantics.

---

# 3. One exact stopping insertion point

For **all six directions**, an early-stop decision is evaluated only at an **evaluable MapEx decision**:

1. there is a current map;
2. there is no active navigation goal;
3. the decision is not a recovery retry;
4. LaMa prediction and MapEx scoring completed successfully;
5. a non-empty `selectable_evaluations` set remains after:
   - frontier extraction,
   - 1 m distance preference / all-near fallback,
   - planner-blocking suppression;
6. the stop rule is evaluated **after those steps and before selecting/sending the next Nav2 goal**.

Therefore:

```text
prediction + scoring
→ execution/selectability filtering
→ if selectable_evaluations is empty:
     DO NOT call an early-stop rule;
     let baseline revalidation/completion logic handle the state
→ else:
     evaluate D1...D6
     ├── CONTINUE → unchanged MapEx argmax(score) → Nav2
     └── STOP     → no new exploration goal is sent
```

A non-evaluable state must **neither increment nor reset** a K-consecutive confirmation counter.

This keeps early stopping counterfactual and interpretable: the rule only stops when baseline MapEx would otherwise have had a valid next exploration action.

---

# 4. Shared data definitions

## 4.1 Runtime-sized prediction arrays

LaMa predictions are padded. Every global stopping metric must first crop each ensemble member back to the exact current runtime occupancy-grid shape.

For runtime grid height `H`, width `W`:

```text
P_j_runtime =
    P_j[pad_top : pad_top + H,
        pad_left: pad_left + W]
```

Do the same crop for `mean_map` and `variance_map` when a stopping metric is defined in runtime-map coordinates.

**Never count padded cells as remaining environment.**

## 4.2 Observed masks

From the current ROS occupancy grid:

```text
observed_free     = grid == 0
observed_unknown  = grid < 0
observed_occupied = grid > 0
```

Cell area:

```text
cell_area_m2 = resolution_m * resolution_m
             = 0.10 * 0.10
             = 0.01 m2 under hospital_v2
```

## 4.3 Predicted-free threshold

For the six-direction program, the primary binary interpretation of a LaMa member is fixed as:

```text
predicted_free_j(x) := P_j_runtime(x) < 0.5
```

Exactly `0.5` is **not** treated as predicted free.

This threshold is part of the method definition. If another threshold is tested, it is an explicit ablation and must not silently replace the primary definition.

## 4.4 Reachable predicted free space

For ensemble member `j`, build:

```text
navigable_j =
    observed_free
    OR
    (observed_unknown AND predicted_free_j)
```

Observed occupied cells are always blocked, regardless of prediction.

Find the **8-connected component** of `navigable_j` containing the robot's current runtime-grid cell.

If the robot cell is outside the map or is not in `observed_free`, the stopping decision is non-evaluable and MapEx continues normally. Do not invent a nearest seed inside stopping code.

Define the predicted remaining region:

```text
R_j = connected_component_8(navigable_j, robot_cell)
      AND observed_unknown
```

This is the canonical meaning of **predicted reachable remaining area** in D1, D3, D4 and D5.

It deliberately does **not** mean:
- all unknown cells;
- all predicted-free cells anywhere in the canvas;
- sum of per-frontier visibility with overlap counted repeatedly.

## 4.5 Remaining area per ensemble member

```text
A_j_m2 = count(R_j) * cell_area_m2
```

Ensemble mean remaining area:

```text
A_mean_m2 = (A_1_m2 + A_2_m2 + A_3_m2) / 3
```

Remaining-region support:

```text
R_union = R_1 OR R_2 OR R_3
```

## 4.6 Primary uncertainty definition

Reuse the existing MapEx `variance_map`; do not recompute a different ensemble variance.

The **primary D1 uncertainty metric** is:

```text
U_p95 = percentile_95(
    variance_map_runtime[x]
    for x in R_union
)
```

If `R_union` is empty:

```text
U_p95 = 0.0
```

Mean variance, max variance, entropy or binary disagreement may be tested later as ablations, but they are **not** the primary D1 definition.

Important safety rule:

> Uncertainty is a separate gate. High uncertainty must never reduce a score in a way that makes STOP easier.

---

# 5. Shared state machine and parameter discipline

## 5.1 No one-decision stopping

Every rule must be true for `K_confirm` consecutive **evaluable** decisions before STOP.

Pseudo-code:

```python
if not evaluable:
    # preserve counter
    return CONTINUE_BASELINE_HANDLING

if stop_condition:
    valid_count += 1
else:
    valid_count = 0

if valid_count >= K_confirm:
    return STOP
return CONTINUE
```

## 5.2 Warm-up

All directions use exactly one warm-up mechanism:

```text
mapex_decision_id < warmup_decisions
→ early stopping disabled
```

Do not mix time-, distance- and coverage-based warm-up in the main implementation.

The numerical value of `warmup_decisions` is a tunable development parameter that must be frozen before prospective validation.

## 5.3 Parameters are not results

Thresholds such as:

```text
T_area_m2
T_uncertainty
T_ig
T_score
T_progress_m2_per_m
T_brier
A_critical_m2
K_confirm
warmup_decisions
```

must be selected on development data only, then frozen.

Do not retune them from the prospective validation runs.

## 5.4 Termination reason

Record explicit reasons:

```text
STOP_D1_COMPLETENESS
STOP_D2_IG_SATURATION
STOP_D3_PREDICTION_STAGNATION
STOP_D4_RELIABILITY_GATED
STOP_D5_LOW_MISS_RISK
STOP_D6_LEARNED
STOP_NO_FRONTIER
STOP_BASELINE_REVALIDATION
STOP_TIMEOUT
STOP_MAX_DISTANCE
```

---

# 6. Direction 1 — Uncertainty-Aware Predicted Map Completeness

## Question

Does MapEx already predict that only a small amount of reachable environment remains, and is it sufficiently certain about that conclusion?

## Required inputs

- `P_1_runtime, P_2_runtime, P_3_runtime`
- `variance_map_runtime`
- current ROS occupancy grid
- robot runtime-grid cell

## Derived inputs

Use the shared definitions exactly:

```text
R_1, R_2, R_3
A_1_m2, A_2_m2, A_3_m2
A_mean_m2
R_union
U_p95
```

## Primary stop condition

```text
D1_base_valid :=
    (A_mean_m2 < T_area_m2)
    AND
    (U_p95 < T_uncertainty)
```

STOP only after `K_confirm` consecutive evaluable D1-valid decisions.

## Meaning

The robot stops only when:

> the ensemble predicts little reachable unknown free space remains **and** the relevant predicted remaining region is not highly uncertain.

## Explicit non-goals

D1 must not:
- stop from global unknown-cell count;
- stop from global variance alone;
- multiply remaining area by confidence and threshold the product;
- use ground truth online;
- change MapEx frontier ranking.

## Required ablation

At minimum compare:

```text
MapEx baseline
D1-area-only: A_mean_m2 < T_area_m2
D1-full:      A_mean_m2 < T_area_m2 AND U_p95 < T_uncertainty
```

This isolates the value of the uncertainty gate.

---

# 7. Direction 2 — Information-Gain Saturation

## Question

Even though a valid next frontier exists, has the expected information value of the best remaining frontier become too small?

## Required inputs

From current `selectable_evaluations`:

```text
IG_max    = max(candidate["information_gain"])
Score_max = max(candidate["score"])
```

## Primary stop condition

The primary D2 baseline uses raw MapEx IG:

```text
D2_base_valid := IG_max < T_ig
```

STOP after `K_confirm` consecutive evaluable decisions.

## Cost-aware ablation

A separate ablation may use:

```text
Score_max < T_score
```

Do not mix both thresholds in the primary D2 rule.

## Meaning

D2 asks:

> If even the best valid frontier has little remaining information, is further exploration still worthwhile?

## Relation to old Way2

D2 is a **new simple baseline**, not old Way2.

Do not import:
- Way2 threshold `R<=0.30`;
- `visible_unknown_cells / distance`;
- candidate-count confirmation;
- any Way2 frozen constants.

---

# 8. Direction 3 — Prediction + Stagnation Hybrid

## Question

Can false early stops be reduced by requiring both:
1. D1 says little predicted environment remains; and
2. recent real exploration is producing very little new known area per metre travelled?

## Required inputs

- D1 quantities `A_mean_m2` and `U_p95`
- absolute known area
- cumulative executed trajectory distance
- history over `W_progress` evaluable decisions

## Known area

```text
KnownArea_t_m2 =
    count(grid >= 0) * cell_area_m2
```

## Primary progress metric

Let `t0` be the evaluable decision `W_progress` decisions earlier:

```text
delta_known_m2 =
    KnownArea_t_m2 - KnownArea_t0_m2

delta_distance_m =
    trajectory_distance_t - trajectory_distance_t0

Progress_m2_per_m =
    delta_known_m2 / max(delta_distance_m, 1e-6)
```

D3 progress is evaluable only when:

```text
delta_distance_m >= D_window_min_m
```

If this is not satisfied, D3 cannot stop.

This avoids treating a stationary/recovery period as mapping stagnation.

## Primary stop condition

```text
D3_base_valid :=
    (A_mean_m2 < T_area_m2)
    AND
    (U_p95 < T_uncertainty)
    AND
    (Progress_m2_per_m < T_progress_m2_per_m)
```

STOP after `K_confirm` consecutive evaluable D3-valid decisions.

## Optional ablation only

Frontier discovery rate may be logged and later tested, but it is **not** part of the primary D3 rule.

---

# 9. Direction 4 — Prediction Reliability-Gated Stopping

## Question

Before trusting D1, have recent MapEx predictions actually matched cells that the robot later observed?

## Required inputs

- previous runtime-sized ensemble mean prediction;
- previous observed-unknown mask;
- current observed map;
- D1 quantities;
- rolling reliability buffer.

## Exact prediction-to-observation pairing

For each pair of consecutive evaluable decisions:

1. keep the **immediately previous** runtime-sized `mean_map`;
2. find cells that were unknown at the previous evaluable decision and are known now;
3. for those newly revealed cells:
   - old prediction = previous `mean_map_runtime[x]`;
   - observed target:
     - free -> `0.0`
     - occupied -> `1.0`;
4. append these pairs to a rolling cell buffer.

Do not choose an arbitrary older prediction for the same cell. The canonical pairing is the **most recent prediction made while that cell was still unknown**.

## Reliability metric

Use Brier error:

```text
Brier =
    mean((old_prediction - observed_target)^2)
```

over the most recent `N_reliability_cells` valid revealed-cell pairs.

D4 is non-evaluable until at least:

```text
N_reliability_min
```

pairs are available.

## Primary stop condition

```text
D4_base_valid :=
    (A_mean_m2 < T_area_m2)
    AND
    (U_p95 < T_uncertainty)
    AND
    (Brier < T_brier)
```

STOP after `K_confirm` consecutive evaluable D4-valid decisions.

## Meaning

The robot stops only when it thinks little remains **and** its recent prediction record gives evidence that the prediction can currently be trusted.

## Important limitation

Recently revealed cells may not represent the still-unobserved region. D4 therefore provides a reliability gate, not proof of global correctness.

---

# 10. Direction 5 — Ensemble Risk / Missing-Area Consensus

## Question

Do all three current MapEx ensemble members agree that no materially large reachable region remains?

## Why the previous probability wording is rejected

Current MapEx has exactly:

```text
N = 3 ensemble members
```

An empirical tail probability based only on three members can take only:

```text
0, 1/3, 2/3, 1
```

Therefore statements such as "stop when miss probability < 5%" are misleading with the current ensemble size.

## Required inputs

Use the same per-member remaining areas as D1:

```text
A_1_m2
A_2_m2
A_3_m2
```

Choose a development parameter:

```text
A_critical_m2
```

meaning "a remaining region this large would still be materially important."

## Primary conservative rule for N=3

```text
MissVotes =
    count(A_j_m2 > A_critical_m2 for j in {1,2,3})

D5_base_valid := MissVotes == 0
```

Equivalent:

```text
max(A_1_m2, A_2_m2, A_3_m2) <= A_critical_m2
```

STOP after `K_confirm` consecutive evaluable D5-valid decisions.

## Meaning

All three completion hypotheses must agree that the remaining reachable unknown free area is below the critical size.

## Future extension

Only if ensemble size is increased substantially may D5 be generalized to an empirical tail-risk threshold such as:

```text
P_hat(A_remaining > A_critical) < epsilon
```

With the current N=3 implementation, the consensus rule above is canonical.

---

# 11. Direction 6 — Lightweight Learned Stop Predictor

## Question

Can a lightweight model learn when stopping is safe from interpretable MapEx-native features, without training another raw-map CNN?

## Runtime feature contract

Primary initial feature vector:

```text
X_t = [
    A_mean_m2,
    U_p95,
    IG_max,
    Score_max,
    selectable_candidate_count
]
```

Extension features may later add:

```text
Progress_m2_per_m
Brier
best_frontier_distance_m
```

Do not use ground-truth values as runtime features.

Do not feed raw occupancy images in the first D6 implementation.

## Offline label contract

D6 labels are generated only from completed baseline MapEx runs.

For each recorded decision `t`, compare the map state at that decision with the final baseline reference from the same run.

Define `SAFE_STOP_t = 1` only if **both** pre-declared quality losses are within tolerance:

```text
final_coverage - coverage_t <= delta_coverage
AND
final_observed_occupied_iou - occupied_iou_t <= delta_iou
```

Otherwise:

```text
SAFE_STOP_t = 0
```

Ground truth/future information is used only to create offline labels, never at runtime.

If occupied-IoU semantics differ across old run cohorts, those cohorts must not be mixed until the evaluator semantics are reconciled.

## Primary model

Start with:

```text
logistic regression
```

Then compare with a shallow tree-based model.

A deep neural network is not justified until simpler models are shown insufficient.

## Dataset split rule

Never randomly split individual decision rows from the same run across train and test.

Use group splitting at least by:

```text
run_id
```

and, for generalization claims, evaluate on held-out environments.

## Runtime stop condition

```text
p_safe_stop = model.predict_proba(X_t)

D6_base_valid := p_safe_stop >= T_probability
```

STOP after `K_confirm` consecutive evaluable decisions.

## Critical error metric

The most dangerous error is:

```text
predicted SAFE_STOP
but stopping would exceed the allowed map-quality loss
```

Report this false-STOP rate separately. Overall classification accuracy is not sufficient.

---

# 12. Common output and logging contract

Every evaluable decision must record enough information to replay the stop decision offline.

Minimum shared log:

```text
decision_id
sim_time_s
termination_method
evaluable
warmup_active
valid_count
should_stop
stop_reason

A_1_m2
A_2_m2
A_3_m2
A_mean_m2
U_p95

IG_max
Score_max
selectable_candidate_count

KnownArea_m2
trajectory_distance_m
Progress_m2_per_m

reliability_pair_count
Brier

MissVotes
A_critical_m2

D6_probability
```

Fields unavailable for a direction are recorded as null, not silently omitted when a shared schema is used.

Also preserve the existing MapEx per-decision artifacts:
- observed raw map;
- observed fixed-canvas map;
- P1/P2/P3;
- mean/variance;
- frontier candidate metrics;
- selected goal;
- trajectory/goals/plans.

---

# 13. Common evaluation protocol

## Baseline

Original `scripts/mapex.py` with its normal completion machinery.

The baseline file should remain unchanged.

## Primary efficiency metrics

```text
time_saved_fraction =
    (T_baseline - T_stop) / T_baseline

distance_saved_fraction =
    (D_baseline - D_stop) / D_baseline
```

## Primary quality metrics

At minimum:
- coverage on the canonical evaluation ROI;
- observed occupied IoU where evaluator semantics are valid and comparable;
- MapEx paper metrics already available in the workspace.

## Premature-stop definition

A run is a premature/unsafe early stop if its final quality loss exceeds the tolerance declared **before validation**.

Do not redefine the tolerance after seeing validation outcomes.

## Required reporting

For every direction report:
- trigger rate;
- time saving;
- distance saving;
- final quality loss;
- false/premature STOP rate;
- distribution across repeated runs;
- environment generalization.

A single successful run is never sufficient evidence.

---

# 14. Parameter-freeze checklist before prospective validation

Before any D1...D6 prospective validation run, the implementation must record a frozen configuration containing every parameter used by that direction.

## Shared

```text
warmup_decisions
K_confirm
predicted_free_threshold = 0.5
connectivity = 8
```

## D1

```text
T_area_m2
T_uncertainty
uncertainty_statistic = p95
```

## D2

```text
T_ig
# or T_score for the separate cost-aware ablation
```

## D3

```text
T_area_m2
T_uncertainty
W_progress
D_window_min_m
T_progress_m2_per_m
```

## D4

```text
T_area_m2
T_uncertainty
N_reliability_cells
N_reliability_min
T_brier
```

## D5

```text
A_critical_m2
MissVotes_required = 0
```

## D6

```text
feature_list
model artifact/hash
T_probability
delta_coverage used for labels
delta_iou used for labels
training-run IDs
validation-run IDs
```

---

# 15. Implementation architecture

Do not copy the full `mapex.py` six times.

Implement one reusable stopping layer that consumes a snapshot of already-computed MapEx state.

Conceptual structure:

```text
mapex.py                         # unchanged baseline
early_stopping/
    common.py                    # runtime crop, R_j, A_j, U_p95, state machine
    direction1.py
    direction2.py
    direction3.py
    direction4.py
    direction5.py
    direction6.py

mapex_early_stop.py              # MapEx subclass / integration point
```

A conceptual snapshot:

```text
StopSnapshot:
    decision_id
    grid
    robot_cell
    predictions_runtime[3]
    mean_map_runtime
    variance_map_runtime
    selectable_evaluations
    cumulative_trajectory_distance
    history
```

A conceptual result:

```text
StopDecision:
    stop: bool
    direction: D1...D6
    reason: string
    valid_count: int
    diagnostics: dict
```

Do not expose a generic "confidence" field unless the direction has a quantity with a defensible meaning. D5 consensus and D6 model probability are not automatically calibrated confidence.

---

# 16. Required implementation order

The six directions are the active research program, but they should be implemented in a controlled order.

## Stage A — common infrastructure

Implement and unit-test:
- prediction crop;
- robot-cell conversion;
- `R_j`;
- `A_j_m2`;
- `A_mean_m2`;
- `R_union`;
- `U_p95`;
- generic K-confirmation state machine;
- shared logging.

## Stage B — D2 plumbing baseline

Implement D2 first because it uses existing frontier metrics and validates that:
- CONTINUE leaves MapEx unchanged;
- STOP prevents a new Nav2 goal;
- normal no-frontier completion is still separate.

## Stage C — D1 main global-completeness method

Implement D1 exactly from Sections 4 and 6.

This is the first major new prediction-aware method.

## Stage D — D3 and D4 robustness

Add:
- progress-per-metre history;
- online Brier reliability buffer.

## Stage E — D5 consensus risk

Reuse D1's already-tested per-member `A_j_m2`.

## Stage F — D6 learned extension

Only after stable D1-D5 features and logs exist should D6 training data be generated.

---

# 17. How the six directions relate

They are not six arbitrary unrelated ideas.

```text
D2
simple MapEx-native saturation baseline
        ↓
D1
global predicted remaining-map completeness + uncertainty
        ↓
D3 / D4
two independent robustness extensions:
real progress / online prediction reliability
        ↓
D5
conservative ensemble consensus about missed area
        ↓
D6
learned combination of interpretable MapEx-native signals
```

All six are now active research candidates.

For implementation order, **D1 is the first major method**, D2 is the plumbing/baseline method, D3-D5 are principled robustness/probabilistic extensions, and D6 is the final learned extension.

This ordering is an engineering/research plan, not a claim that D1 will necessarily be the final winning method.
