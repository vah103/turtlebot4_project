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


# Source tag index — 14 stopping references

Use these tags inside D1...D6. The tags indicate **research ancestry / implementation inspiration**, not a claim that the new direction is identical to the cited source.

| Tag | Source | Type / role |
|---|---|---|
| **[SRC-01]** | *Enough is Enough: Towards Autonomous Uncertainty-driven Stopping Criteria* | Saturation / repeated low-improvement stopping |
| **[SRC-02]** | *Estimating Map Completeness in Robot Exploration* | Learned map-completeness stopping; companion code: `aislabunimi/exploration-aware` |
| **[SRC-03]** | *Optimizing Exploration with a New Uncertainty Framework for Active SLAM Systems* | Uncertainty as exploration/completion evidence |
| **[SRC-04]** | *PUL-SLAM: Path-Uncertainty Co-Optimization with Lightweight Stagnation Detection for Efficient Robotic Exploration* | Stagnation detection |
| **[SRC-05]** | *A Novel Stop Criterion to Support Efficient Multi-Robot Mapping* | Expected-vs-actual information stopping |
| **[SRC-06]** | *Exploration of Indoor Environments through Predicting the Layout of Partially Observed Rooms* | Predicted remaining useful area → early termination |
| **[SRC-07]** | *Sampling-based Incremental Information Gathering with Applications to Robotic Exploration and Environmental Monitoring* | Information/entropy saturation |
| **[SRC-08]** | Valerii Stakanov MSc thesis — *A frontier-based exploration strategy informed by an estimation of map completeness* | Map-completeness model + Grad-CAM/frontier guidance; same research lineage as SRC-02 |
| **[SRC-09]** | Zhuoqi Zheng PhD thesis — *Autonomous Exploration of Mobile Robots in Complex Environments* | Predicted layout / expected remaining information / early termination |
| **[SRC-10]** | `Leety09/autonomous-frontier-explorer` | Practical threshold + repeated/fallback completion pattern |
| **[SRC-11]** | `mertgulerx/frontier_exploration_ros2` | ROS2 exploration-complete event / termination integration |
| **[SRC-12]** | `cvg/OpenFrontier` | Termination guards, no-frontier handling, timeout/completion separation |
| **[SRC-13]** | `Incomprehensible/RRT_exploration` | Frontier-detection-rate / late-stage stagnation idea; proposed rather than completed stopping implementation |
| **[SRC-14]** | `geo-179/autonomous_exploration_of_unknown_environments` | Entropy-boundary + coverage completion heuristic; completion function exists but was not wired into runtime loop when inspected |

### How to use the tags

- **PRIMARY**: closest methodological precedent; read first before modifying the direction.
- **SUPPORTING**: informs a component, safety guard, or failure mode.
- **IMPLEMENTATION**: useful for software/state-machine integration, not evidence that the research algorithm itself is novel or validated.
- A tag does **not** mean "copy this method". Each D1...D6 definition below remains authoritative.

Shared implementation references for all six directions:

- **[SRC-10]** — threshold / persistence / fallback patterns.
- **[SRC-11]** — ROS2 completion signalling and clean mission termination.
- **[SRC-12]** — keep early-stop, no-frontier completion, timeout and other termination reasons distinct.

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


## 4.4 Reachability primitives

For ensemble member j, the shared **raw predicted navigable mask** is:

~~~text
navigable_j_raw =
    observed_free
    OR
    (observed_unknown AND predicted_free_j)
~~~

Observed occupied cells are always blocked, regardless of prediction.

The shared raw connected component is:

~~~text
CC_j_raw =
    8-connected component of navigable_j_raw
    containing robot_cell

R_j_raw =
    CC_j_raw
    AND observed_unknown
    AND predicted_free_j
~~~

If the robot cell is outside the map, the crop is invalid, or the robot seed cannot be represented consistently, the stopping decision is non-evaluable. Do not invent a nearest seed inside stopping code.

### Important: raw connectivity is not automatically the final D1 reachability definition

An 8-connected pixel path can pass through gaps that the real robot cannot traverse.

Therefore:

- D1 must evaluate both raw and footprint-aware reachability as specified in Section 6.8;
- D3 and D4 inherit the **frozen D1 reachability mode** because they extend D1;
- D5 uses the same frozen per-member remaining-area construction when it reuses A_1...A_3;
- no direction may silently switch reachability semantics between development and validation.

The exact meaning of predicted reachable remaining area is therefore:

> currently unknown cells predicted free by a given ensemble member and belonging to the robot-reachable predicted component under the direction's frozen reachability mode.

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


## 4.6 D1 uncertainty candidates and selection discipline

The current MapEx variance map is useful evidence, but **D1 must not assume in advance that one arbitrary summary statistic is the correct stopping uncertainty signal**.

For every evaluable decision, compute and log at least the following candidates over:

~~~text
R_union = R_1 OR R_2 OR R_3
~~~

### Candidate U1 — P95 MapEx variance

~~~text
U_p95 =
    percentile_95(
        variance_map_runtime[x]
        for x in R_union
    )
~~~

### Candidate U2 — mean MapEx variance

~~~text
U_mean =
    mean(
        variance_map_runtime[x]
        for x in R_union
    )
~~~

### Candidate U3 — ensemble binary disagreement

For each cell x in R_union, count how many of the three ensemble members classify it as predicted free:

~~~text
free_votes(x) in {0, 1, 2, 3}

cell_disagreement(x) =
    min(free_votes(x), 3 - free_votes(x)) / 3
~~~

Then:

~~~text
U_disagreement =
    mean(cell_disagreement(x) for x in R_union)
~~~

With N=3 this is intentionally coarse. It is still useful as a direct measure of ensemble class disagreement and must not be described as a calibrated probability.

### Empty-support handling

If all three members predict no reachable remaining region:

~~~text
R_union is empty
AND
A_1_m2 = A_2_m2 = A_3_m2 = 0
~~~

record:

~~~text
U_p95          = 0
U_mean         = 0
U_disagreement = 0
~~~

This means the ensemble has no remaining-region support to evaluate; it does **not** by itself prove that the map is complete. D1 still requires temporal confirmation and must pass the offline validation protocol in Section 6.

If R_union is empty because of an invalid crop, invalid robot pose, missing prediction, or another data-integrity problem, the decision is **non-evaluable**. Never convert a data failure into zero uncertainty.

### Which uncertainty statistic is primary?

Before prospective online validation, D1 must select exactly one:

~~~text
U_primary in {
    U_p95,
    U_mean,
    U_disagreement
}
~~~

using **development data only**.

Selection is based first on whether the statistic is informative about future prediction error, not on which statistic produces the largest time saving.

At minimum, check:

- prediction error as a function of uncertainty quantile;
- rank correlation between uncertainty and later prediction error;
- the rate of **low-uncertainty / high-error** cells or decisions;
- stability across runs;
- stability across New Room and Hospital when both are available.

The selected statistic and the evidence used to select it must be written into the frozen D1 configuration before prospective validation.

If none of the candidate uncertainty measures is meaningfully related to prediction error, then the uncertainty-aware D1 hypothesis has failed. In that case:

- do **not** tune an uncertainty threshold until the result looks good;
- retain remaining-area-only stopping only as an ablation/baseline;
- do not claim that MapEx uncertainty makes stopping safer.

### Safety rule

Uncertainty is always a **separate caution gate**.

Never use a construction such as:

~~~text
remaining_area * confidence
~~~

and then stop when that product is small, because high uncertainty could shrink the product and accidentally make stopping easier.

The intended logic is always:

~~~text
small remaining-environment evidence
AND
low enough uncertainty
~~~

High uncertainty must force CONTINUE, not encourage STOP.


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
T_remaining_fraction
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

# 5.5 Shared implementation references

The following small repositories are mainly **implementation-pattern references**, not evidence that the six research rules are already solved elsewhere:

- **[S10] Leety09/autonomous-frontier-explorer:** simple threshold + repeated/fallback termination pattern.
- **[S11] mertgulerx/frontier_exploration_ros2:** ROS2 completion event / mission-level exploration-complete publication pattern.
- **[S12] cvg/OpenFrontier:** explicit no-frontier termination and clear termination-reason separation.

Use these for software structure, not for novelty claims or threshold values.

---


# 6. Direction 1 — Uncertainty-Aware Predicted Map Completeness

## 6.1 Status and role

D1 is the **main prediction-based early-stopping research direction**.

Its purpose is not merely to invent a threshold. Its purpose is to test the following causal research hypothesis:

> If MapEx can reliably predict the still-unobserved reachable free space near the end of exploration, and if MapEx's uncertainty is informative about when those predictions can be trusted, then the same prediction pipeline can be reused to estimate map completeness and terminate exploration before exhaustive traversal.

D1 is therefore divided into two logically separate parts:

~~~text
D1 feasibility study
    ↓
prove the required signals are useful
    ↓
D1 rule development
    ↓
freeze the rule
    ↓
prospective online validation
~~~

Do **not** skip directly to threshold tuning.

A threshold that happens to work on historical runs is not sufficient evidence that the underlying D1 idea is valid.

---

## 6.2 Research ancestry

### Primary references

- **[SRC-06]** — predicted remaining useful/unexplored area can support early termination.
- **[SRC-02]** — explicit estimation of whether a partial map is sufficiently complete.
- **[SRC-09]** — predicted layout / expected remaining information as exploration-completion evidence.

### Supporting references

- **[SRC-03]** — uncertainty should affect whether prediction-based exploration evidence is trusted.
- **[SRC-08]** — completeness reasoning and the generalization risk of learned completeness estimates.

### Implementation references

- **[SRC-10]** — persistence/confirmation patterns.
- **[SRC-11]** — clean exploration-complete signalling.
- **[SRC-12]** — explicit separation between early stop and ordinary no-frontier termination.

### Novelty discipline

D1 must not be described as:

> "paper 6 but with LaMa."

The research question is more specific:

> Can the **existing MapEx ensemble** be used not only to rank frontiers, but also to estimate **how much reachable environment remains**, while using MapEx's own prediction uncertainty as a safety gate?

The exact contribution claim must still be checked against the literature after the method is frozen.

---

## 6.3 What D1 is and is not

D1 is a **global predicted-completeness supervisor**.

It asks:

> According to the full current MapEx ensemble, how much still-unobserved reachable environment plausibly remains, and is the prediction reliable enough to trust that estimate?

D1 does **not** ask:

> Is the currently best frontier still attractive?

That is D2 / frontier-value saturation.

D1 must therefore remain conceptually separate from:

- IG thresholding;
- IG/distance thresholding;
- visible-unknown/distance thresholding;
- candidate-count rules;
- old Way2 constants;
- global unknown variance alone;
- ordinary no-frontier completion.

When D1 returns CONTINUE, the original MapEx frontier ranking and Nav2 execution remain unchanged.

---

## 6.4 Required runtime inputs

At each evaluable MapEx decision, D1 requires:

~~~text
current ROS occupancy grid
robot pose / runtime-grid robot cell

P_1_runtime
P_2_runtime
P_3_runtime

mean_map_runtime
variance_map_runtime

map resolution
crop/padding metadata
~~~

D1 may additionally use:

~~~text
robot footprint / inflation radius
current connected observed-free component
Nav2 footprint/costmap information
~~~

for a more realistic reachability mask.

Ground truth, final maps, future observations and evaluation ROI masks are **offline-only evidence**. They must never enter the runtime STOP decision.

---

## 6.5 Data-integrity preconditions

Before computing any D1 signal, verify:

1. all three predictions exist;
2. all three predictions crop to exactly the current runtime map shape;
3. variance-map crop matches the same shape;
4. robot grid cell is valid;
5. map resolution is known;
6. occupancy masks use the same decision-time map as the predictions;
7. no stale prediction from a previous decision is mixed with the current map.

If any precondition fails:

~~~text
D1 evaluable = False
D1 must not increment or reset K-confirmation
MapEx continues through baseline handling
~~~

A data-integrity failure must never be interpreted as low remaining area or low uncertainty.

---

## 6.6 Step 1 — crop LaMa output to the runtime map

For each ensemble member j:

~~~text
P_j_runtime =
    P_j[
        pad_top : pad_top + H,
        pad_left: pad_left + W
    ]
~~~

where H and W are the exact current ROS occupancy-grid dimensions.

Do the equivalent crop for:

~~~text
mean_map_runtime
variance_map_runtime
~~~

The crop must be unit-tested.

**Padding is not environment.** No stopping quantity may include padded cells.

---

## 6.7 Step 2 — construct the predicted navigable mask

For each ensemble member j:

~~~text
predicted_free_j := P_j_runtime < 0.5
~~~

Observed occupancy always has priority over prediction:

~~~text
observed occupied
    → BLOCKED

observed free
    → FREE

observed unknown + predicted free
    → PROVISIONAL FREE

observed unknown + predicted occupied
    → BLOCKED
~~~

Therefore:

~~~text
navigable_j =
    observed_free
    OR
    (observed_unknown AND predicted_free_j)
~~~

Prediction must never overwrite an already observed occupied cell.

---

## 6.8 Step 3 — define reachability carefully

A central D1 risk is confusing **pixel connectivity** with **robot reachability**.

### Development variant R0 — raw 8-connected reachability

The simplest implementation is:

~~~text
CC_j_raw =
    8-connected component of navigable_j
    containing robot_cell
~~~

then:

~~~text
R_j_raw =
    CC_j_raw
    AND observed_unknown
~~~

This is easy to reproduce and should be implemented first for debugging.

### Development variant R1 — footprint-aware reachability

Raw pixel connectivity can count narrow gaps that a TurtleBot/Nav2 stack cannot physically traverse.

Therefore D1 must also test a footprint-aware variant in which predicted/observed obstacles are inflated by a fixed robot-clearance radius before the connected component is computed.

Conceptually:

~~~text
blocked_j =
    observed_occupied
    OR
    (observed_unknown AND NOT predicted_free_j)

blocked_j_safe =
    inflate(blocked_j, robot_clearance_radius)

navigable_j_safe =
    NOT blocked_j_safe
    AND
    (observed_free OR observed_unknown)

CC_j_safe =
    connected component containing robot_cell

R_j_safe =
    CC_j_safe
    AND observed_unknown
    AND predicted_free_j
~~~

The exact inflation radius must be tied to the robot footprint / Nav2 safety semantics and frozen before validation.

### Reachability freeze rule

Before threshold selection, choose exactly one canonical D1 reachability definition:

~~~text
reachability_mode = raw_8_connected
# or
reachability_mode = footprint_aware
~~~

based on development evidence.

The choice must not be changed after seeing prospective validation results.

The main question is not which variant saves more time. It is which one better represents **actually explorable remaining free space**.

---

## 6.9 Step 4 — per-member remaining area

After the reachability mode is frozen, define for each member j:

~~~text
R_j =
    currently unknown cells
    that member j predicts as free
    and that belong to the robot-reachable predicted component
~~~

Then:

~~~text
A_j_m2 =
    count(R_j) * resolution_m^2
~~~

For the current MapEx ensemble:

~~~text
A_1_m2
A_2_m2
A_3_m2
~~~

and:

~~~text
A_mean_m2 =
    (A_1_m2 + A_2_m2 + A_3_m2) / 3
~~~

Also log:

~~~text
A_min_m2 = min(A_1_m2, A_2_m2, A_3_m2)
A_max_m2 = max(A_1_m2, A_2_m2, A_3_m2)
A_range_m2 = A_max_m2 - A_min_m2
~~~

These extra quantities are diagnostics. They are not automatically part of the primary D1 rule.

---

## 6.10 Step 5 — also compute a scale-normalized completeness signal

An absolute-area threshold can transfer poorly between differently sized environments.

Therefore D1 must log a normalized remaining fraction in addition to A_mean_m2.

First compute the currently observed robot-connected free area:

~~~text
KnownReachableFree_m2 =
    count(
        observed_free cells
        in the current robot-connected observed-free component
    )
    * resolution_m^2
~~~

Then define:

~~~text
RemainingFraction =
    A_mean_m2
    /
    max(
        KnownReachableFree_m2 + A_mean_m2,
        epsilon
    )
~~~

Interpretation:

~~~text
RemainingFraction ≈ 0
→ predicted exploration is nearly complete

larger RemainingFraction
→ a larger fraction of the predicted reachable environment is still unseen
~~~

### Why log both absolute and normalized remaining area?

Absolute area A_mean_m2 is easy to interpret physically and is closest to the predicted-remaining-area literature.

Normalized RemainingFraction is more likely to transfer between New Room and Hospital because it is less tied to map scale.

### Primary completeness-signal selection

Before prospective validation, D1 must choose one primary completeness signal:

~~~text
C_primary =
    A_mean_m2
# or
    RemainingFraction
~~~

The selection must be based on development-data stability and cross-environment transfer, not merely on maximum saving.

The non-selected quantity remains a required ablation and diagnostic.

---

## 6.11 Step 6 — uncertainty over the relevant remaining region

Define:

~~~text
R_union = R_1 OR R_2 OR R_3
~~~

Compute all uncertainty candidates defined in Section 4.6:

~~~text
U_p95
U_mean
U_disagreement
~~~

Then, using development data only, freeze:

~~~text
U_primary_name
U_primary_value(t)
~~~

where U_primary_value(t) is the selected statistic at decision t.

### Critical requirement

The uncertainty statistic must be selected because it provides evidence about **prediction error**, not because one threshold on it happens to create attractive early-stopping numbers.

If low uncertainty frequently coexists with large prediction error, the uncertainty gate is unsafe.

---

## 6.12 Phase 0 — mandatory feasibility study before online D1

D1 is not allowed to become an online stopping method until three gates are evaluated.

### Gate P — prediction fidelity

Question:

> When a cell is unknown at decision t, does the MapEx prediction meaningfully predict what that cell later turns out to be?

Evaluate predictions only on cells that were unknown at t.

Use two offline references when available.

#### P1 — later-observed reference

Compare prediction at t against cells that become observed later in the same completed baseline run.

Advantages:

- uses the robot's own future observations;
- does not require structural ground-truth alignment.

Limitation:

- only evaluates cells the baseline eventually observes.

#### P2 — structural-GT reference

When a frame-correct structural ground truth exists, compare the decision-time prediction against GT on the valid canonical ROI.

Advantages:

- can evaluate still-unobserved regions directly.

Limitation:

- depends on correct frame alignment and evaluator semantics.

At minimum report:

- free/occupied classification accuracy on unknown-at-t cells;
- free-class precision and recall;
- occupied-class precision and recall where class support is sufficient;
- IoU on the evaluated unknown region;
- absolute prediction error if continuous prediction values are meaningful;
- performance by exploration stage;
- performance specifically over the last N decisions before normal completion.

Visual inspection is required for representative success and failure cases, but images alone are not sufficient evidence.

### Gate U — uncertainty informativeness

Question:

> Does higher MapEx uncertainty actually correspond to less reliable prediction?

At minimum evaluate:

~~~text
prediction error vs uncertainty quantile
Spearman rank correlation:
    uncertainty ↔ prediction error

low-uncertainty / high-error rate
cross-run stability
cross-environment stability
~~~

The most dangerous failure mode is:

~~~text
low uncertainty
AND
large prediction error
~~~

because D1 could confidently stop for the wrong reason.

If no uncertainty statistic provides useful separation, D1-full must not be promoted as an uncertainty-aware method.

### Gate R — remaining-area informativeness

Question:

> Does the predicted remaining area at decision t actually tell us how much useful mapping remains after t?

For each completed baseline run and each recorded decision t, compute offline targets such as:

~~~text
FutureCoverageGain_t =
    final_coverage - coverage_t

FutureObservedAreaGain_t =
    final_known_area - known_area_t

FutureIoUGain_t =
    final_observed_iou - observed_iou_t
~~~

when the corresponding metrics are valid.

Then test whether A_mean_m2 and/or RemainingFraction decrease as the actual future gain decreases.

At minimum report:

- rank correlation with future coverage gain;
- rank correlation with future known-area gain;
- error of predicted remaining area against any valid GT-derived remaining-area target;
- curves over the final exploration stage;
- failure cases where D1 predicts "little remains" but large useful map gain still occurs later.

### D1 go/no-go rule after Phase 0

Proceed to D1 threshold development only if:

1. prediction has useful late-stage fidelity;
2. at least one uncertainty statistic is informative enough to act as a caution gate;
3. at least one completeness signal is meaningfully associated with actual remaining exploration gain.

If one of these fails, diagnose the failure before implementing online stopping.

Do not rescue a failed signal by threshold overfitting.

---

## 6.13 Phase 1 — offline counterfactual replay

Once Phase 0 passes, replay D1 over completed baseline MapEx runs.

For every decision t, reconstruct exactly what would have been known online:

~~~text
C_primary(t)
U_primary(t)
warmup state
K-confirm state
~~~

No future information may enter the runtime replay rule.

Future/final information is used only to score the counterfactual consequence of stopping at t.

For a hypothetical stop at decision t, compute:

~~~text
time_saved_fraction
distance_saved_fraction

coverage_loss
occupied_iou_loss
known_area_loss

stop_decision_id
remaining baseline goals avoided
~~~

where metric semantics are valid.

The primary counterfactual question is:

> How much exploration cost would D1 save if it stopped here, and what map quality would be sacrificed?

---

## 6.14 Phase 2 — threshold and rule selection

### Do not optimize saving alone

A threshold that maximizes time saving can trivially stop too early.

D1 must be treated as a constrained optimization problem:

~~~text
maximize:
    time/distance saving

subject to:
    map-quality loss <= pre-declared tolerance
    premature-stop rate <= pre-declared tolerance
~~~

### Search a robust region, not one lucky point

Sweep development parameters around candidate values and inspect the neighborhood.

A candidate is stronger when nearby thresholds produce similar behavior.

Avoid selecting an isolated parameter combination that works only at one exact point.

### Grouped development/holdout discipline

Do not let decision rows from the same run act as independent train/test evidence.

Threshold selection must be grouped by run.

When enough data exist:

~~~text
development runs
    → choose signal definition + thresholds

held-out historical runs
    → audit robustness

prospective online runs
    → final validation
~~~

Environment transfer should be tested explicitly.

For example:

~~~text
develop mainly on New Room
freeze
evaluate on Hospital without retuning
~~~

or the reverse, depending on the available evidence.

---

## 6.15 Candidate D1 stop rules

D1 has two pre-declared rule families.

### D1-A — absolute predicted remaining area

~~~text
D1_A_valid :=
    (A_mean_m2 < T_area_m2)
    AND
    (U_primary < T_uncertainty)
~~~

### D1-B — normalized predicted remaining fraction

~~~text
D1_B_valid :=
    (RemainingFraction < T_remaining_fraction)
    AND
    (U_primary < T_uncertainty)
~~~

The development phase must choose one as the **primary frozen D1 rule**.

The other remains an ablation.

Do not combine both simply because doing so improves one dataset unless that combined rule was pre-declared and evaluated separately.

---

## 6.16 Temporal confirmation

The chosen D1 base condition must hold for K_confirm consecutive **evaluable** decisions.

~~~text
if decision is non-evaluable:
    preserve valid_count
    CONTINUE baseline handling

elif D1_base_valid:
    valid_count += 1

else:
    valid_count = 0

if valid_count >= K_confirm:
    STOP_D1_COMPLETENESS
else:
    CONTINUE
~~~

K-confirm protects against transient prediction fluctuations.

It does **not** protect against a systematic prediction error that persists over many decisions. That is why Phase 0 and the uncertainty gate are mandatory.

---

## 6.17 Warm-up

D1 uses the shared decision-count warm-up.

During warm-up:

~~~text
D1 diagnostics are still computed and logged
but stopping is disabled
~~~

This is important: warm-up should prevent STOP, not prevent data collection.

The warm-up threshold must be frozen on development data before prospective validation.

---

## 6.18 Recommended online decision flow

At an evaluable MapEx decision:

~~~text
current observed map
        ↓
3 LaMa predictions
        ↓
runtime crop validation
        ↓
per-member predicted-free masks
        ↓
per-member reachable predicted free space
        ↓
R_1, R_2, R_3
        ↓
A_1, A_2, A_3
        ↓
A_mean + RemainingFraction
        ↓
R_union
        ↓
U_p95 + U_mean + U_disagreement
        ↓
select frozen C_primary + U_primary
        ↓
warm-up check
        ↓
D1 base condition
        ↓
K-confirm
   ↓             ↓
STOP          CONTINUE
                 ↓
        unchanged MapEx frontier selection
                 ↓
              Nav2 goal
~~~

D1 decides only:

> send another exploration goal, or stop?

It does not decide:

> which frontier should MapEx choose?

---

## 6.19 Required offline ablations

Before claiming D1 works, compare at least:

1. **MapEx baseline** — no early stopping.
2. **Absolute-area only** — A_mean_m2 threshold.
3. **Normalized-fraction only** — RemainingFraction threshold.
4. **Primary completeness signal + uncertainty**.
5. **K = 1** diagnostic only.
6. **Frozen K_confirm > 1**.
7. **raw 8-connected reachability**.
8. **footprint-aware reachability**.
9. uncertainty candidates:
   - U_p95;
   - U_mean;
   - U_disagreement.

The purpose of these ablations is to answer:

- Does prediction-based completeness help?
- Does normalization improve transfer?
- Does uncertainty reduce unsafe stops?
- Does realistic reachability matter?
- Does temporal confirmation reduce transient false stops?

Do not report only the best variant.

---

## 6.20 Required prospective online comparison

After the full D1 configuration is frozen, run:

~~~text
original MapEx
vs
MapEx + frozen D1
~~~

under the same experimental conditions.

At minimum report:

~~~text
trigger rate
time saved
distance saved
final coverage difference
final map-quality difference
premature-stop rate
run-to-run variance
termination reason
environment
~~~

D1 thresholds must not be changed from prospective outcomes.

A run where D1 does not trigger is still valid evidence.

---

## 6.21 Premature-stop definition

Before prospective validation, declare acceptable map-quality tolerances.

For a run stopped by D1, mark it unsafe/premature if the counterfactual/final-quality loss exceeds a frozen tolerance.

Examples of possible quality constraints:

~~~text
coverage_loss <= delta_coverage_max
occupied_iou_loss <= delta_iou_max
~~~

The exact metrics and tolerances must be chosen before validation.

Never redefine "safe" after seeing D1 results.

---

## 6.22 Logging contract specific to D1

Every evaluable decision must log:

~~~text
decision_id
sim_time_s
map_shape
resolution_m
robot_cell
evaluable
non_evaluable_reason
warmup_active

reachability_mode
robot_clearance_radius_m

A_1_m2
A_2_m2
A_3_m2
A_min_m2
A_max_m2
A_range_m2
A_mean_m2

KnownReachableFree_m2
RemainingFraction

R_1_cell_count
R_2_cell_count
R_3_cell_count
R_union_cell_count

U_p95
U_mean
U_disagreement
U_primary_name
U_primary_value

C_primary_name
C_primary_value

T_area_m2
T_remaining_fraction
T_uncertainty

base_valid
valid_count
K_confirm
should_stop
stop_reason
~~~

For development/offline analysis also preserve:

~~~text
observed map at decision t
P1/P2/P3 runtime crop
mean map
variance map
R1/R2/R3 masks
R_union mask
robot pose
final/future evaluation references
~~~

This logging is required so every STOP can be explained after the run.

---

## 6.23 Unit tests required before online experiments

At minimum add tests for:

### Crop tests

- no padding;
- symmetric padding;
- asymmetric padding;
- runtime shape changes;
- crop mismatch must become non-evaluable.

### Occupancy-priority tests

- prediction cannot turn observed occupied into free;
- observed free remains traversable;
- unknown predicted occupied is blocked;
- unknown predicted free is provisional free.

### Reachability tests

- one connected predicted room;
- disconnected predicted room;
- one-cell diagonal connection under 8-connectivity;
- narrow corridor rejected by footprint-aware mode when appropriate;
- robot outside grid;
- robot on invalid seed cell.

### Area tests

- exact cell-count-to-m2 conversion;
- empty R_j;
- three members with different remaining areas;
- RemainingFraction bounds in [0,1].

### Uncertainty tests

- empty support;
- all members agree;
- one member disagrees;
- high variance localized inside R_union;
- high variance outside R_union must not affect D1 uncertainty.

### State-machine tests

- one-frame valid condition does not stop when K>1;
- invalid evaluable decision resets valid_count;
- non-evaluable decision preserves valid_count;
- warm-up disables STOP but keeps diagnostics;
- STOP prevents a new exploration goal.

---

## 6.24 Main failure modes to actively search for

D1 is not considered validated until these cases are inspected.

### F1 — confident hallucination

~~~text
prediction wrong
AND
uncertainty low
~~~

This is the most dangerous failure.

### F2 — map-size dependence

A fixed T_area_m2 works in New Room but not Hospital.

This motivates RemainingFraction.

### F3 — unreachable predicted free space

Prediction creates a large free region connected only through a gap the robot cannot traverse.

This motivates footprint-aware reachability.

### F4 — hidden large room behind a bottleneck

D1 predicts little remains immediately before exploration reveals a large new region.

These cases must be explicitly counted as false/premature-stop risks.

### F5 — late-stage oscillation

A_mean or uncertainty alternates around the threshold.

This motivates K-confirmation and threshold-neighborhood analysis.

### F6 — stable systematic error

The wrong prediction persists for many decisions.

K-confirmation does not solve this failure.

### F7 — evaluator leakage

A runtime feature accidentally uses final map, structural GT or future observation.

This invalidates the experiment.

### F8 — threshold overfitting

A threshold is chosen because one or two historical runs look impressive.

Use grouped runs and robust threshold neighborhoods instead.

---

## 6.25 D1 success criteria

D1 should be considered a credible thesis candidate only if all of the following are true:

1. **Prediction evidence** — late-stage predictions contain useful information about still-unobserved cells.
2. **Uncertainty evidence** — the chosen U_primary provides meaningful caution about prediction error.
3. **Completeness evidence** — the chosen C_primary is associated with actual future exploration gain.
4. **Offline replay** — there exists a non-trivial threshold region that saves exploration cost while respecting frozen quality tolerances.
5. **Transfer** — the frozen signal/rule behaves reasonably across multiple runs and does not require per-environment retuning for every map.
6. **Prospective validation** — online D1 reproduces the expected trade-off without using future information.
7. **Interpretability** — each STOP can be explained from logged predicted remaining area, uncertainty and confirmation history.

If these conditions are not met, D1 should be rejected or downgraded rather than repeatedly retuned.

---

## 6.26 Minimum thesis figures/tables for D1

If D1 survives validation, prepare at least:

1. pipeline diagram showing observed map → ensemble → remaining area → uncertainty → confirmation → STOP/CONTINUE;
2. representative late-stage maps showing observed map, P1/P2/P3, R1/R2/R3, variance/disagreement and final/GT reference offline;
3. uncertainty calibration plot: uncertainty quantile vs prediction error;
4. remaining-completeness plot: C_primary(t) vs actual future coverage/known-area gain;
5. threshold-neighborhood / Pareto table: time saved, distance saved, quality loss, premature-stop rate;
6. final baseline comparison: MapEx vs MapEx + D1;
7. failure-case table.

These figures provide evidence for **why** D1 works, not only whether it happened to stop earlier.

---

## 6.27 Final concise definition after freezing

Only after Phases 0–2 are completed should D1 be summarized in a short runtime form.

The frozen rule will have the structure:

~~~text
At each evaluable MapEx decision:

1. Estimate the still-unknown, predicted-free, robot-reachable area
   from each of the three MapEx completions.

2. Aggregate the three members into a frozen completeness signal
   C_primary.

3. Evaluate a frozen uncertainty statistic U_primary over the
   predicted remaining region.

4. Require:
       C_primary < T_completeness
       AND
       U_primary < T_uncertainty

5. Require the condition for K_confirm consecutive evaluable decisions.

6. If true:
       STOP_D1_COMPLETENESS
   else:
       keep original MapEx planning unchanged.
~~~

Until the signal definitions and thresholds are frozen from development evidence, D1 remains a **research hypothesis under evaluation**, not a validated stopping rule.


---

# 7. Direction 2 — Information-Gain Saturation

# 7. Direction 2 — Information-Gain Saturation

## Reference anchors

- **PRIMARY:** **[SRC-05]**, **[SRC-07]**
  - [SRC-05]: stopping from diminishing information value.
  - [SRC-07]: stopping when information/entropy gain saturates.
- **SUPPORTING:** **[SRC-14]**
  - [SRC-14]: practical entropy/boundary-completion heuristic showing how low remaining information structure can be used as a completion signal.
- **IMPLEMENTATION:** **[SRC-10]**, **[SRC-11]**, **[SRC-12]**.

D2 is deliberately a simple MapEx-native saturation baseline; it is not old Way2.

## Question

Even though a valid next frontier exists, has the expected information value of the best remaining frontier become too small?

## Reference tags

- **Primary conceptual:** **[S7]** — information/entropy saturation as a principled termination concept.
- **Supporting expected-information comparison:** **[S5]** — stopping logic based on the relationship between expected and realized information gain.
- **Supporting prototype:** **[S14]** — entropy-boundary/completion heuristic showing how low remaining information structure can be used in a completion test.
- **Implementation support:** **[S10]**, **[S12]** — threshold persistence and explicit separation between early stopping and ordinary frontier exhaustion.

**Interpretation constraint:** D2 is deliberately simpler than the cited systems and is not old Way2.

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

## Reference anchors

- **PRIMARY:** **[SRC-01]**, **[SRC-04]**
  - [SRC-01]: repeated low improvement / saturation over consecutive checks.
  - [SRC-04]: explicit stagnation detection during exploration.
- **SUPPORTING:** **[SRC-13]**, **[SRC-14]**, **[SRC-06]**
  - [SRC-13]: frontier-detection-rate as a late-stage progress signal; note that this was future-work style logic, not a completed stopping implementation.
  - [SRC-14]: combines global progress/completion with remaining entropy-boundary structure.
  - [SRC-06]: provides the prediction-based "little remains" half of the hybrid.
- **IMPLEMENTATION:** **[SRC-10]**, **[SRC-11]**, **[SRC-12]**.

The key D3 distinction is that stagnation alone must never trigger STOP; prediction and uncertainty must agree first.

## Question

Can false early stops be reduced by requiring both:
1. D1 says little predicted environment remains; and
2. recent real exploration is producing very little new known area per metre travelled?

## Reference tags

- **Primary stagnation references:** **[S1]**, **[S4]** — stop only after progress/improvement remains small over a temporal window or repeated decisions.
- **Supporting progress-rate idea:** **[S13]** — frontier-detection-rate slowdown as a stagnation signal; note that S13 proposes switching search mode, not an implemented stop rule.
- **Supporting multi-signal completion prototype:** **[S14]** — combines global completion evidence with remaining boundary structure.
- **Prediction component inherited from D1:** **[S6]**, **[S9]**, **[S3]**.
- **Implementation support:** **[S10]**, **[S12]**.

**Interpretation constraint:** stagnation is only a guard combined with prediction; low progress alone must never be sufficient for the primary D3 STOP.

## Required inputs

- frozen D1 completeness signal `C_primary`, threshold `T_completeness`, and `U_primary`
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
    (C_primary < T_completeness)
    AND
    (U_primary < T_uncertainty)
    AND
    (Progress_m2_per_m < T_progress_m2_per_m)
```

STOP after `K_confirm` consecutive evaluable D3-valid decisions.

## Optional ablation only

Frontier discovery rate may be logged and later tested, but it is **not** part of the primary D3 rule.

---

# 9. Direction 4 — Prediction Reliability-Gated Stopping

## Reference anchors

- **FOUNDATIONAL / SUPPORTING:** **[SRC-02]**, **[SRC-03]**, **[SRC-08]**
  - [SRC-02] and [SRC-08]: show that map-completeness decisions depend on how well a predictor generalizes from partial maps.
  - [SRC-03]: motivates explicit treatment of uncertainty/reliability rather than treating predictions as ground truth.
- **IMPLEMENTATION:** **[SRC-10]**, **[SRC-11]**, **[SRC-12]**.

**Important provenance note:** none of the 14 sources is a direct precedent for the exact D4 rule "validate previous MapEx predictions against cells later observed, compute an online Brier error, then gate D1". D4 is a synthesis motivated by the failure mode exposed by the literature. This distinction must be preserved when writing novelty/related-work text.

## Question

Before trusting D1, have recent MapEx predictions actually matched cells that the robot later observed?

## Reference tags

- **Primary motivation:** **[S2]**, **[S8]** — learned completeness/prediction methods expose a generalization/reliability problem when deciding that a partial map is "complete enough".
- **Uncertainty motivation:** **[S3]** — confidence/uncertainty must be considered before trusting a stopping signal.
- **Prediction-based stopping context:** **[S6]**, **[S9]** — stopping decisions based on predicted unseen structure motivate validating whether prediction can be trusted.
- **Implementation support:** **[S10]**, **[S12]**.

**Important provenance note:** the exact online scheme "old prediction vs later observation → rolling Brier gate" is a **new synthesis for this project**, not a direct algorithm copied from S2/S3/S6/S8/S9.

## Required inputs

- previous runtime-sized ensemble mean prediction;
- previous observed-unknown mask;
- current observed map;
- frozen D1 completeness/uncertainty quantities and thresholds;
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
    (C_primary < T_completeness)
    AND
    (U_primary < T_uncertainty)
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

## Reference anchors

- **PRIMARY:** **[SRC-06]**, **[SRC-09]**
  - Both support the high-level principle "predicted remaining useful environment becomes small → early termination".
- **SUPPORTING:** **[SRC-03]**
  - Supports treating disagreement/uncertainty as meaningful evidence rather than relying on one deterministic completion.
- **IMPLEMENTATION:** **[SRC-10]**, **[SRC-11]**, **[SRC-12]**.

**MapEx-specific step:** the 3-member consensus rule is derived from the existing MapEx ensemble. It is not taken directly from SRC-06 or SRC-09. With N=3, use consensus/votes as specified below rather than claiming a finely calibrated tail probability.

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

## Reference tags

- **Primary predicted-remaining-area lineage:** **[S6]**, **[S9]** — predicted unseen layout/remaining useful area determines whether further exploration is worthwhile.
- **Supporting uncertainty perspective:** **[S3]** — disagreement/uncertainty should affect whether prediction-based stopping is trusted.
- **Implementation support:** **[S10]**, **[S12]**.

**MapEx-specific step:** the 3-member consensus rule comes from the existing MapEx ensemble structure. MapEx itself is the system substrate and is **not counted as one of [S1]...[S14]**.

**Important provenance note:** no source among S1...S14 is claimed to use the exact `max(A_1,A_2,A_3) <= A_critical` rule.

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

## Reference anchors

- **PRIMARY:** **[SRC-02]**, **[SRC-08]**
  - [SRC-02]: direct learned map-completeness / stop-decision precedent; inspect its companion `aislabunimi/exploration-aware` implementation for dataset/evaluation patterns.
  - [SRC-08]: extends the same research lineage and is useful for learned completeness features and interpretation.
- **SUPPORTING:** **[SRC-01]**, **[SRC-03]**, **[SRC-04]**
  - These motivate candidate input features representing saturation, uncertainty and stagnation.
- **IMPLEMENTATION:** **[SRC-10]**, **[SRC-11]**, **[SRC-12]**.

D6 must remain distinct from SRC-02: the first implementation learns from **interpretable MapEx-native scalar features**, not from a new raw occupancy-map CNN.

## Question

Can a lightweight model learn when stopping is safe from interpretable MapEx-native features, without training another raw-map CNN?

## Reference tags

- **Primary learned-completeness lineage:** **[S2]**, **[S8]** — learn whether exploration/map completeness is sufficient from partial-map evidence.
- **Feature motivation from the rule-based directions:** **[S1]**, **[S3]**, **[S6]**, **[S7]**, **[S9]** — progress, uncertainty, predicted remaining area and information saturation provide interpretable candidate features.
- **Evaluation pattern associated with the S2/S8 lineage:** the `exploration-aware` implementation is useful for saved-time and false-positive/false-negative accounting, but it is not counted as an additional conceptual source beyond S2/S8.
- **Implementation support:** **[S10]**, **[S11]**, **[S12]**.

**Interpretation constraint:** D6 learns from MapEx-native scalar features; it is not a reimplementation of the raw-map CNN used in S2/S8.

## Runtime feature contract

Primary initial feature vector:

```text
X_t = [
    A_mean_m2,
    U_primary_value,
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
KnownReachableFree_m2
RemainingFraction

U_p95
U_mean
U_disagreement
U_primary_name
U_primary_value

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
reachability_mode
robot_clearance_radius_m            # if footprint-aware mode is frozen

completeness_statistic              # A_mean_m2 or RemainingFraction
T_area_m2                           # if absolute-area D1 is frozen
T_remaining_fraction                # if normalized D1 is frozen

uncertainty_statistic               # U_p95, U_mean, or U_disagreement
T_uncertainty

warmup_decisions
K_confirm
development_run_ids
heldout_audit_run_ids
quality_tolerances
```

## D2

```text
T_ig
# or T_score for the separate cost-aware ablation
```

## D3

```text
inherit frozen D1 reachability_mode
inherit frozen D1 completeness_statistic + T_completeness
inherit frozen D1 uncertainty_statistic + T_uncertainty
W_progress
D_window_min_m
T_progress_m2_per_m
```

## D4

```text
inherit frozen D1 reachability_mode
inherit frozen D1 completeness_statistic + T_completeness
inherit frozen D1 uncertainty_statistic + T_uncertainty
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
    common.py                    # runtime crop, reachability, R_j/A_j, completeness + uncertainty candidates, state machine
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
- raw and footprint-aware reachability masks;
- R_j;
- A_j_m2;
- A_mean_m2;
- KnownReachableFree_m2;
- RemainingFraction;
- R_union;
- U_p95;
- U_mean;
- U_disagreement;
- generic K-confirmation state machine;
- shared logging.

## Stage B — D2 plumbing baseline

Implement D2 first because it uses existing frontier metrics and validates that:
- CONTINUE leaves MapEx unchanged;
- STOP prevents a new Nav2 goal;
- normal no-frontier completion is still separate.

## Stage C — D1 feasibility and global-completeness method

Do **not** begin with online threshold tuning.

Follow Section 6 in this order:

```text
Gate P: prediction fidelity
→ Gate U: uncertainty informativeness
→ Gate R: remaining-area informativeness
→ offline counterfactual replay
→ freeze reachability/completeness/uncertainty definitions
→ freeze thresholds and K
→ prospective online validation
```

Only after the three feasibility gates pass should D1 be enabled as an online stopping method.

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


# 17. Source-to-direction quick matrix

This matrix is only a navigation aid; the detailed role labels above are authoritative.

| Source | D1 | D2 | D3 | D4 | D5 | D6 |
|---|---:|---:|---:|---:|---:|---:|
| [SRC-01] Enough is Enough |  |  | **P** |  |  | S |
| [SRC-02] Map Completeness | **P** |  |  | S |  | **P** |
| [SRC-03] Uncertainty Framework | S |  |  | S | S | S |
| [SRC-04] PUL-SLAM |  |  | **P** |  |  | S |
| [SRC-05] Multi-Robot Stop Criterion |  | **P** |  |  |  |  |
| [SRC-06] Predicted Room Layout | **P** |  | S |  | **P** |  |
| [SRC-07] IIG |  | **P** |  |  |  |  |
| [SRC-08] Stakanov thesis | S |  |  | S |  | **P** |
| [SRC-09] Zheng thesis | **P** |  |  |  | **P** |  |
| [SRC-10] Leety09 repo | I | I | I | I | I | I |
| [SRC-11] ROS2 frontier-exploration repo | I | I | I | I | I | I |
| [SRC-12] OpenFrontier | I | I | I | I | I | I |
| [SRC-13] RRT_exploration |  |  | S |  |  |  |
| [SRC-14] geo-179 exploration |  | S | S |  |  |  |

Legend:

```text
P = PRIMARY methodological reference
S = SUPPORTING reference
I = IMPLEMENTATION reference
```

---

# 18. How the six directions relate

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


---

# 18. Source tag index — S1...S14

These tags refer to the 14 sources reviewed before defining D1...D6.

| Tag | Source | Main idea relevant here | Evidence role |
|---|---|---|---|
| **S1** | **Enough is Enough: Towards Autonomous Uncertainty-driven Stopping Criteria** (2022) | Repeated small change / saturation before stopping | D3 |
| **S2** | **Estimating Map Completeness in Robot Exploration** | Learned map-completeness / safe stopping from partial maps | D1, D4, D6 |
| **S3** | **Optimizing Exploration with a New Uncertainty Framework for Active SLAM Systems** | Uncertainty as an exploration/termination signal | D1, D4, D5 |
| **S4** | **PUL-SLAM** | Lightweight stagnation detection | D3 |
| **S5** | **A Novel Stop Criterion to Support Efficient Multi-Robot Mapping** | Expected-vs-actual information gain for stopping | D2 |
| **S6** | **Exploration of Indoor Environments through Predicting the Layout of Partially Observed Rooms** (AAMAS 2021) | Predicted remaining visible/unexplored area; early stopping | D1, D4, D5 |
| **S7** | **Sampling-based Incremental Information Gathering (IIG)** | Information/entropy convergence and saturation | D2, D6 feature motivation |
| **S8** | **Valerii Stakanov master thesis — A frontier-based exploration strategy informed by an estimation of map completeness** | Completeness CNN + Grad-CAM / incomplete-region reasoning | D1, D4, D6 |
| **S9** | **Zhuoqi Zheng PhD thesis — Autonomous Exploration of Mobile Robots in Complex Environments** | Predicted layout / remaining useful information; crowd-flow extension | D1, D4, D5 |
| **S10** | **Leety09/autonomous-frontier-explorer** | Simple threshold stop, fallback, counter-style control pattern | Shared implementation |
| **S11** | **mertgulerx/frontier_exploration_ros2** | ROS2 exploration-complete event / mission integration | Shared implementation |
| **S12** | **cvg/OpenFrontier** | No-frontier termination and explicit termination reasons | Shared implementation |
| **S13** | **Incomprehensible/RRT_exploration** | Proposed frontier-detection-rate slowdown; not an implemented stop algorithm | D3 supporting signal |
| **S14** | **geo-179/autonomous_exploration_of_unknown_environments** | Prototype completion heuristic combining explored fraction + entropy-boundary density + sanity check; not wired into runtime | D2/D3 supporting prototype |

## Lineage / non-independence notes

- **S8 is closely tied to the same research lineage as S2**. Do not count S2 and S8 as fully independent evidence for novelty.
- **S9 is conceptually very close to the predicted-layout line represented by S6**. Treat them as one related family when making novelty claims.
- **S13 is not an implemented early-stopping method**; its frontier-rate idea is only supporting inspiration for D3.
- **S14 contains a candidate completion function but it is not integrated into the runtime exploration loop**.
- The `exploration-aware` repository is implementation/evaluation support for the **S2/S8 lineage**, not a separate conceptual source in the 14-source count.

## Quick direction-to-source map

```text
D1 ← S6, S9, S3   + S2, S8
D2 ← S7, S5, S14
D3 ← S1, S4, S13, S14 + D1 lineage
D4 ← S2, S8, S3   + S6, S9
D5 ← S6, S9, S3   + MapEx ensemble substrate
D6 ← S2, S8       + features motivated by S1/S3/S6/S7/S9

Shared software patterns ← S10, S11, S12
```

The tags are for traceability and implementation reference. They **do not mean that a direction is already published exactly as specified here**.
