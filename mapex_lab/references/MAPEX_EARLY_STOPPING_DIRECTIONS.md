# Six research directions for early stopping in MapEx

> **Purpose of this document**
>
> This file is an implementation-oriented research specification for six possible early-stopping directions built on top of MapEx.
> It is intentionally stricter than a brainstorming note: each direction defines the required MapEx inputs, the derived signal, the STOP/CONTINUE rule, failure cases, and the minimum experimental checks needed to avoid drifting away from the original idea.
>
> Related literature notes are in `STOPPING_CRITERIA_RESEARCH.md`.
>
> **Important:** these are candidate research directions, not claims of novelty. Novelty must be checked separately against the literature before writing the thesis/paper contribution statement.

---

## 0. Shared objective

The common objective is:

> **Stop exploration before the robot exhaustively traverses the whole map when the expected value of continuing is already too small, while keeping final map quality within an acceptable loss.**

The stopping mechanism must be an additional supervisory layer. Unless a direction explicitly says otherwise, the original MapEx frontier-selection/planning logic should remain unchanged.

The runtime output should be conceptually:

```text
StopDecision:
    stop: bool
    confidence: float
    reason: string
    diagnostics: dict
```

At every MapEx decision step:

```text
Observed map
    ↓
MapEx ensemble prediction
    ↓
MapEx frontier / information-gain computation
    ↓
StopEvaluator
    ├── CONTINUE → keep original MapEx planner
    └── STOP     → terminate exploration cleanly
```

---

## 1. Shared MapEx inputs

The six directions should reuse signals that MapEx already produces or that can be derived directly from them.

Use the following notation only as a conceptual interface; actual variable/function names must be mapped to the current codebase during implementation.

### 1.1 Observed occupancy map

[
M_t
]

Current occupancy map at decision step (t), with at least:

- observed free cells,
- observed occupied cells,
- unknown cells.

Define:

[
Unknown_t = {x mid M_t(x)	ext{ is unknown}}
]

and:

[
Known_t = {x mid M_t(x)	ext{ is observed}}
]

### 1.2 Ensemble predictions

MapEx generates multiple map completions:

[
hat M_t^{(1)},hat M_t^{(2)},ldots,hat M_t^{(N)}
]

From the ensemble we may derive:

- mean prediction (mu_t(x)),
- variance/disagreement (sigma_t^2(x)),
- per-cell agreement,
- a distribution over a global remaining-area quantity.

### 1.3 Frontiers

[
F_t = {f_1,f_2,ldots,f_m}
]

Current set of valid/reachable MapEx frontiers.

### 1.4 Information gain and cost

For each frontier (f), MapEx can provide or derive:

[
IG_t(f)
]

and a navigation/travel cost such as:

[
d_t(f)
]

A utility can be defined if needed:

[
V_t(f)=rac{IG_t(f)}{d_t(f)+epsilon}
]

Do not silently replace MapEx's current scoring equation with this simplified ratio if the implementation already uses another exact utility formula. Reuse the real MapEx quantity whenever possible.

### 1.5 Temporal history

Store a short history over the last (W) decision steps:

- known-map area,
- number of frontiers,
- new frontier count,
- remaining-area estimate,
- uncertainty,
- best information gain,
- stop-condition truth value.

---

## 2. Shared safety rules for every direction

These rules should be kept unless an experiment is explicitly testing their removal.

### Rule A — no one-frame stopping

A STOP condition must hold for (K) consecutive decision steps:

[
condition_t = condition_{t-1}=cdots=condition_{t-K+1}=True
]

before actually stopping.

This prevents transient prediction noise from terminating exploration.

### Rule B — warm-up period

Do not evaluate early stopping at the very beginning of exploration.

Use either:

- a minimum elapsed time,
- a minimum travelled distance,
- a minimum known-map fraction,
- or a minimum number of MapEx decision steps.

The exact warm-up criterion must be fixed before experiments.

### Rule C — uncertainty must not accidentally encourage stopping

A dangerous formulation is:

[
A^{conf}=sum_x P(x)C(x)
]

followed by STOP when (A^{conf}) is small, because high uncertainty can make (C(x)) small and therefore make the score small, causing a premature stop.

Unless uncertainty is handled probabilistically as in Direction 5, use it as a separate guard:

[
	ext{small remaining area}
quadlandquad
	ext{low enough uncertainty}
]

not as a factor that can shrink the remaining-area score by itself.

### Rule D — distinguish early stop from normal termination

Log a termination reason such as:

```text
STOP_EARLY_COMPLETENESS
STOP_IG_SATURATION
STOP_STAGNATION
STOP_RELIABILITY_GATED
STOP_LOW_MISS_RISK
STOP_LEARNED
STOP_NO_FRONTIER
STOP_TIMEOUT
STOP_MAX_DISTANCE
```

This is necessary for fair experiments.

### Rule E — MapEx planner stays unchanged unless the direction explicitly modifies it

The stopping layer decides **whether to continue**, not **where to go next**.

This keeps the research question interpretable:

> How much exploration can be saved by a stopping mechanism, rather than by simultaneously changing both planning and stopping?

---

# Direction 1 — Uncertainty-Aware Predicted Map Completeness

## Research question

Can MapEx stop when its own ensemble predictions indicate that the amount of **still-unobserved but plausibly reachable/explorable environment** is already small, while also requiring the prediction to be sufficiently certain?

This direction combines:

1. predicted remaining map completeness, and
2. uncertainty-aware gating.

It is the main global-completeness direction.

## Inputs from MapEx

Required:

- (M_t): current observed occupancy map,
- (hat M_t^{(1:N)}): MapEx ensemble predictions,
- ensemble mean/variance or equivalent agreement measure,
- map resolution.

Optional:

- current robot-connected free-space component,
- MapEx reachability or ray-casting utilities.

## Derived quantity 1 — predicted remaining explorable area

For each ensemble completion (j):

1. convert the predicted occupancy map into a predicted free/occupied representation using a fixed threshold;
2. identify predicted free space that is connected/reachable from the currently observed navigable component;
3. keep only cells that are still unknown in (M_t);
4. compute their area.

Let:

[
R_t^{(j)}
]

be the set of currently unknown cells predicted to belong to reachable/explorable free space in ensemble member (j).

Then:

[
A_t^{(j)} = |R_t^{(j)}|cdot a_{cell}
]

and the ensemble mean remaining area is:

[
ar A_t =
rac{1}{N}sum_{j=1}^{N}A_t^{(j)}
]

The exact definition of "explorable" must remain fixed throughout an experiment. Do not switch between "all unknown cells", "predicted free cells", and "visible-from-frontier cells" without explicitly defining separate variants.

## Derived quantity 2 — uncertainty of the remaining region

Compute uncertainty only over cells relevant to the predicted remaining region.

A generic form is:

[
U_t =
rac{sum_x w_t(x)sigma_t^2(x)}
{sum_x w_t(x)+epsilon}
]

where (w_t(x)) selects/weights cells belonging to the predicted remaining region.

Possible implementation alternatives:

- ensemble variance,
- entropy of the ensemble occupancy probability,
- disagreement rate across binary ensemble predictions.

Pick one primary definition for the main experiment; treat others as ablations.

## STOP rule

Base rule:

[
oxed{
ar A_t < T_A
quadlandquad
U_t < T_U
}
]

for (K) consecutive decision steps.

Interpretation:

> MapEx predicts little reachable environment remains **and** it is sufficiently confident in that conclusion.

## CONTINUE rule

Continue if either:

[
ar A_t ge T_A
]

or:

[
U_t ge T_U
]

High uncertainty must force caution rather than encouraging stopping.

## Main implementation risk

The largest risk is defining "remaining area" too loosely.

Do **not** simply count all currently unknown cells, because unknown cells can lie outside the actually reachable/meaningful environment and would recreate a crude coverage metric rather than a prediction-aware completeness metric.

## Minimum experiments

Compare at least:

1. original MapEx with normal termination,
2. remaining-area only:
   [
   ar A_t<T_A
   ]
3. remaining-area + uncertainty:
   [
   ar A_t<T_Aland U_t<T_U
   ]
4. different (K) values.

Measure:

- exploration time,
- travel distance,
- final observed coverage against ground truth **offline**,
- occupied/free IoU or the MapEx map-quality metrics already used,
- premature-stop rate.

---

# Direction 2 — Information-Gain Saturation

## Research question

Can the robot stop when even the best currently available frontier no longer promises enough information to justify further travel?

This is the simplest MapEx-native baseline.

## Inputs from MapEx

Required:

- valid frontier set (F_t),
- MapEx information gain (IG_t(f)),
- optionally navigation cost (d_t(f)) or MapEx's current frontier utility.

## Derived signals

Primary:

[
G_t = max_{fin F_t} IG_t(f)
]

Optional cost-aware version:

[
V_t = max_{fin F_t}
rac{IG_t(f)}{d_t(f)+epsilon}
]

If MapEx already has an exact frontier score, use:

[
S_t=max_{fin F_t}Score_{MapEx}(f)
]

rather than inventing a new utility.

## STOP rule

Information-only:

[
oxed{G_t<T_{IG}}
]

for (K) consecutive decision steps.

Cost-aware:

[
oxed{V_t<T_V}
]

for (K) consecutive steps.

## Interpretation

MapEx normally asks:

> Which frontier is best?

This direction adds:

> Is the best frontier still good enough to be worth visiting?

## Main implementation risk

Low (IG) does **not** prove global map completeness.

A temporary geometry/navigation situation can produce low frontier utility even when substantial unseen space remains.

Therefore this direction should be treated primarily as:

- a simple baseline,
- or a component of a stronger combined method.

## Minimum experiments

Compare:

- (IG_{max}) threshold,
- utility threshold,
- no-frontier normal termination,
- Direction 1.

Track false early stops in maps with long corridors, bottlenecks, or temporarily poor frontiers.

---

# Direction 3 — Prediction + Stagnation Hybrid

## Research question

Can early stopping be made more robust by requiring two independent types of evidence:

1. MapEx prediction says little useful environment remains;
2. actual exploration progress has also become very small?

This combines prediction-based completeness with stagnation/progress signals inspired by several stopping approaches.

## Inputs from MapEx

Required:

- Direction-1 remaining-area estimate (ar A_t),
- current occupancy map (M_t),
- temporal history over a window (W).

Optional:

- number of newly detected frontiers.

## Derived progress signals

### Known-area growth

[
Delta K_t =
rac{|Known_t|-|Known_{t-W}|}{W}
]

or convert to square metres per decision/time unit.

### Frontier discovery rate

Let (N_{new}(t-W:t)) be the number of genuinely new frontier regions appearing during the window.

[
R^F_t =
rac{N_{new}(t-W:t)}{W}
]

Do not use raw current frontier count as a direct substitute for frontier discovery rate.

## STOP rule

A strict version:

[
oxed{
ar A_t<T_A
land
Delta K_t<T_K
land
R^F_t<T_F
}
]

for (K) consecutive decisions.

A simpler first implementation may use:

[
oxed{
ar A_t<T_A
land
Delta K_t<T_K
}
]

## Interpretation

The system stops only when:

> prediction says little remains, **and** recent real exploration confirms that continuing is yielding almost no new map.

## Main implementation risk

Stagnation alone is unsafe.

The robot can show low progress while:

- traversing a long corridor,
- recovering from navigation,
- rotating/relocalizing,
- waiting for a goal,
- temporarily receiving poor sensor geometry.

Therefore stagnation must not be the sole STOP trigger in the main version of this direction.

## Minimum experiments

Ablate:

1. prediction only,
2. stagnation only,
3. prediction + known-area growth,
4. prediction + known-area growth + frontier discovery rate.

Record whether the hybrid reduces false early stops compared with Direction 1.

---

# Direction 4 — Prediction Reliability-Gated Stopping

## Research question

Can the robot decide whether it should trust MapEx's current stopping evidence by measuring how accurate recent MapEx predictions have been on regions that later became actually observed?

This direction adds **online self-validation**.

## Core idea

At an earlier time (t_0), MapEx predicts cells that are still unknown.

At a later time (t_1), some of those cells become observed by the robot.

Those cells provide an online test set:

[
Prediction_{t_0}
quad	ext{vs}quad
Observation_{t_1}
]

## Inputs from MapEx

Required:

- stored previous ensemble mean/probability predictions,
- timestamps or decision-step IDs,
- current observed map,
- Direction-1 completeness signal or another candidate stopping signal.

## Reliability buffer

For every cell that changes from unknown to observed:

1. retrieve the prediction made when the cell was still unknown;
2. compare it with the later observation;
3. accumulate the error over a recent rolling buffer.

Possible reliability metrics:

- Brier score for occupancy probability,
- binary accuracy,
- free/occupied IoU,
- negative log-likelihood if probabilities are calibrated.

A probability-aware metric such as Brier score is preferable for the main implementation if the MapEx prediction output can be interpreted probabilistically.

Example:

[
B_t =
rac{1}{|mathcal V_t|}
sum_{xinmathcal V_t}
(p_{old}(x)-y_t(x))^2
]

where:

- (mathcal V_t) = recently revealed cells,
- (p_{old}(x)) = old predicted occupancy probability,
- (y_t(x)) = later observed occupancy label.

Lower (B_t) means better recent prediction reliability.

## STOP rule

Use reliability as a **gate**, not as the stopping signal by itself.

For example:

[
oxed{
ar A_t<T_A
land
U_t<T_U
land
B_t<T_B
}
]

for (K) consecutive decisions.

If reliability is poor:

[
B_tge T_B
]

then early stopping is disabled and MapEx continues normally.

## Interpretation

The robot effectively says:

> I think little remains, I am not highly uncertain, and my recent predictions have actually been matching reality; therefore I am willing to trust the early-stop decision.

## Main implementation risks

1. Too few revealed cells early in the run make reliability statistically weak.
2. Recently observed cells may not be representative of the still-unobserved region.
3. Class imbalance between free and occupied cells can make simple accuracy misleading.

Use a minimum validation-sample count before enabling this gate.

## Minimum experiments

Compare:

- Direction 1 without reliability,
- Direction 1 + reliability gate,
- different reliability-window sizes,
- different minimum revealed-cell counts.

Measure whether reliability gating mainly reduces premature stops in prediction-failure cases.

---

# Direction 5 — Probabilistic Risk-of-Missing-Area Stopping

## Research question

Instead of thresholding only the mean remaining area, can the ensemble be used directly to estimate the **risk that a significant unexplored region still exists**?

This direction uses the ensemble distribution itself rather than reducing uncertainty to a separate scalar threshold.

## Inputs from MapEx

Required:

- all ensemble completions,
- current observed map,
- the same fixed definition of predicted reachable/explorable area used in Direction 1.

## Per-ensemble remaining area

For every completion (j):

[
A_t^{(j)}
]

is the predicted remaining reachable/explorable area.

The ensemble then approximates a distribution:

[
p(A_t^{remain})
]

## Risk quantity

Choose a critical amount of missed area:

[
A_{critical}
]

Then estimate:

[
P_t^{miss}
=
P(A_t^{remain}>A_{critical})
]

empirically:

[
hat P_t^{miss}
=
rac{1}{N}
sum_{j=1}^{N}
mathbf{1}
[A_t^{(j)}>A_{critical}]
]

## STOP rule

[
oxed{
hat P_t^{miss}<epsilon
}
]

for (K) consecutive decisions.

Interpretation:

> Stop only when the ensemble says the probability that a materially important region still remains is sufficiently small.

Example conceptual statement:

> The robot stops when the estimated probability of more than (A_{critical}) square metres of meaningful unexplored space remaining falls below (epsilon).

Do not hard-code values such as (5%) or (1,m^2) before calibration; they are experiment parameters.

## Difference from Direction 1

Direction 1:

[
	ext{mean remaining area small}
+
	ext{uncertainty low}
]

Direction 5:

[
	ext{tail risk of a large remaining area is small}
]

Direction 5 is therefore a more explicitly probabilistic formulation.

## Main implementation risks

1. Ensemble size may be too small for a stable tail-probability estimate.
2. Ensemble members may not be statistically independent.
3. A poorly calibrated ensemble can make (hat P^{miss}) overconfident.

The thesis must describe this as an **ensemble-based empirical risk estimate**, not automatically as a perfectly calibrated probability.

## Minimum experiments

Compare:

- Direction 1,
- risk threshold with several (A_{critical}),
- several (epsilon) values,
- calibration/reliability analysis of the ensemble if feasible.

Record both average saving and worst-case premature-stop failures.

---

# Direction 6 — Lightweight Learned Stop Predictor from MapEx Features

## Research question

Can a lightweight model learn the STOP/CONTINUE decision from interpretable signals already produced by MapEx, without training a new raw-map CNN?

This is the learned extension, not the first implementation to build.

## Input feature vector

At decision step (t), construct a feature vector such as:

[
X_t =
[
ar A_t,
U_t,
IG_{max,t},
V_{max,t},
|F_t|,
Delta K_t,
R_t^F,
B_t,
d_{best,t}
]
]

Not every feature is mandatory. Start with a minimal feature set and add features through ablation.

Features should come from:

- existing MapEx prediction,
- existing frontier evaluation,
- short temporal history,
- optional online reliability.

Avoid feeding the raw occupancy image initially; otherwise the direction becomes a different heavy map-completeness network problem.

## Offline target / label

A rigorous label must answer:

> Would stopping at this decision step have been safe?

For an offline completed baseline run, define final baseline map quality:

[
Q_{final}
]

and quality available at decision (t):

[
Q_t
]

A possible safe-stop label is:

[
y_t=1
]

if:

[
Q_t ge Q_{final}-delta_Q
]

and optionally if stopping would save at least a minimum meaningful exploration cost.

Otherwise:

[
y_t=0
]

The exact map-quality metric (Q) must be fixed before dataset generation. Prefer metrics already used to evaluate MapEx.

## Model

Start with interpretable/lightweight models:

1. logistic regression,
2. shallow decision tree / random forest,
3. gradient-boosted trees,
4. small MLP only if needed.

Do not begin with a deep network unless simpler models clearly fail.

## Runtime output

[
p_t=P(SAFE_STOPmid X_t)
]

## STOP rule

[
oxed{
p_t>T_P
}
]

for (K) consecutive decisions.

A conservative safety guard may also require:

[
U_t<T_U
]

during early experiments.

## Main implementation risks

1. Label leakage: do not use information at runtime that is only available from ground truth/future observations.
2. Dataset bias: train/test environments must be separated.
3. The model can learn dataset-specific map sizes instead of real stopping structure.
4. A high classification accuracy can hide dangerous false-positive STOP decisions.

The most important error class is:

> predicted STOP when continuing was actually necessary.

Report false-positive STOP rate separately.

## Minimum experiments

Compare:

- logistic regression,
- a tree-based model,
- best hand-crafted rule from Directions 1–5.

Report:

- false STOP rate,
- missed saving opportunities,
- map-quality loss,
- time saved,
- distance saved,
- generalization to unseen maps.

---

# 3. Recommended implementation order

To keep the work controlled and scientifically interpretable, implement in this order:

### Stage A — infrastructure

Create one reusable `StopEvaluator` interface that receives a snapshot of MapEx signals and returns a `StopDecision`.

Do not duplicate stopping logic inside the planner.

### Stage B — simple baseline

Implement Direction 2 first:

[
IG_{max}<T_{IG}
]

This validates the full STOP plumbing with minimal algorithmic work.

### Stage C — main research baseline

Implement Direction 1:

[
ar A_t<T_A
land
U_t<T_U
]

This is the primary prediction-aware global-completeness direction.

### Stage D — robustness extensions

Implement Direction 3 and Direction 4.

These test two different robustness ideas:

- temporal progress consistency,
- online prediction reliability.

### Stage E — probabilistic formulation

Implement Direction 5 from the same per-ensemble remaining-area values already computed for Direction 1.

### Stage F — learned extension

Only after stable rule-based features and logs exist, implement Direction 6.

This prevents collecting an ill-defined training dataset.

---

# 4. Common experimental protocol

Every direction should be evaluated against the same baseline and termination accounting.

## Baseline

Use original MapEx without the new early-stopping rule.

## Required metrics

At minimum:

[
Saving_{time}
=
rac{T_{base}-T_{stop}}
{T_{base}}
]

[
Saving_{dist}
=
rac{D_{base}-D_{stop}}
{D_{base}}
]

and a final map-quality difference:

[
Delta Q = Q_{stop}-Q_{base}
]

Also record:

- observed coverage against ground truth offline,
- occupied/free IoU if available,
- MapEx's existing map-quality metrics,
- number/rate of premature stops,
- stopping step,
- stopping reason.

## Key research constraint

A stopping method is not successful merely because it stops earlier.

The target is:

[
	ext{large time/distance saving}
]

subject to:

[
|Delta Q|le delta_Q
]

for an explicitly chosen acceptable map-quality loss.

---

# 5. Compact comparison of the six directions

| # | Direction | Main STOP evidence | Training required? | Main role |
|---|---|---|---|---|
| **1** | Uncertainty-Aware Predicted Map Completeness | Small predicted remaining area + low uncertainty | No | Main global prediction-based method |
| **2** | Information-Gain Saturation | Best remaining frontier has low expected value | No | Simple MapEx-native baseline |
| **3** | Prediction + Stagnation | Little predicted remaining area + little real progress | No | Temporal robustness |
| **4** | Prediction Reliability-Gated | Base stopping evidence + recent predictions validated by reality | No | Protection against bad predictions |
| **5** | Probabilistic Risk-of-Missing-Area | Low ensemble-estimated risk of significant missed area | No | Probabilistic formulation |
| **6** | Lightweight Learned Stop Predictor | Learned probability that stopping now is safe | Yes, lightweight | Data-driven extension |

---

# 6. Current recommended interpretation

The six directions should not be treated as six unrelated algorithms.

They form a progression:

```text
Direction 2
simple frontier-value baseline
        ↓
Direction 1
global prediction-aware completeness
        ↓
Direction 3 / 4
robustness checks from progress or prediction reliability
        ↓
Direction 5
explicit probabilistic risk formulation
        ↓
Direction 6
learned combination of MapEx-native signals
```

The clearest main research line at this point is **Direction 1**, with Directions 3–5 as principled extensions and Direction 2 as a baseline.

Direction 6 should be considered only after the rule-based signals are defined and logged correctly.
