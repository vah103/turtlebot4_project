# D1 Research Log — Uncertainty-Aware Predicted Map Completeness

> **Project:** TurtleBot4 + MapEx early stopping  
> **Research direction:** D1 — Uncertainty-Aware Predicted Map Completeness  
> **Repository:** `vah103/turtlebot4_project`  
> **Primary specification:** `mapex_lab/references/ES.md`  
> **Analysis workspace:** `mapex_lab/analysis/d1/`  
> **Log status:** active, append-only research record  
> **Initial log reconstruction date:** 2026-09-21

---

# 1. Purpose of this file

This file is the detailed research diary for D1.

It records:

- why D1 was selected;
- how the D1 hypothesis changed over time;
- which implementation decisions were made;
- which alternatives were rejected or kept as ablations;
- exact files and commits associated with each change;
- offline experiment protocol;
- quantitative results;
- limitations discovered after each experiment;
- decisions about whether to continue, revise or stop D1;
- the next concrete action.

This file is intentionally more chronological and decision-oriented than `ES.md`.

Use the documents as follows:

- `ES.md` = current research specification / methodological contract;
- `analysis/d1/README.md` = how to run D1 analysis code;
- `D1_RESEARCH_LOG.md` = what happened, why it happened, what was learned, and what comes next.

## Logging rule

When D1 changes, append a new dated entry instead of silently rewriting history.

Each major entry should contain:

1. **Question**
2. **Decision**
3. **Reason**
4. **Implementation**
5. **Evidence / result**
6. **Interpretation**
7. **Limitations**
8. **Next action**
9. **Relevant commit(s)**

---

# 2. D1 research objective

The overall project goal is to add an early-stopping layer to MapEx without changing MapEx frontier selection when exploration continues.

The objective is not to stop as early as possible.

The objective is:

> reduce exploration time and traveled distance while keeping the loss of final map quality within an acceptable, pre-declared tolerance and avoiding premature termination.

D1 specifically asks:

> Can MapEx's existing LaMa ensemble be reused to estimate how much **still-unknown, predicted-free, robot-reachable environment remains**, and can MapEx uncertainty be used as a safety gate before stopping?

The central idea is therefore global predicted completeness, not current-frontier utility.

D1 must remain separate from Way2-style questions such as:

> Is the best current frontier worth its travel cost?

When D1 returns CONTINUE, original MapEx planning must remain unchanged.

---

# 3. Context before D1

Before D1, two earlier stopping directions had already exposed important risks.

## Way1 lesson

Way1 used a global unknown-variance P95 threshold.

It showed good results in New Room but failed to transfer to Hospital because Hospital variance remained near a different regime.

Main lesson carried into D1:

> do not assume that one raw uncertainty statistic or one threshold transfers across environments.

Therefore D1 must validate uncertainty before using it as a stop gate.

## Way2 lesson

Way2 used frontier utility based on information gain and visible unknown cells normalized by distance.

Way2 was useful as an online stopping implementation and logging reference, but it answers a local frontier-value question.

Main lesson carried into D1:

> D1 should not become another frontier-utility rule. It should test global prediction-based completeness.

These previous results motivated a more explicit research-validation pipeline before enabling a new online STOP condition.

---

# 4. 2026-09-21 — D1 introduced as one of six research directions

## Question

What new early-stopping directions are worth investigating after the earlier Way1/Way2 work?

## Decision

Define six candidate research directions and make D1 the principal prediction-based global-completeness direction.

Initial D1 concept:

```text
Observed map
    ↓
MapEx LaMa ensemble
    ↓
estimate predicted remaining environment
    ↓
combine with prediction uncertainty
    ↓
STOP / CONTINUE
```

## Important conceptual constraint

Uncertainty must not be multiplied into remaining area in a way that can make high uncertainty encourage STOP.

Unsafe form:

```text
remaining_area * confidence
```

Preferred logic:

```text
small remaining area
AND
low uncertainty
```

High uncertainty should force CONTINUE.

## Relevant commits

- `cecdc850e4be2a365ca1613a545190ff6cc86441` — define six MapEx early-stopping research directions.
- `9ed8b3ca76c00b7ffafb1e9d39fec2cb8d7112de` — harden the six stopping specifications.
- `e8b5fd1ea506f8807a79b8ce97e2835495c3ad82` — close Way2 as the active development direction and activate the six-direction research program.
- `e4bec0b01a526c3c93d54208ebdb51a45d16a796` — tag reviewed literature sources across D1–D6.

---

# 5. 2026-09-21 — D1 methodology substantially revised

## Question

Should D1 immediately implement a fixed rule such as:

```text
A_mean < T_area
AND
U_p95 < T_uncertainty
```

and then tune thresholds?

## Decision

No.

D1 was rewritten so that the research must first validate the signals that the stopping rule depends on.

The revised sequence became:

```text
Gate P — prediction fidelity
    ↓
Gate U — uncertainty informativeness
    ↓
Gate R — remaining-area informativeness
    ↓
offline counterfactual replay
    ↓
freeze signal definitions + thresholds + K
    ↓
prospective online validation
```

The key methodological change is:

> D1 is first a research hypothesis to validate, not a threshold rule to tune.

## Three mandatory feasibility gates

### Gate P — Prediction fidelity

Question:

> When a cell is unknown at decision t, does MapEx prediction provide useful information about what that cell later turns out to be?

### Gate U — Uncertainty informativeness

Question:

> Does larger MapEx uncertainty actually correspond to larger prediction error?

Particular danger:

```text
prediction wrong
AND
uncertainty low
```

### Gate R — Remaining-area informativeness

Question:

> Does predicted remaining explorable area at decision t correspond to how much useful mapping is actually left after t?

Only if these gates are sufficiently supported should D1 proceed to threshold development.

## Completeness signal candidates added

Absolute area:

```text
A_j = count(R_j) * resolution^2
A_mean = mean(A_1, A_2, A_3)
```

Normalized signal:

```text
RemainingFraction =
    A_mean
    /
    (KnownReachableFree + A_mean)
```

Reason for the normalized candidate:

> absolute area may transfer poorly between environments with very different sizes.

## Uncertainty candidates added

D1 no longer assumes `U_p95` is automatically correct.

Candidates:

- `U_p95`
- `U_mean`
- `U_disagreement`

The primary uncertainty statistic must later be chosen using development evidence about prediction error.

## Reachability candidates added

Raw version:

```text
8-connected predicted navigable component containing the robot
```

Footprint-aware version:

```text
inflate blocked cells using robot-clearance semantics
then compute connected reachable component
```

Reason:

> pixel connectivity is not always physical robot reachability.

## Relevant commits

- `71c8c008d04132bc2db5004e330b9590577e3881` — revise D1 research protocol for robust prediction-based early stopping.
- `cee34c0d927459a5b646e645cc50d7b4151435c1` — clean up D1 reachability and inherited stopping contracts.
- `2846132439d0311ea925aba19091e81d011da430` — final documentation cleanup after the D1 rewrite.

## Research status after this change

D1 online stopping was **not allowed yet**.

Immediate next task became Gate P.

---

# 6. 2026-09-21 — dedicated D1 analysis workspace created

## Decision

Keep D1 feasibility work isolated from MapEx runtime.

Directory:

```text
mapex_lab/analysis/d1/
```

Initial structure:

```text
analysis/d1/
├── README.md
└── d1_gate_p.py
```

Later result artifacts are written under:

```text
analysis/d1/results/
```

## Rationale

Gate P is an offline scientific analysis.

It must not:

- change frontier ranking;
- alter Nav2 behavior;
- add an online STOP;
- use future information as a runtime feature.

Future observations are allowed only as offline evaluation targets.

## Relevant commit

- `c1a8432dc2bfe056a2c041828ca37248078f20b0` — create dedicated D1 analysis directory.

---

# 7. 2026-09-21 — Gate P1 initial implementation

## Gate P1 question

For a prediction made at policy decision t:

> among cells that are unknown at t but become observed later in the same baseline run, how accurate was the prediction made at t?

## Initial cohort

New Room baseline:

```text
mpx_001
...
mpx_010
```

Expected total:

```text
365 policy decisions / 10 runs
```

## Prediction sources

For every decision:

- G1
- G2
- G3
- ensemble mean

All saved padded predictions are cropped back to the exact runtime occupancy-grid dimensions before evaluation.

## Primary prediction threshold

Current MapEx convention:

```text
prediction < 0.5   → predicted free
prediction >= 0.5  → predicted occupied
```

## Occupancy target convention

Evaluation class encoding:

```text
free     = 0
occupied = 1
```

## Primary P1 target

The implementation was refined to use:

```text
unknown at decision t
AND
first later policy-decision map where that cell becomes known
```

For each decision-time unknown cell:

1. convert its center to world coordinates;
2. project that position into future decision occupancy grids;
3. scan future decisions in chronological order;
4. take the first future known label;
5. compare the decision-time prediction against that label.

## Why first-later-observed?

The first implementation discussion considered using a final/later map directly.

The chosen target was improved to first-later-observed because it more directly answers:

> Did the prediction at t correctly predict what the robot subsequently discovered?

The final raw map is retained only as a secondary stability diagnostic.

## Relevant commits

- `e1c55118eaa357a88049adec97f8e4b3860be069` — implement D1 Gate P prediction-fidelity analysis.
- `2804f87de6de1a5daf4bf673497062c79a869fc6` — document D1 Gate P workflow.
- `b8fa8501653720a1288fc132e77db51a1acd73d1` — use first later observation for Gate P targets.
- `34545d58ff2d94ad19544bdcf92b92507e4c6326` — document first-observation target semantics.

---

# 8. Gate P1 metrics and diagnostics

## Per-decision metrics

For each predictor:

- accuracy;
- free precision;
- free recall;
- occupied precision;
- occupied recall;
- free IoU;
- occupied IoU;
- macro IoU;
- MAE;
- confusion counts;
- output-range diagnostic.

## Target-support diagnostics

Per decision:

- number of unknown cells;
- number of cells that later receive a P1 target;
- free target count;
- occupied target count;
- fraction of decision-time unknown cells eventually observed;
- first reveal decision;
- last reveal decision;
- mean / median / P90 decisions until reveal;
- mean / median / P90 time until reveal;
- fraction revealed at the immediately following decision.

## First-target stability diagnostic

For cells available in both references:

```text
first later known class
vs
final raw-map class
```

Metric:

```text
first_vs_final_class_agreement
```

This diagnostic was explicitly added so that the research can determine whether the first known observation is stable enough to remain the P1 target.

## Late-stage analysis

Because D1 is intended to stop near completion, a separate last-N analysis was added.

Current default:

```text
late_n = 10
```

Late-stage metrics include class-specific free/occupied precision, recall and IoU.

This change was important because overall accuracy can hide class imbalance.

## Relevant commits

- `24dbe7f09010898e4494f822ce819ade58d771de` — add reveal and target-stability diagnostics.
- `c7b9cef508715cf7f4fa0ca1a7efbe2aa321f0cd` — align Gate P output documentation.
- `120798490c795c3e8394f9304e8e57591ed2832b` — add class-specific late-stage Gate P analysis.
- `057fd81f70060784bad76fbad8764c9eba5e388b` — document class metrics and later-observed limitation.

---

# 9. Gate P1 output contract

Current outputs:

```text
mapex_lab/analysis/d1/results/gate_p/
├── gate_p_decisions.csv
├── gate_p_runs.csv
├── gate_p_summary.json
└── gate_p_plots/
    ├── accuracy_vs_decision_progress.png
    ├── macro_iou_vs_decision_progress.png
    ├── mae_vs_decision_progress.png
    ├── late_stage_accuracy_by_run.png
    ├── late_stage_class_recall_by_run.png
    └── late_stage_class_iou_by_run.png
```

## Relevant commit

- `a3b5457fe55acadc95ccc8434befbf03576755ef` — run D1 Gate P on the New Room baseline and commit the result artifacts.

---

# 10. Gate P1 quantitative results — New Room baseline

## Dataset support

Runs:

```text
10
```

Decisions analyzed:

```text
365
```

Decisions having at least one later-observed target:

```text
286
```

Total prediction-target pairs:

```text
2,647,324
```

Mean reveal delay across run-level summaries:

```text
4.2237 decisions
130.69 s
```

## Target stability

Run-macro agreement:

```text
first-later-observed class
vs
final-map class

≈ 0.9615
≈ 96.15%
```

### Decision

Do **not** change P1 from first-later-observed to a more complicated "stable for two consecutive decisions" target yet.

Reason:

> 96.15% first-vs-final class agreement is already high enough that target instability is not currently the main Gate P problem.

This decision may be revisited if later analysis identifies environment-specific instability.

---

# 11. Gate P1 prediction results — ensemble mean

## Overall run-macro metrics

Ensemble mean:

```text
accuracy ≈ 0.8889
macro IoU ≈ 0.6935
MAE ≈ 0.1229
```

## Overall pair-micro metrics

```text
accuracy            ≈ 0.8871

free precision      ≈ 0.9390
free recall         ≈ 0.9284
free IoU            ≈ 0.8756

occupied precision  ≈ 0.6013
occupied recall     ≈ 0.6414
occupied IoU        ≈ 0.4500

macro IoU           ≈ 0.6628
```

Interpretation:

- predictions contain substantial useful information;
- overall free-space prediction is strong on the P1-observed sample;
- occupied performance is weaker overall;
- accuracy alone is therefore not sufficient to characterize prediction quality.

---

# 12. Prediction behavior by exploration stage

## Early stage — ensemble mean

```text
accuracy            ≈ 0.8860
free IoU            ≈ 0.8770
occupied IoU        ≈ 0.3866
macro IoU           ≈ 0.6318
```

## Mid stage — ensemble mean

```text
accuracy            ≈ 0.8903
free IoU            ≈ 0.8434
occupied IoU        ≈ 0.6609
macro IoU           ≈ 0.7521
```

## Late stage — ensemble mean

For the broad final-third stage:

```text
accuracy            ≈ 0.8918

free precision      ≈ 0.6731
free recall         ≈ 0.6244
free IoU            ≈ 0.5052

occupied precision  ≈ 0.9217
occupied recall     ≈ 0.9231
occupied IoU        ≈ 0.8577

macro IoU           ≈ 0.6915
MAE                 ≈ 0.1113
```

Important observation:

> the composition of later-observed targets changes strongly near the end of exploration.

Late-stage occupied labels are much easier / more dominant in the P1 target sample, while free-space metrics become substantially weaker and more variable.

This matters because D1 ultimately wants to estimate **remaining reachable free space**.

---

# 13. Last-10 analysis and the right-censoring problem

The last-10 summary initially appears encouraging:

```text
ensemble-mean late run-macro accuracy ≈ 0.9079
ensemble-mean late run-macro macro IoU ≈ 0.7122
ensemble-mean late run-macro MAE ≈ 0.0960
```

However, these values must not be interpreted alone.

## Support in the last 10 decision positions

Across 10 runs:

```text
100 last-10 decision positions
21 have at least one later-observed target
2,758 prediction-target pairs
```

Class support:

```text
free targets     = 656
occupied targets = 2,102
```

Mean future-observed fraction of unknown among evaluable last-10 decisions:

```text
≈ 0.0743
≈ 7.43%
```

Therefore the last-10 P1 evaluation is heavily right-censored.

Reason:

- the closer t is to the end of the baseline run,
- the less future exploration remains,
- therefore fewer currently unknown cells can ever receive a later-observed target.

## Consequence

The result:

```text
late accuracy ≈ 0.91
```

does **not** prove that the predictor is approximately 91% accurate over the complete unknown region near termination.

It only describes the small subset of late unknown cells that baseline exploration subsequently observes.

This is the largest limitation identified in Gate P1.

---

# 14. Broad late-stage support

Using the broad final-third "late" stage:

```text
late decision rows          = 128
evaluable late decisions    = 49
target pairs                = 13,293
free targets                = 4,330
occupied targets            = 8,963
mean observed fraction      ≈ 13.98%
```

Comparison:

```text
early target coverage ≈ 95.04%
mid target coverage   ≈ 64.92%
late target coverage  ≈ 13.98%
last-10 coverage      ≈ 7.43%
```

This confirms that P1 target availability collapses as the exploration approaches completion.

---

# 15. Run-level late-stage variability

Prediction quality is not uniformly strong across runs.

Examples for ensemble mean:

## mpx_001

```text
late accuracy     ≈ 0.9257
late free IoU     ≈ 0.6631
late occupied IoU ≈ 0.9102
```

## mpx_004

```text
late accuracy     ≈ 0.9100
late free IoU     ≈ 0.0769
late occupied IoU ≈ 0.9093
```

This is an important warning:

> high overall accuracy can coexist with extremely weak free-space fidelity.

## mpx_007

```text
late accuracy     ≈ 0.8841
late free IoU     ≈ 0.2933
late occupied IoU ≈ 0.8773
```

## mpx_008 — case study candidate

```text
late accuracy     ≈ 0.7495
late free IoU     ≈ 0.5134
late occupied IoU ≈ 0.6422

late free recall      ≈ 0.7903
late occupied recall  ≈ 0.6933
```

This is currently the most visible low-accuracy late-stage run.

### Decision

`mpx_008` should be inspected as a **case study**, but it should not block the next main research step.

Its apparent failure could be a combination of:

- genuine prediction failure;
- very low target support;
- class composition;
- decision-level aggregation.

P2 is more important than over-analyzing one P1 run.

---

# 16. Ensemble mean vs G1/G2/G3

P1 evaluates all four predictors:

```text
mean
G1
G2
G3
```

Overall run-macro accuracy:

```text
mean ≈ 0.8889
G1   ≈ 0.8854
G2   ≈ 0.8633
G3   ≈ 0.8842
```

Last-N run-macro accuracy:

```text
mean ≈ 0.9079
G1   ≈ 0.9067
G2   ≈ 0.9100
G3   ≈ 0.9023
```

Interpretation:

- the ensemble mean is strong overall;
- it is not automatically superior to every individual member on every late-stage aggregate;
- member comparison is useful diagnostic evidence;
- member comparison is **not** the main Gate P pass/fail criterion.

The key Gate P question remains whether the D1 prediction representation is useful over the relevant unknown space.

---

# 17. Gate P1 limitation formally recorded

P1 evaluates:

```text
Unknown_t ∩ EventuallyObserved
```

It does **not** evaluate:

```text
all valid Unknown_t
```

Therefore the strongest valid P1 claim is:

> MapEx predictions contain useful fidelity on cells that are unknown at decision t and are subsequently observed by the completed baseline exploration.

P1 alone cannot justify the stronger claim:

> MapEx predictions are accurate over the complete unknown region near exploration termination.

This distinction is now part of the analysis documentation and must be preserved in thesis reporting.

---

# 18. Current Gate P1 verdict

## Verdict

```text
Gate P1 = PRELIMINARY PASS
Gate P  = NOT YET COMPLETE
```

## Why P1 receives a preliminary pass

Positive evidence:

- large total prediction-target sample;
- useful overall accuracy/IoU;
- strong first-vs-final target stability;
- prediction quality does not collapse globally near the end;
- class-specific diagnostics are available;
- final/future information is kept offline-only.

## Why Gate P is not yet complete

Main unresolved issue:

> late-stage P1 evaluates only a small, non-random subset of the unknown region.

Additional concern:

> late predicted-free fidelity is substantially weaker and more variable than the headline accuracy suggests.

Because D1's remaining-area estimate depends on predicted free space, this issue must be tested with a broader reference.

---

# 19. Metadata provenance issue

The Gate P summary reports metadata run-ID mismatches for 9 of the 10 run directories.

Example pattern:

```text
directory: mpx_001
metadata:  mpx_101
```

The Gate P analyzer intentionally treats the **directory name** as the canonical run identifier.

## Decision

This provenance mismatch should be documented or repaired separately.

It does **not** currently block Gate P analysis because:

- artifact paths are resolved from the actual run directory;
- the analysis records both directory and metadata IDs;
- the scientific target/metric computation does not depend on the historical metadata run ID.

Do not silently hide this mismatch in future reporting.

---

# 20. Next decision — Gate P2 structural ground truth

## Main question

Can the good/borderline conclusions of P1 be reproduced when prediction is evaluated over a broader valid unknown region rather than only cells the baseline later happens to observe?

## Decision

The **next main coding task is Gate P2 structural-GT evaluation**.

Do not move to Gate U yet.

Do not implement D1 online stopping yet.

Do not tune D1 thresholds yet.

## Preferred implementation

Extend the existing:

```text
mapex_lab/analysis/d1/d1_gate_p.py
```

instead of creating a separate metric implementation.

Suggested interface:

```bash
--reference later_observed
--reference structural_gt
--reference both
```

Preferred analysis mode:

```bash
--reference both
```

Reason:

> P1 and P2 should use the same crop, prediction threshold, class conventions, aggregation logic and reporting pipeline so their results can be compared directly.

---

# 21. Gate P2 intended semantics

For each decision t:

```text
decision-time observed map
+
decision-time G1/G2/G3/mean prediction
+
frame-correct structural GT
+
valid canonical ROI
        ↓
select cells:
    unknown at t
    AND valid in structural GT / evaluation ROI
        ↓
compare prediction(t)
against structural-GT class
        ↓
compute the same fidelity metrics
```

Primary P2 region:

```text
Unknown_t ∩ ValidStructuralGT
```

The exact valid ROI/frame semantics must match the current corrected New Room structural-GT v2 evaluation contract.

P2 must not reuse the old New Room frame-bugged evaluation semantics.

---

# 22. Gate P2 required outputs

At minimum preserve the same metric families as P1.

## Overall

- accuracy;
- free precision / recall;
- occupied precision / recall;
- free IoU;
- occupied IoU;
- macro IoU;
- MAE where meaningful.

## Stage analysis

- early;
- mid;
- late;
- last-10.

## Support

- evaluated GT cell count;
- evaluated free/occupied support;
- fraction of valid decision-time unknown region covered by the GT reference.

## Predictor comparison

- mean;
- G1;
- G2;
- G3.

## Direct P1–P2 comparison

For each run / stage where both references exist:

```text
P1 metric
P2 metric
difference
support_P1
support_P2
```

The purpose is not to require identical numerical values.

The purpose is to determine whether P1's qualitative conclusion survives the removal of the EventuallyObserved selection bias.

---

# 23. Decision rule after Gate P2

## Case A — P2 broadly agrees with P1

Examples:

- late prediction remains meaningfully informative;
- predicted-free performance is acceptable;
- no widespread collapse across runs;
- P1 optimism is not explained entirely by target selection.

Then:

```text
Gate P = PASS
↓
proceed to Gate U
```

## Case B — P2 is materially worse than P1

Then:

```text
Gate P = unresolved / fail
```

Before proceeding, diagnose:

- hallucinated free regions;
- hidden occupied structure;
- map-frame or ROI mistakes;
- environment-stage effects;
- individual member disagreement;
- whether only a restricted reachable subset should be evaluated.

Do not proceed to uncertainty-threshold tuning merely to compensate for a weak prediction foundation.

---

# 24. Gate U — planned only after Gate P

If P2 supports Gate P, the next research phase is Gate U.

Candidate uncertainty signals already declared in `ES.md`:

```text
U_p95
U_mean
U_disagreement
```

Gate U must test whether uncertainty relates to actual prediction error.

Minimum questions:

1. Does prediction error increase with uncertainty quantile?
2. Is rank correlation between uncertainty and error meaningful?
3. How frequent are:
   ```text
   low uncertainty + high error
   ```
4. Which uncertainty statistic is most stable across runs?
5. Does the relationship transfer to another environment?

No `T_uncertainty` should be frozen before this evidence exists.

---

# 25. Gate R — planned after Gate U

After prediction and uncertainty are supported, D1 must test whether predicted remaining area is actually a useful completeness signal.

Core per-member definition remains conceptually:

```text
R_j =
    Unknown_t
    ∩ PredictedFree_j
    ∩ Reachable_j
```

```text
A_j = |R_j| * resolution^2
A_mean = mean(A_j)
```

Candidate normalized version:

```text
RemainingFraction =
    A_mean
    /
    (KnownReachableFree + A_mean)
```

Gate R must compare these signals against actual future mapping gain.

Only after Gate R should D1 begin offline stopping-rule replay.

---

# 26. Current research state at the time this log was created

```text
D1 specification          DONE
D1 online STOP            NOT IMPLEMENTED

Gate P1 code              DONE
Gate P1 New Room run      DONE
Gate P1 verdict           PRELIMINARY PASS

Gate P2 structural GT     NEXT / NOT YET IMPLEMENTED

Gate U                    NOT STARTED
Gate R                    NOT STARTED
offline stop replay       NOT STARTED
threshold freeze          NOT STARTED
online D1 validation      NOT STARTED
```

## Immediate next action

```text
Extend d1_gate_p.py
with structural-GT reference support
and run P1 + P2 in the same analysis pipeline.
```

---

# 27. Open research questions

These questions remain intentionally unresolved.

## Q1 — Is the ensemble mean the final D1 prediction representation?

Not frozen.

P1 suggests the mean is strong, but no final decision is needed until P2/Gate U evidence is available.

## Q2 — Which uncertainty statistic should D1 use?

Not frozen.

Candidates:

- P95 variance;
- mean variance;
- ensemble binary disagreement.

## Q3 — Should completeness be absolute or normalized?

Not frozen.

Candidates:

- `A_mean_m2`;
- `RemainingFraction`.

## Q4 — Which reachability definition should be canonical?

Not frozen.

Candidates:

- raw 8-connected;
- footprint-aware.

## Q5 — What are the stopping thresholds?

Not selected.

Threshold tuning before Gates P/U/R is explicitly prohibited by the current research protocol.

## Q6 — What K-confirmation value should be used?

Not frozen.

K-confirmation will be evaluated only after the base stopping signals are justified.

---

# 28. Research integrity rules for future D1 work

The following rules should remain fixed unless a dated log entry explicitly changes them.

1. Do not use structural GT, final maps or future observations as runtime features.
2. Do not change MapEx frontier selection when D1 says CONTINUE.
3. Do not tune thresholds before validating their underlying signals.
4. Do not report accuracy without class-specific metrics.
5. Do not treat all spatial cells as independent experimental runs.
6. Keep run-level aggregation.
7. Report non-triggering runs, not only successful early stops.
8. Do not hide environment-transfer failures.
9. Keep uncertainty as a separate caution gate.
10. K-confirmation does not solve systematic confident hallucination.
11. Preserve failed experiments and negative results in this log.
12. Freeze method definitions before prospective online validation.

---

# 29. Template for the next diary entry

Append future entries using this structure.

```markdown
# YYYY-MM-DD — <short event title>

## Question

What are we trying to determine?

## Decision

What was chosen?

## Reason

Why was this choice made?

## Implementation

Files, commands, parameters, datasets and code changes.

## Evidence / results

Exact metrics and artifact paths.

## Interpretation

What the result does and does not support.

## Limitations / anomalies

Failures, data issues, bias, missing support, provenance problems.

## Status

PASS / PRELIMINARY PASS / FAIL / UNRESOLVED / IN PROGRESS.

## Next action

One concrete next step.

## Relevant commits

- `<sha>` — <message>
```

---

# 30. Short current summary

At the end of the first D1 Gate P1 cycle:

> MapEx/LaMa predictions show useful fidelity on later-observed unknown cells, and the first-later-observed target is sufficiently stable to keep. However, late-stage P1 evidence is strongly right-censored and biased toward cells the baseline still has time to observe. Free-space prediction near termination is also much more variable than headline accuracy suggests. Therefore P1 is only a preliminary pass. The next required experiment is Gate P2 using the corrected structural ground truth over the valid unknown region. Only if P2 broadly supports P1 should D1 move to Gate U.


---

# 31. 2026-09-21 — Gate P2 structural-GT implementation added

## Question

How should Gate P remove the major P1 selection bias caused by evaluating only cells that baseline exploration later observes?

## Decision

Extend the existing `d1_gate_p.py` rather than create a second metric script.

The CLI now supports:

```bash
--reference later_observed
--reference structural_gt
--reference both
```

Default:

```bash
--reference both
```

## P2 evaluation domain

P2 evaluates:

```text
Unknown_t ∩ StructuralGT.evaluation_mask
```

using the corrected:

```text
ground_truth/new_room/generated/new_room_structural_gt_v2.npz
```

The connected-free ROI is **not** used as the classification mask because that ROI represents free-space connectivity and would remove occupied-class targets. Gate P needs both free and occupied fidelity.

## Alignment semantics

The saved MapEx runtime prediction is first cropped to the exact decision-time raw map.

For P2:

```text
runtime map / prediction: 0.10 m
structural GT canvas:     0.05 m
```

Each runtime prediction/unknown cell is nearest-neighbour expanded onto the canonical GT canvas, matching the repository's existing evaluation reprojection semantics.

Only cells that are:

```text
unknown at t
AND
inside StructuralGT.evaluation_mask
```

are scored.

## Output format

When using `--reference both`:

- `gate_p_decisions.csv` contains one row per decision/reference;
- `gate_p_runs.csv` contains one row per run/reference;
- `gate_p_summary.json` stores separate `references.later_observed` and `references.structural_gt` blocks;
- `p1_vs_p2` reports ensemble-mean P2-minus-P1 differences for overall and late accuracy / macro IoU / MAE;
- figures are prefixed with the reference name.

Because P1 uses 0.10 m runtime cells while P2 evaluates on 0.05 m canonical cells, raw cell counts must not be compared directly. The analyzer therefore also reports evaluated area in square metres.

## Evidence / results

Implementation completed.

No P2 numerical result has been claimed yet. The updated analyzer still needs to be executed against the local binary NPZ artifacts.

## Status

```text
Gate P1 = PRELIMINARY PASS
Gate P2 code = DONE
Gate P2 experiment = PENDING
Gate P = NOT YET COMPLETE
```

## Next action

Run:

```bash
python3 mapex_lab/analysis/d1/d1_gate_p.py --reference both
```

on `mpx_001...mpx_010`, then compare P1 and P2 before moving to Gate U.

## Relevant commits

- `0d9559322fe054f69d61d47c276a1e8721322ba4` — add structural-GT reference to D1 Gate P.
- `e73d753ae1e5c21fecabd35cc367326cceed2825` — document D1 Gate P structural-GT modes.


---

# 32. 2026-09-21 — Gate P2 provenance and comparison hardening

## Decision

Before the 10-run P1+P2 execution, harden two parts of `d1_gate_p.py` without changing Gate-P methodology or threshold.

### Structural-GT provenance guard

For P2, validate per run:

```text
environment compatibility
structural_ground_truth_id
structural_ground_truth_file
fixed_canvas_id
fixed_canvas_resolution_m
runtime_map_resolution_m
decision raw-map resolution consistency
runtime/GT integer resolution ratio
```

Missing legacy metadata fields produce WARN. Present fields that contradict the selected GT/canvas/runtime artifacts produce a hard failure.

### P1-vs-P2 comparison

The `p1_vs_p2` block now compares ensemble-mean P2-minus-P1 for overall and last-10:

```text
accuracy
macro IoU
MAE
free precision / recall / IoU
occupied precision / recall / IoU
```

Support is recorded globally and by run, including target counts, free/occupied support, evaluated area and fraction of unknown area. Because P1 and P2 use different cell resolutions, evaluated area/fraction is preferred over direct raw-cell-count comparison.

## Threshold note

Gate P remains:

```text
prediction < 0.5  -> free
prediction >= 0.5 -> occupied
```

The legacy evaluator uses `>0.5` for occupied; this difference is documented and affects only the exact boundary value.

## Status

```text
P2 code hardening = DONE
10-run P1+P2 execution = PENDING
Gate P = NOT YET COMPLETE
```

## Next action

Smoke test one run with `--reference both --no-figures`, then run `mpx_001...mpx_010`.

## Relevant commit

- `614f5e96ad553815d4bda27916f9966cd911863d` — harden D1 Gate P2 provenance and P1/P2 comparison.


---

# 33. 2026-09-21 — Exact prediction-to-raw integrity checks

## Decision

Before the P1+P2 cohort run, strengthen artifact identity checks and relax only environment naming.

### Environment provenance

`metadata.environment` mismatch is now **WARN**, not FAIL. Environment naming is descriptive and may vary historically.

Hard failures remain for contradictions in:

```text
structural_ground_truth_id / file
fixed_canvas_id / resolution
runtime map resolution
runtime-to-GT resolution compatibility
```

### Prediction-to-raw identity

Each saved G1/G2/G3/mean NPZ must match the exact decision raw map on:

```text
source_height / source_width
resolution
origin_x / origin_y
source_map_stamp_s
member identity
```

The raw-map `source_stamp_s` and prediction `source_map_stamp_s` come from the same source OccupancyGrid when recorded, so timestamp mismatch is treated as a hard artifact-association failure.

Prediction environment metadata is warning-only.

## Why

Shape alone cannot detect a stale or wrong-decision prediction when adjacent decisions share the same grid extent. Source timestamp plus spatial metadata gives a much stronger association to the exact raw map.

## Status

```text
prediction/raw integrity guard = DONE
environment naming policy      = WARN-only
10-run P1+P2 execution         = PENDING
```

## Next action

Run one-run smoke test, then the full `mpx_001...mpx_010 --reference both` cohort if clean.

## Relevant commit

- `d522ab56582f2eec055a748d4f0968b35b175157` — verify D1 Gate P predictions against exact raw maps.
