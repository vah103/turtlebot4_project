# MX071 follow-up R1 — bounded proposal corrections
Date: 2026-10-10. USER authorized a MapEx rerun and a separate investigation of PIPE; selected **all remaining open questions**, not only the three MX071 core questions. Journal #7 remains DROPPED_BY_USER.

## Provenance and boundaries
The original eight ideas concern possible omissions in MapEx (Agent1 journal §17). PIPE verification remains a separate requirement (§16, §17.5). MX071 R2 measured PIPE_ALIGNED source states, first-goal interventions, and common PIPE continuation; its results are preserved unchanged. A MAPEX_ALIGNED first goal in those branches is not a native MapEx trajectory.

This is a **bounded R1 proposal correction**, not a sealed execution protocol or an independent methodology/result ACCEPT. R0 remains unchanged in PLAN_R0.md at a3e08eef57d4871c56370e67b0c96a6a42995100. IR1's R0 verdict is REVISE; the four maker corrections below still require independent closure. Do not inherit the accepted R2 method for the additional questions, native controller, or new branch ceiling. Do not alter MX072 on COM1, its implementation/review, or its New Room world.

## Seven open questions and evidence required

| Journal question | Direct MapEx diagnostic | Evidence needed for a task-level conclusion |
| --- | --- | --- |
| #1 aggregation | Source pointwise probabilistic masks from mean versus mean of three member masks, frozen U, distance, candidates and frame | First-goal pair with identical **MapEx** continuation, including unchanged actions/zeros; extra raycast compute |
| #2 observation-conditioned global prediction update | Before/after G1/G2/G3, ensemble mean and variance, probabilistic visibility, variance mass, score/rank and candidate/action disagreement on common still-unknown support, excluding the new sensor observations; G/all-train is auxiliary | No weights change. Distinguish inference change from accuracy improvement and actual action change. Shadow rescoring is not native rescoring; a later expected-update selector and matched MapEx continuation must be separately sealed |
| #3 topology/connection | False/missing walls and access/connectivity changes on the correct source frame; preserve previous negative structural pilots | Bounded structural repair versus same-size route-error/simple controls, action difference, whole-budget utility and travel/compute. GT repair is oracle-only |
| #4 gain/visibility error | Same source endpoint and renderer: prediction versus matched sensor-truth substitution, frozen U; separately audit physical sensing/frame mismatch | Matched first-goal oracle pair, common MapEx continuation; no claim of recoverable online benefit |
| #5 goal commitment | Separate OBSERVATION_CONTROL_STATE nominees: post-scan, source goal still valid, new observation received since the lock's score input and no native rescore after that observation; retain lock/cache and compare a labelled fresh shadow rescore | Precommitted one-time fresh rescoring during a still-active goal versus unchanged source commitment; include early goal releases/zeros, compute and additional travel |
| #6 cluster representative | Native representative and a bounded set of observed-geometry alternative poses, same sensor/scorer and planning validity | Representative intervention versus source, with common MapEx continuation; pool expansion is explicit and never mixed with fixed-pool #1/#4 |
| #8 future positioning | Realized 10m gains and remaining-budget Q for distinct tested executable actions on a MapEx source snapshot | Report strict reversal separately from tied short-horizon gains. Best-among-tested regret is not global optimum or proof of a deployable lookahead policy |

No diagnostic-only row may be reported as a causal policy win. All seven remain in scope, but they do not become seven scientific conclusions from one rerun.

## Acquisition foundation
- DELL remains the requested machine. Reuse four sealed KTH map identities and two sealed starts each as the first cohort; retain exact map/start hashes and MAP_ID_ONLY / TRAIN_OVERLAP_UNVERIFIED. Do not imply building-level independence, unseen training support, or that KTH and New Room are equally complex.
- Pin MapEx 53636bd1c79153acc3c74a532837d78c926bae5e and LaMa b61dcb33e063fe9586b50e2dc7b70f97d5046c1d; validate every used source file and all four model identities.
- Execute the pinned source action loop headlessly, retaining original endpoint scorer, padding/origins, effective unknown-as-occupied false, four-connected A*, goal lock/reselection, controller path index 3, and physical 2500 rays.
- Sequential CPU model residency and removal of plotting/incidental output are explicit implementation adaptations; do not silently replace source behavior with MX071's one-cell, cropped-frame, unknown-blocked controller.
- Preserve a 100m diagnostic prefix with **LAST_OBSERVED_STATE_AT_OR_BEFORE_BUDGET** as primary; retain the preceding complete post-scan state if a discrete step crosses 100m. Record actual endpoint distance/overshoot and any endpoint scan separately as POST_CROSSING_ENDPOINT sensitivity. Section C fixes the source-order and paired-Q rule. This is a source-control distance-prefix adaptation, not the original paper benchmark or proof of full exploration.
- Maintain two separately labelled checkpoint strata under section B: NATIVE_SCORE_EPOCH at the first source-score event at/after each 20/40/60/80m target within budget; OBSERVATION_CONTROL_STATE at the first eligible active-lock post-scan event in each precommitted target window. Eligibility never uses prediction, GT, rank disagreement or utility. Aliases and unavailable targets remain explicit.
- Baseline acquisition is not a substitute for paired continuations. New branches require a separate full-state replay seal, selectors, commitment rules, reuse key, resource bound and declared ceiling. The previous 80-request ceiling is not automatically a ceiling for seven questions.
- Source data may only start after baseline/source parity, model/source identity and resource gates pass. Additional paired policies remain closed until their own contracts and replay checks are implemented.


## A. Question #2: the ensemble state used by visvarprob

Mechanism label: **OBSERVATION_CONDITIONED_GLOBAL_PREDICTION_UPDATE**. Model weights are fixed; this measures inference conditioned on newly observed cells, not online weight learning.

The pinned source computes G1/G2/G3 channel-0 predictions, NumPy ensemble mean and Torch K=3 variance; probabilistic raycasting on the mean replaces the preliminary G/all-train visibility. The final visvarprob value is visible-unknown variance mass divided by Euclidean candidate distance. Preserve the source tensor/frame/threshold/ordering semantics, including the source's cost sign. G/all-train remains an auxiliary channel and its actual source computation/cost is retained; it is not the sole direct #2 evidence.

At each nominated observation state, retain the immediately preceding complete observation input, post-scan input and actual sensor-observation mask. Define common still-unknown support in the pinned source frame as cells unknown in both inputs and outside the newly acquired sensor observations. Record GT-valid support separately for error evaluation; padding/non-GT cells cannot become truth labels. Empty support is explicit NA, unchanged input is an explicit zero-change case.

For this common support record before/after member predictions, mean, variance, probabilistic visibility masks, visible variance mass, cost/score/rank and candidate/action disagreement. Freeze pose, candidate IDs/coordinates, Euclidean distances, source ray settings and support within this diagnostic pair; retain invalid/nonfinite/tie cases. Do not freeze variance across #2: its change is part of the mechanism. A common-support diagnostic score is labelled **MATCHED_SUPPORT_SHADOW**, rather than an actual native score.

At observation-control nominees, fresh before/after inference and rescoring are shadow work on copies: the source itself did not rerun its models there. Record the actual native cached outputs, their score-epoch/input IDs and locked goal separately. At native score epochs, preserve actual source full-support scores without relabelling the matched-support diagnostic as native. Prediction change, outside-view accuracy change, ranking change and executed action change remain separate outcomes. This R1 defines the diagnostic scope; it does not implement inference or seal an expected-update action selector.

## B. Two event strata and outcome-independent nominations

A shared lightweight event ledger must identify source iteration, successful pose/scan generation, cumulative successful endpoint-chord distance, score-epoch/input generation, lock ID, native rescore/reselection/release events and termination status. Full immutable snapshots are required for nominated strata plus the immediately preceding observation input needed by #2; exact complete-state fields/lossless reconstruction remain an execution gate.

| Stratum | Event and precommitted nomination | Interpretation |
| --- | --- | --- |
| NATIVE_SCORE_EPOCH | Source has just computed its native prediction/pool/scores and is about to select a new goal. For targets 20/40/60/80m, nominate the first such event at distance >= target and <=100m; retain initial native score provenance too. | Actual source scoring. Several targets may alias one exact state; aliases do not add independent support. No qualifying event is NA. |
| OBSERVATION_CONTROL_STATE | Immediately after a successful endpoint scan/accumulation, before the next source control iteration can release/reselect/rescore. Nominate the first eligible event within [20,40), [40,60), [60,80), and [80,100]m respectively. | Fresh shadow predictions/scores are diagnostic only; preserve the actual native lock, cached ranking and model outputs. |

Observation-state eligibility is bookkeeping-only: a lock exists; it remains valid under the exact next-iteration source validity/reached-goal rules evaluated read-only on the captured map/pose; scan generation is newer than the lock's scoring-input generation; and no source rescore has used that newly arrived observation. Eligibility does not require nonzero new area or a favourable shadow rank change. A fresh scan with an unchanged map is retained as a structural-zero diagnostic. If the window contains no eligible state, keep NA with the reason; never substitute a later favourable state.

Identify last native score/input/lock IDs and pending next-control position explicitly. Preserve post-scan mapper internals/history, goal/cache/pool/cost state and the preceding observation input; a map-plus-pose snapshot alone is not certified replay state. New score events, ordinary unlocked observations, invalid/reached goals and active-lock observations are distinct ledger categories. A read-only eligibility check must not release the goal or call a native rescore.

#5's later paired intervention is still one precommitted fresh-rescore/retarget opportunity versus keeping the source commitment, then common MapEx continuation. It needs its own candidate-pool, revalidation, commitment duration and failure/zero rules before branching. A shadow disagreement at a nominee does not prove retargeting helps. #2 uses the same observation nominees where valid, so acquisition is shared.

## C. Exact 100m source-order and Q convention

Let S_i be a complete state after the initial scan or a successful native endpoint scan. Let d_i be cumulative chord distance between successfully appended poses, in metres. These are actual discrete source endpoints; d_0=0. The source order is plan next_pose -> set pose -> center-cell collision check -> append successful pose -> scan/accumulate.

Primary at B=100m is **LAST_OBSERVED_STATE_AT_OR_BEFORE_BUDGET**: choose the last complete S_i with d_i<=B. If d_i<B<d_(i+1), use S_i's map **and S_i's actual pose/control state**, rather than mixing the old map with the crossing pose or inventing a state exactly at B. An endpoint exactly at B includes its successfully completed scan.

A crossing step, if executed for boundary bookkeeping, completes in source order. Its endpoint S_(i+1) is a separate **POST_CROSSING_ENDPOINT** sensitivity, not primary 100m data and not a fork at B. Record d_i, B-d_i, d_(i+1), d_(i+1)-B, crossing-step travel/time and actual total executed distance. Native control-step limits/early termination still apply; do not extend source execution just to obtain a crossing. If the crossing was not executed, record predicted crossing distance separately and do not call it actual overshoot.

For primary distance-indexed coverage, use the last completed observation at or before each x:
`C_B(x) = C(S_last), last = latest completed scan with d_last <= x`.
Thus between successful scan endpoints coverage is held at the previous complete observation; there is no interpolated pose, partial scan or credit for an endpoint beyond x. For duplicate distances use the latest successfully completed observation at that distance. This formula, rather than ambiguous continuity wording, defines the metric.

For a successfully evaluable pair starting from the same actual snapshot at distance d_s<B, both arms use the same B and R=B-d_s:
`Q(s,a) = (1/R) * integral_[0,R] (C_a(d_s+u)-C(s)) du`,
with the endpoint-hold rule above applied independently and identically to their valid scan traces. The #8 10m-prefix diagnostic uses that same rule at its shorter cutoff; report actual last endpoint and any short-prefix gap.

A collision/failed step with no successful pose append/scan creates no new observed endpoint; zero movement creates no invented distance or observation. Apply the same last-observation boundary rule in all arms. Natural terminal carry-forward remains a labelled trace convention; technical/infrastructure failure cannot silently be converted into an evaluable full-budget Q. Final failure/NA/penalty and denominator rules must be sealed before execution; retain every failed/zero/unchanged request rather than dropping inconvenient outcomes. The boundary convention does not itself certify those remaining execution rules. POST_CROSSING_ENDPOINT observations and unequal overshoots cannot be mixed into primary paired Q.

## D. Remaining execution gates after this bounded R1

These four corrections are proposal semantics, not an implemented recorder or sealed seven-question batch. Before any model probe/collection/paired policies, return the exact proposal to IR1 and separately complete the required methodology/runtime scope: cohort/overlap boundary; per-question executable interventions/controls; full-state capture/replay and common MapEx continuation; reuse keys, failures/zeros/NA; a new explicit branch ceiling; real G/G1/G2/G3 parity/resource probe; lossless storage and total-time bounds.

Share acquisition/replay across questions where valid. Do not borrow R2's 80-request ceiling, its old failure/control engine or MX072 acceptance. PIPE remains its own audit-first stream in PIPE_AUDIT_R1.md, with at most one precisely selected hypothesis promoted through support -> action mismatch -> paired utility -> online detectability -> cost.

## Implementation sequence
1. Native baseline/source parity and lossless state capture; record read-only source facts and resource feasibility.
2. Freeze and test common **MapEx** continuation and full-state clones. CORE #1/#4/#8 can share this verified foundation.
3. Add specific bounded tests for #2/#3/#5/#6 with their controls. Do not manufacture an expected-model-update or structural selector from post-hoc winning cases.
4. PIPE audit is independent: audit what is already handled before selecting any new scientific test. Any later PIPE batch needs its own stated hypothesis, bounded method and technical preflight; none has been started.

## Current status
BOUNDED_R1_CORRECTIONS / PENDING_INDEPENDENT_CLOSURE / EXECUTION_UNSEALED. No new scientific source/branch result at document creation. Original MX071: 8/8 sources and 75/75 logical requests complete, results scoped PIPE_ALIGNED. Its prefix-regret screening requires the documented tie qualification.
