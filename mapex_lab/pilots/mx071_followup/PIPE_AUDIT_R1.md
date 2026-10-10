# PIPE source and paper audit R1 — bounded budget correction
Date: 2026-10-10. Read-only mechanism audit, not a weakness or novelty verdict. R0 remains unchanged for review history; this R1 separates native source step limits from the researcher-defined 100m distance prefix.

## Exact sources
- Paper v1: https://arxiv.org/html/2503.07504v1 (§IV-A–E, Algorithms 1/2).
- Official repository pin: castacks/pipe-planner@e5bcb5ec4a9a13cbe14aa9f7850bc8b5989a2bb0.
- Read scripts/explore.py, scripts/sim_utils.py, configs/base.yaml and the official README.
- Original MapEx pin: castacks/MapEx@53636bd1c79153acc3c74a532837d78c926bae5e.

## Mechanisms already addressed
1. Path gain: evaluate sensor coverage along the A* route, not only the frontier endpoint.
2. Repeated observation: union path visibility rather than summing overlapping masks.
3. Predicted obstruction: probabilistic raycasting with the inherited 0.8 threshold; prediction-related over/under-estimation is not an untouched problem.
4. Computation: polygon union/single fill and frontier parallelism. DELL's serial aligned implementation is not the paper's performance benchmark.

## Candidate checks, all UNVERIFIED as task-level weaknesses

| Candidate | What exact code/paper supports | What must be measured before claiming weakness |
| --- | --- | --- |
| Residual map/visibility error, especially low-variance wrong agreement | Variance-weighted path visibility depends on ensemble predictions; probabilistic rays mitigate some errors | Source CODE_REFERENCE matched oracle, preserve U/path/pool/samples and zero action differences; classify error types and measure closed-loop loss. MX071's +2.24pp Q is frozen-U headroom in PIPE_ALIGNED, not native proof |
| Cached goal value after new observations | New score epochs occur when the locked goal is invalid/reached; no-path reselection can use the existing ranked pool | Frequency, fresh-score disagreement while original goal remains valid, paired retarget utility and its inference/turning cost. Do not say PIPE never replans: local A* and invalidation/reselection exist |
| SOURCE_STEP_BUDGET_AWARENESS | Source scoring receives t but does not clip sampled path support to a remaining native control-iteration count; the actual loop limit must be resolved from source/config | Test support unreachable within the remaining effective source iterations using source controller/scan cadence, same pool/path/U/renderer and common native continuation. Step-to-path reachability must be validated, not guessed from path sample count or 100m |
| DISTANCE_PREFIX_BUDGET_AWARENESS | The source scorer receives no remaining-distance budget; a 100m prefix is our additional evaluation contract | Same-pool/path full-support versus distance-reachable-prefix scoring under the explicitly declared 100m adaptation, with the same boundary rule/common continuation and zeros. Positive results apply to that adaptation, not an omission of PIPE's original source budget |
| One candidate path per frontier | explore.py obtains an A* path for each frontier and evaluates that path | A bounded alternative-path control at matched travel cost, same goal/pool, realized task benefit and computation. Absence of path search alone is not proof of harm |
| Positioning after reaching a frontier | Algorithm 2 evaluates the next frontier path; it does not expose a future continuation-value estimate in that score | Strict short-vs-long rank reversals under unique prefix winner, not tie-breaking regret. MX071 currently provides only tied-prefix divergence, so this candidate is not established |

These are audit-derived hypotheses, not claims that the authors missed them or that they are novel. Confirm the actual configuration and execution traces before promoting any.

## Two budget domains: record configured and effective limits

The primary source/control-step budget is independent of wall-clock seconds and of distance. Pin its exact loop-index/control/scan order. Record source t, executed iteration count, configured mission_time, effective loop limit N_native and remaining iterations at each score event; retain the source's off-by-one/index semantics.

An additional exact-pin guard matters here: PIPE explore.py defines time_settings by map category and log_iou (large 6001/2001, small 1501/501, medium 3001/1001), assigns time_step, and runs `for t in range(time_step)`. The base YAML's mission_time=1000 alone therefore does **not** certify this pin's effective loop limit. Do not silently substitute configured mission_time, sampled-pose count or 100m for N_native. Record the resolved category/file identity and log_iou branch; any modified fixed mission_time loop is an explicit adaptation.

At loop entry t, distinguish N_native-t iterations including the current iteration from iterations remaining after it. Only source-order/controller reachability checks can establish which scored scan poses are still attainable; path[2::3] sample indices are not automatically control-step or scan indices. Native failure/early termination/reselection must remain source behavior.

For DISTANCE_PREFIX_BUDGET_AWARENESS, record distance B=100m and snapshot d separately; evaluate the proposed prefix with PLAN_R1 section C's LAST_OBSERVED_STATE_AT_OR_BEFORE_BUDGET convention. That is a diagnostic adaptation, not the original PIPE benchmark or mission objective. No 100m effect may be reported as evidence that the authors ignored their native step budget.

These are two possible definitions of **one budget-awareness hypothesis family**, not authorization to run both or expand a batch. Audit/support must select at most one precise test before implementation. Both require same candidates/path/U/renderer, non-favourable cases and unchanged actions, matched continuation and compute/travel costs. Neither omission alone establishes task harm.

## Implementation discrepancies to keep separate
- Public helper takes unknown_as_occ but calls inflate_map with False. This differs from the ALIGNED_SHARED adaptation.
- Public path sampling is path[n-1::n]; with n=3, path[2::3].
- Public denominator is max(1, sampled-pose count), while the paper describes path-length normalization. Do not quietly change it and still call the run source PIPE.
- Source polygon/MultiPolygon/fill edge cases require technical classification and renderer parity; a code defect is not automatically a thesis direction.

## Evidence from completed MX071
- Aggregation P1: macro ΔQ = 0.0003783782707618118; no qualifying improvement in that pilot.
- Visibility P4: macro ΔQ = 0.022408133008130098; positive means in 3/4 maps; some points negative or zero.
- H8: all six positive tested-prefix-regret cases have tied 10m prefix winners; four eligible unique-winner points have no strict rank reversal.
- These results concern PIPE_ALIGNED, fixed-U offline first-goal tests, and common PIPE continuation. No demonstrated practical new policy, native PIPE weakness, universal robustness or inferential significance.

## Next scientific gate
Choose at most one precise failure hypothesis through mechanism gap -> frequency/support -> action mismatch -> paired end-to-end utility -> online detectability -> cost; seal its source baseline, minimal matched intervention and evaluation budget first. If only oracle choices help, report oracle headroom. Do not start a wide collection merely to seek a positive result.
