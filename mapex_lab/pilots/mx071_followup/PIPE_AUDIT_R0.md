# PIPE source and paper audit R0
Date: 2026-10-10. Read-only mechanism audit, not a weakness or novelty verdict.

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
| Gain beyond remaining travel budget | A full candidate path is evaluated; scoring signature receives the current step but does not itself receive a remaining-distance budget | Same-pool full-path versus reachable-prefix scoring near budget exhaustion, common source continuation, keep zeros; show frequency and utility, not merely a score difference |
| One candidate path per frontier | explore.py obtains an A* path for each frontier and evaluates that path | A bounded alternative-path control at matched travel cost, same goal/pool, realized task benefit and computation. Absence of path search alone is not proof of harm |
| Positioning after reaching a frontier | Algorithm 2 evaluates the next frontier path; it does not expose a future continuation-value estimate in that score | Strict short-vs-long rank reversals under unique prefix winner, not tie-breaking regret. MX071 currently provides only tied-prefix divergence, so this candidate is not established |

These are audit-derived hypotheses, not claims that the authors missed them or that they are novel. Confirm the actual configuration and execution traces before promoting any.

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
Choose one precise failure hypothesis, an executable source baseline, a minimal matched intervention, frequency/support accounting, and whole-budget utility plus compute/travel cost. If only oracle choices help, report oracle headroom. Do not start a wide collection merely to seek a positive result.
