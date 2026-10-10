# Maker response R1 — four bounded R0 findings

Date: 2026-10-10. Candidate: PR #42, same preparation branch. This is the original drafting session's response, **not an independent verdict**.

## Exact predecessor and review

- R0 technical head: a3e08eef57d4871c56370e67b0c96a6a42995100.
- [IR1 R0 REVISE](https://github.com/vah103/chat-gpt/blob/63cd2ce00166284ad298d6b9fafd4b33c058a3c3/company/reviews/HO-20261010T052408Z-MX071-FOLLOWUP-R0-IR1-REVIEW__independent-research-qa.md).
- [Bounded return](https://github.com/vah103/chat-gpt/blob/46758a76ae0c827afec5a962ddf116aea0cbb629/company/operations/HO-20261010-MX071-FOLLOWUP-R0-IR1-REVISE-RETURN.md).
- PR review 5477802003. R0 PLAN_R0.md and PIPE_AUDIT_R0.md remain unchanged for exact review history.
- Current proposal: [PLAN_R1.md](PLAN_R1.md); current audit: [PIPE_AUDIT_R1.md](PIPE_AUDIT_R1.md).

Maker agrees with all four findings. The G-only #2 channel, score-only schedule, ambiguous boundary terminology and conflated budget wording were insufficient.

| Finding | Bounded R1 change | Requested focused closure |
| --- | --- | --- |
| MX071-FU-R0-1 | PLAN_R1 #2 / section A tracks G1/G2/G3, mean, variance, probabilistic visibility, variance mass and score/rank on common support; G auxiliary; fixed weights; native cache versus shadow clearly labelled | Is the direct evidence now relevant to visvarprob, without turning matched-support shadow scores or inference changes into native execution/learning claims? |
| MX071-FU-R0-2 | PLAN_R1 section B separates NATIVE_SCORE_EPOCH and OBSERVATION_CONTROL_STATE; first eligible active-lock scan in precommitted distance windows; outcome-independent nomination, actual lock/cache retained, missing/unchanged states explicit | Does the proposed schedule capture #5/#2 opportunities missed by score epochs, while remaining a proposal rather than an implemented complete-state recorder? |
| MX071-FU-R0-3 | PLAN_R1 section C uses LAST_OBSERVED_STATE_AT_OR_BEFORE_BUDGET, exact endpoint source order, actual pose/map pairing, separate crossing sensitivity and a common endpoint-hold Q/10m-prefix rule | Are boundary, actual overshoot, exact-endpoint, zero/collision and primary-versus-sensitivity semantics unambiguous in both arms? |
| MX071-FU-R0-4 | PIPE_AUDIT_R1 separates SOURCE_STEP_BUDGET_AWARENESS and DISTANCE_PREFIX_BUDGET_AWARENESS; records configured versus effective native loop limits; no source-budget claim from 100m | Are both budget domains and source reachability limits correct, without inheriting a weakness or author-omission claim? |

## Source verification used for this correction

Read-only exact official files were checked; no model or simulator was invoked:
- [MapEx explore.py](https://github.com/castacks/MapEx/blob/53636bd1c79153acc3c74a532837d78c926bae5e/scripts/explore.py): prediction only under need_new_locked_frontier; G1/G2/G3 mean/variance; discrete move, collision check, pose append, then scan.
- [MapEx sim_utils.py](https://github.com/castacks/MapEx/blob/53636bd1c79153acc3c74a532837d78c926bae5e/scripts/sim_utils.py): score_frontiers/get_frontier_val use probabilistic mean visibility, unknown mask, variance mass and Euclidean distance.
- [PIPE explore.py](https://github.com/castacks/pipe-planner/blob/e5bcb5ec4a9a13cbe14aa9f7850bc8b5989a2bb0/scripts/explore.py), [sim_utils.py](https://github.com/castacks/pipe-planner/blob/e5bcb5ec4a9a13cbe14aa9f7850bc8b5989a2bb0/scripts/sim_utils.py), [base.yaml](https://github.com/castacks/pipe-planner/blob/e5bcb5ec4a9a13cbe14aa9f7850bc8b5989a2bb0/configs/base.yaml): scoring does not clip to remaining steps/distance; the actual loop bound is category/log_iou-derived time_step at this pin, so configured mission_time alone is not a runtime-budget certificate. This is an exact-source clarification within finding R0-4, not a new weakness claim.

## Preserved PASS and scope

Preserve #1's frozen-U aggregation contrast; #3/#4/#6/#8 claim limits and required controls; source-fidelity intent; diagnostic/oracle/causal distinction; old PIPE_ALIGNED metadata and H8 ties; bounded stand-in fixture claims. Keep #7 DROPPED_BY_USER. PIPE remains audit-first and promotes at most one precise supported hypothesis; shared MapEx acquisition/replay is preferred.

R1 changes documents only. Python implementation, native_source_readiness.json, completed MX071 evidence and the source fixture are unchanged. No fresh fixture/model probe, scientific source, paired branch or COM1 action occurred. Old fixture checks are not evidence that R1's proposed strata or boundary logic have been implemented/tested.

The complete execution protocol remains **unsealed**: interventions/controls, full-state replay/common continuation, reuse/failure/zero/NA rules, new branch ceiling, full-model resource parity, lossless storage and time bounds still require their separate method/implementation work. A focused R1 proposal ACCEPT would not close these gates, certify runtime, authorize a batch or change existing MX071/MX072 verdicts.

Return exact R1 to IR1 for focused closure/regression of R0-1..4. REVISE returns finite maker findings; methodology support remains with RM and strategy/resources with PM/USER. No maker self-review, merge/adoption, READY or START.
