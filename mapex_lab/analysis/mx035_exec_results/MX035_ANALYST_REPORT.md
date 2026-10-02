# MX035 Analyst04 retrospective result candidate — executed under MX036

Status: COMPLETE_PENDING_INDEPENDENT_RESULT_QA

Frozen Method V2: 5842dd9a6fb34bb6266904bdd943dfdc514db38b

Result classification: **MX035_TEMPORAL_RECHECK_NO_STABLE_CANDIDATE**

## Outer LORO
- solved folds: 0/10
- held-out fired: 0/10
- held-out positive-saving: 0/10
- held-out premature: 0
- held-out SevereFalseStop10: 0
- median non-premature delay: NA decisions
- mean non-premature delay: NA decisions

## Full-fit descriptive
- status: NO_ADMISSIBLE_MX035_FULL_FIT
- q_full: undefined

## Frozen gates
- S1: **FAIL** — 0/10
- S2: **PASS** — premature=0;severe=0
- S3: **FAIL** — fired=0;positive=0;median_delay=NA
- S4: **FAIL** — full_status=NO_ADMISSIBLE_MX035_FULL_FIT;q_full=None;exact_q=0;theta_range=nan;J_range=nan
- S5: **PASS** — blob_fail=0;value_fail=0;base_parity_fail=0;monotonicity=0
- S6: **PASS** — adversarial=11/11

## Integrity
- 365-row fixed-base parity failures: 0
- source blob/value parity failures: 0
- temporal-before-base monotonicity violations: 0
- adversarial semantic audits: 11/11 PASS

Retrospective S2 is only a non-regression check because the frozen 5%/K1 fixed base
already has zero in-sample premature stops on these studied runs.

No retuning, Hospital, MX028, prospective data, engineering, deployment or robot STOP
is authorized or performed.
