# MX033 Analyst04 result candidate — executed under MX034 authorization

Status: COMPLETE_PENDING_IR1_RESULT_QA

Frozen Method V2: 86e17f061b3acee14b84d27da2f95b43c007e677

Result classification: **VETO_SAFETY_SIGNAL_ONLY_NO_STABLE_CANDIDATE**

## Outer held-out summary

- solved outer folds: 0/10
- held-out fired runs: 0/10
- held-out positive-saving runs: 0/10
- held-out premature stops: 0
- held-out SevereFalseStop10: 0
- median non-premature held-out delay: NA decisions
- mean non-premature held-out delay: NA decisions

## Frozen gates

- S1: **FAIL** — 0/10
- S2: **PASS** — premature=0;severe=0
- S3: **FAIL** — fired=0;positive=0;median_delay=NA
- S4: **FAIL** — full_status=NO_ADMISSIBLE_MX033_FULL_FIT;q_full=None;exact_q=0;tau_range=nan;c_range=nan
- S5: **PASS** — outer_parity_fail=0;full_parity_fail=0;source_fail=0;monotonicity=0
- S6: **PASS** — semantic_pass=7/7

## Full-fit descriptive status

- status: NO_ADMISSIBLE_MX033_FULL_FIT
- q_full: undefined

The full-fit result is descriptive only and cannot rescue outer failures.

## Integrity

- 365 primary decision rows.
- outer paired-base parity failures: 0
- full-fit 365-row parity failures: 0
- accepted-source blob/value parity failures: 0
- AV-DUAL-before-base monotonicity violations: 0
- mandatory semantic adversarial checks: 7/7 PASS

## Scientific boundary

This is New Room retrospective development evidence only. It does not authorize
Hospital, Engineer work, deployment, or robot STOP. P/U/MX028 were not used in
the recognizer. No threshold/grid/backbone/gate was changed after outcome
inspection, and no nearest-passing substitute was used.
