# D1 W027 Gate-U Offline Scoring Completion

Status: **COMPLETE_PENDING_REVIEW**  
Owner: **W027 / CODEX**  
Checker: **W016 / Chat 2 / CHAT-CRITIC**  
Authority: **H074 + H068 Gate-U Method V2**  
Policy epoch: `2026-09-24-D047`

## Exact identities

- Technical base: `15092f781c623c3785588674f4ab9be92214fef7`
- Branch: `d1-gate-u-w027`
- Shared evidence: `analysis/d1/results/shared_phase0_evidence_v1/shared_phase0_evidence.csv`
- Shared evidence SHA-256: `993bf0a0e9f1aabfd240778aadd9f6cb34583059ba360cea68fefef97ea2660a`
- Cohort: 10 runs, 365 preserved decision rows
- Result root: `analysis/d1/results/gate_u_v1/`

## Execution evidence

- Focused tests: **18/18 PASS**.
- First real fold smoke: `holdout_mpx_001`, production path, completed successfully.
- Resume validation: the full invocation verified and skipped the completed first fold before processing folds 2–10.
- Completed folds: **10/10**.
- Per-decision result rows: **365**.
- Candidate selected by every fold: `U_p95`.
- No simulation, LaMa inference, shared extraction, or accepted R004 evidence was rerun or changed.

## Frozen historical result

Gate-U historical outcome: **FAIL**.

Exact precedence reason:

`strong broad contradiction veto`

The veto fired in exactly 6 distinct held-out runs:

- `mpx_002`
- `mpx_003`
- `mpx_006`
- `mpx_007`
- `mpx_009`
- `mpx_010`

Other required aggregate diagnostics:

- candidate selection: 10/10 folds;
- held-out runs with at least 5 finite primary pairs: 10/10;
- valid `SRR50_run`: 10/10;
- runs with low-U primary support for CW25: 6/10;
- general all-stage sufficiency: PASS;
- median primary rho: `0.290602036000809`;
- positive-rho runs: 8/10;
- mean SRR50: `0.816416833270123`;
- mean CW25 rate: `0.027777777777777776` over 6 defined runs;
- severe confidently-wrong events: one event in one run (`mpx_010`, decision 15);
- strict-late primary support: `LATE_PRIMARY_INSUFFICIENT` (0 runs meet the >=3-row contribution rule; only `mpx_001` and `mpx_002` have one finite late primary row each).

Descriptive 10,000-run-bootstrap 95% percentile intervals, seed `20260924`:

- median primary rho: `[0.017132905793896494, 0.6156024284846813]`;
- mean SRR50: `[0.6097197114359637, 1.0161607873551433]`;
- mean CW25 rate: `[0.0, 0.08333333333333333]`.

## Review package

- `results/gate_u_v1/preflight.json`
- `results/gate_u_v1/tests.log`
- `results/gate_u_v1/execution.log`
- `results/gate_u_v1/batch_state.json`
- `results/gate_u_v1/folds/*/completion_manifest.json`
- `results/gate_u_v1/final/gate_u_summary.json`
- `results/gate_u_v1/final/per_fold_results.json`
- `results/gate_u_v1/final/per_decision_results.csv`
- `results/gate_u_v1/final/completion_summary.json`
- `results/gate_u_v1/final/artifact_manifest.json`

## Boundary and next action

This is an executor result, not a canonical interpretation by itself. W016 / Chat 2 must independently review the implementation, fold identities, frozen-rule fidelity, hashes, and result classification.

Gate R remains parked. No operational uncertainty threshold, online stopping rule, or confirmation count was selected.
