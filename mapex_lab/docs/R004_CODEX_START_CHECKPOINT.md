# R004 CODEX start checkpoint

Date: 2026-09-23
Task: R004-OFFLINE / H037
Owner: CODEX maker
Criticality: HIGH; independent review required
Status: ACTIVE — implementation not yet complete

## Frozen authority

- Management method: `R004_METHOD_PROPOSAL_V3.md` (independently ACCEPTED).
- Technical base: `867337e69daa8146e08238b952347c5bca05c389`.
- Task branch: `r004-offline-evaluator`.
- No R002/R003 methodology change and no new simulation are authorized.

## Inputs

- Historical cohort: `mpx_001..mpx_010`.
- Read-only data checkout: `/home/dell/turtlebot4_project_r003_p500_backup`.
- Data checkout HEAD at pickup: `cff8ed1fefdfefc960401fe8651726a9513c48cf`.
- Canonical per-run inputs: `decisions.csv`, saved ensemble-mean prediction
  NPZ files, decision-time observed canvases, and `snapshots.csv` final observed
  canvas under the existing fixed-canvas semantics.

## Planned implementation and validation

1. Inventory all ten runs and stop on irreconcilable provenance/layout gaps.
2. Implement the V3 evaluator under `mapex_lab/analysis/r004/`.
3. Add focused synthetic tests for domain membership, strict threshold,
   undefined metrics, final-snapshot selection/fallback, normalized progress,
   deterministic resampling and run-macro weighting.
4. Run compile/tests and a dry-run cohort inventory.
5. Execute bounded offline scoring only on the ten historical runs.
6. Verify deterministic subset rerun, counts, exclusions/fallbacks and figures.
7. Commit code, tests, compact result tables/figures and completion evidence.

Expected result root:
`mapex_lab/analysis/r004/results/prediction_vs_final_observed_v1/`

## Interruption/resume rule

Inspect this branch, `git status`, the latest R004 commit and the expected
result root before rerunning anything. Partial outputs are not scientific
evidence until the completion checkpoint records validation and exact counts.

## Next gate

After CODEX delivery, hand code and outputs to Chat 2 / W016 / CHAT-CRITIC for
independent implementation and result-integrity review. Scientific
interpretation by Chat 1 occurs only after that review.
