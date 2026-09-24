# R004 CODEX start checkpoint

Date: 2026-09-23
Task: R004-OFFLINE / H037
Owner: CODEX maker
Criticality: HIGH; independent review required
Status: ACTIVE — V4 support amendment independently accepted; implementation resumed

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

## Pickup blocker discovered

The frozen domain is `UnknownAtDecision ∩ KnownInFinalObserved`, but each
historical prediction is saved only over that decision's dynamic raw-map
footprint. The fixed canvas contains cells outside that footprint which are
unknown at the decision and known in the final snapshot, but have no saved
decision-time prediction value.

Across `mpx_001..010`, 199 of 365 decisions have this mismatch. The frozen
domain contains 14,589,701 decision-cell pairs, of which 4,170,941 (28.5883%)
have no prediction support. For `mpx_001` decision 1, 197,712 cells belong to
the frozen domain but only 63,516 have saved prediction support.

Silently intersecting with prediction support, excluding affected decisions,
or inventing predictions outside the saved footprint would each change the
frozen methodology and the progress/support selection process. H037 explicitly
requires CODEX to stop rather than improvise when V3 cannot be implemented
exactly. No scoring code or result was produced.

## Resume checkpoint — accepted V4 amendment

H039 reopened this task after Chat 1 froze V4 and Chat 2/W016 independently
returned ACCEPT. Resume authority:

- target universe `F = UnknownAtDecision ∩ KnownInFinalObserved`;
- geometric prediction support `P`, independent of prediction values;
- scoreable domain `E = F ∩ P`;
- accuracy only on E; overall and class-specific support coverage relative to F;
- unsupported cells remain visible and are neither imputed nor errors;
- all other V3 progress, aggregation, sensitivity and provenance rules remain.

Resume begins from technical HEAD `bd7de0b734a85fa749beb900ec40605ea163993a`.
Usage snapshot at resume: NORMAL (72% five-hour and 33% weekly remaining).
Next milestone: focused evaluator tests pass before full ten-run scoring.

## H040 bounded rework checkpoint

Independent review of `b69d209a3b3854a038fa28087345209a063c66a4`
returned REVISE with exactly three bounded corrections:

1. place/validate support from prediction-artifact geometry and enforce the
   raw/prediction geometry provenance invariant;
2. validate final/fallback fixed-canvas artifacts, including corrupt/invalid
   final fallback and invalid-fallback rejection;
3. render actual run-macro support coverage and actual `1-C` unsupported
   fraction figures.

Usage at rework start: LOW (46% five-hour, 29% weekly remaining). SAVE-FIRST
applies. No V3/V4 rule, threshold, bin, sensitivity or interpretation change is
authorized. Next milestone: focused regressions pass, then rerun the same ten
historical runs and compare against the reviewed result set.

## H044 free-error diagnostic start checkpoint

Date: 2026-09-24
Task: R004-FREE-ERROR-OFFLINE / H044
Owner/session role: CODEX maker
Status: ACTIVE; HIGH criticality; independent W016 review required

- Branch: `r004-free-error-diagnostic`.
- Base/accepted source SHA: `550fa35a082041972133700f9688130bc73fd6f3`.
- Frozen authorities: accepted R004 V3+V4 and
  `R004_FREE_ERROR_DIAGNOSTIC_METHOD_V3.md`.
- Base decisions/runs/summary hashes were verified byte-for-byte against H044.
- Usage at pickup: HIGH_PRESSURE (99% five-hour remaining, 26% weekly
  remaining); SAVE-FIRST applies and no reset credit is used.
- Input: read-only historical `mpx_001..mpx_010` under
  `/home/dell/turtlebot4_project_r003_p500_backup`.
- Planned code/tests: a distinct free-error evaluator and focused synthetic
  suite under `mapex_lab/analysis/r004/`; the accepted evaluator/results remain
  unchanged.
- Planned result/log root:
  `mapex_lab/analysis/r004/results/free_error_diagnostic_v1/`.
- Next milestone: synthetic tests for decomposition, threshold, support,
  Euclidean depth bands, run-macro aggregation, and deep-occupied sensitivity
  all pass before any full-cohort execution.
- Stop condition: if fixed-canvas occupancy semantics or Euclidean depth cannot
  satisfy the frozen method, record a blocker and do not improvise.
- Resume rule: inspect branch/HEAD, status, this checkpoint, tests, and the
  distinct result/log root before rerunning any command.
