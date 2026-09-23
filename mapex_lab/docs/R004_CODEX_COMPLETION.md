# R004 CODEX offline evaluator completion

Date: 2026-09-23
Task: R004-OFFLINE / H039
Maker: CODEX / W017
Status: COMPLETE_PENDING_REVIEW
Method: accepted V3 plus independently accepted V4 support amendment

## Outcome

Implemented and executed the bounded offline evaluator on historical
`mpx_001..mpx_010`. No simulation, R002/R003 change, threshold search or
post-outcome methodology tuning occurred.

Branch: `r004-offline-evaluator`
Base: `867337e69daa8146e08238b952347c5bca05c389`
Result root: `mapex_lab/analysis/r004/results/prediction_vs_final_observed_v1/`

## Validation

- Python compile: PASS.
- Focused evaluator suite: 10/10 PASS.
- Dry-run inventory: 10 runs / 365 decisions.
- Full execution: exit 0; all ten runs included; no exclusions or final-snapshot fallbacks.
- Deterministic bounded rerun: `mpx_001` 35 decision records and 21 total-support sensitivity records byte/value equivalent in memory.
- Output count checks: 365 decision rows, 10 run rows, 40 run-bin rows,
  4 run-macro bins, 214 total-support sensitivity rows, 212 class-matched
  sensitivity rows, 8 sensitivity-bin summaries, 17 figures, 30 overlays.
- `git diff --check`: PASS.
- Visual QA: run-macro figure and early overlay inspected; overlay was cropped
  to the evidence extent after QA exposed excessive fixed-canvas whitespace.

## V4 support inventory recomputed by final evaluator

- `F_count`: 14,589,701.
- `E_count`: 10,418,760.
- unsupported: 4,170,941.
- pooled support coverage: 0.7141174449.
- partial-support decisions: 199/365.
- zero-support decisions: 0.

Unsupported cells remain separate support evidence and are not imputed or
counted as prediction errors. Support is constructed from artifact geometry,
not prediction values.

## Required artifacts

- `prediction_vs_final_observed_decisions.csv`
- `prediction_vs_final_observed_runs.csv`
- `prediction_vs_final_observed_summary.json`
- `run_progress_bin_medians.csv`
- `run_macro_progress_bins.csv`
- `total_support_sensitivity.csv`
- `class_composition_sensitivity.csv`
- `sensitivity_run_bin_medians.csv`
- `sensitivity_progress_bins.csv`
- `partial_zero_support_inventory.csv`
- `provenance_exclusions_fallbacks.json`
- `overlay_manifest.csv` and 30 deterministic overlays
- 10 per-run quality curves and 7 cohort/support/class figures
- `execution.log`

Core hashes:

- decisions CSV: `a6a5e84fc62aa8a94b848e88a65a0781d585c65c5a42beb20931b076bacac59a`
- runs CSV: `43cc74f263810b8d5bbf871dd66227e757ad557f205e9e654a688832b8f19348`
- summary JSON: `7ea7ead173cbc5c30c92e2a7f8ef203dd7926991fd74708d40c7e67715f1a785`
- provenance JSON: `58d386b0ad30d135f2aace6ef04778586797630810ce6fd3b926eaa4ffcf38cc`

## Review notes / limitations

- The accuracy result is conditional on future observation and saved
  decision-time prediction support, exactly as V4 states.
- Both fixed sensitivities have no eligible decisions/runs in the final
  progress bin under their preregistered per-run 25th-percentile sample sizes;
  this is reported as unavailable and was not relaxed after outcomes.
- Figures/tables are engineering evidence only. CODEX makes no scientific
  verdict about usefulness or reliability.

## Handoff

Bàn giao cho: Chat 2 / W016 / CHAT-CRITIC.
Đọc trước: this file, then the evaluator, tests, summary/provenance and output
manifests on this branch.
Task: independently review implementation fidelity and result integrity.
Only ACCEPT opens Chat 1/W013 scientific interpretation.
