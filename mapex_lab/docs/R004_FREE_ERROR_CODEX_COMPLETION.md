# R004 H044 free-error diagnostic completion

Date: 2026-09-24
Task: R004-FREE-ERROR-OFFLINE / H044
Maker: CODEX
Checker: Chat 2 / W016 / CHAT-CRITIC
Status: **COMPLETE_PENDING_REVIEW**

## Scope and provenance

- Technical branch: `r004-free-error-diagnostic`.
- Accepted base: `550fa35a082041972133700f9688130bc73fd6f3`.
- Historical input: read-only `mpx_001..mpx_010` from
  `/home/dell/turtlebot4_project_r003_p500_backup`.
- Result root:
  `mapex_lab/analysis/r004/results/free_error_diagnostic_v1/`.
- Frozen R004 V3/V4 and free-error diagnostic V3 methodology was not changed.
- No simulation, threshold tuning, band retuning, or base-result regeneration
  occurred.

The accepted base decisions, runs, and summary artifacts retain their accepted
hashes exactly:

- decisions: `a6a5e84fc62aa8a94b848e88a65a0781d585c65c5a42beb20931b076bacac59a`
- runs: `43cc74f263810b8d5bbf871dd66227e757ad557f205e9e654a688832b8f19348`
- summary: `7ea7ead173cbc5c30c92e2a7f8ef203dd7926991fd74708d40c7e67715f1a785`

## Validation

- Compile/direct-entrypoint check: PASS.
- Accepted evaluator plus diagnostic suites: 25/25 PASS.
- Full cohort execution: exit 0; 10/10 runs included; 365 decisions; no
  exclusions.
- Deterministic rerun: decisions, run bins, summary, provenance, and sampled
  early/late overlays byte-identical.
- `git diff --check`: PASS.
- Visual QA: union-share, denominator-controlled depth-rate, and early/late
  overlays inspected.
- Output inventory: 365 decision rows, 40 run/bin rows, 6 figures, 30 overlays,
  manifest/provenance/summary and durable execution log.

Focused tests cover TP-free/FalseFree/MissedFree identity, strict `p > 0.5`,
support-conditional miss denominator, unsupported-free accounting, Euclidean
axial/diagonal distance, exact depth-band boundaries, geometry independence,
denominator control, equal-run aggregation, cells-to-meters conversion, and
deep-occupied sensitivity without recall redefinition.

## Descriptive output inventory

Pooled cell counts are recorded only as secondary inventory:

- TP-free: 8,347,520
- FalseFree: 557,427
- MissedFree: 614,865
- supported future-observed free: 8,962,385
- unsupported future-observed free: 3,609,329

Primary interpretation remains the frozen run-macro progress-bin tables and
figures. CODEX does not select a scientific case. The completion package keeps
all four possibilities for independent review:

- Case A: FalseFree/deep-occupied dominant; hypothesis strongly supported.
- Case B: deep FalseFree material and MissedFree also rises; partly supported.
- Case C: no disproportionate deep FalseFree or MissedFree dominant; not
  supported.
- Case D: geometry/occupancy semantics insufficient; inconclusive.

## Core diagnostic hashes

- decisions: `be7e963d6652e91bd3bbea5a7aa62ce7d94b8c75ed18ddf77b26c7967b5e1b0a`
- run bins: `d00162d7a7990f0d90a9f51ca324bbaad7da89ace266524ebee679c27fc08fd4`
- summary: `055a5cde8667f52f82c334077bf58eab4bb58eee59ec16f2babfc28e77359c4a`
- provenance: `318e0c8e43b8660c81600a67d65316eaeb87ab0a0ca02b2092149d20f4f0affc`

## Required outputs

- `free_error_decomposition_decisions.csv`
- `free_error_decomposition_run_bins.csv`
- `free_error_decomposition_summary.json`
- `free_error_provenance.json`
- `free_error_overlay_manifest.csv`
- six required figures under `figures/`
- 30 deterministic early/middle/late overlays under `overlays/`
- `execution.log`

## Handoff

Bàn giao cho **Chat 2 / W016 / CHAT-CRITIC** for independent implementation
and result-integrity review. Read this file first, then the evaluator, tests,
summary/provenance, run-bin table and figures on this exact branch/SHA.

Only W016 ACCEPT may return the evidence to Chat 1 for scientific
interpretation.
