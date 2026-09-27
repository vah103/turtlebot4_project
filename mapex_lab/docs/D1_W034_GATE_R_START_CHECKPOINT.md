# D1 W034 Gate R Offline — Start Checkpoint

Policy epoch acknowledged: `2026-09-24-D047`

- Task: `D1-R-OFFLINE` / H084
- Owner/session role: W034 / CODEX
- Host: `dell` (authorized DELL execution host)
- Usage mode at pickup: LOW (55% remaining in the five-hour window)
- Branch: `d1-gate-r-w034`
- Exact starting HEAD: `4f674a81add0c14eb15cc0ab95754130c5923ead`
- Frozen method: `D1_GATE_R_METHOD_V2.md` at Hub commit
  `c883b0ed1e335cad08036d658d305f721ac56429`
- Accepted oracle decisions blob: `9ae8f0479cf37a2b6aa9426a158f120efdc655cf`
- Accepted shared-evidence blob: `3e80124d545ea37b2791594dd775a7d953368bea`

## Phase scope

Implement and test only the frozen historical Gate R V2 evaluator, then run its
one-fold/one-run smoke and complete 10-fold scoring over `mpx_001..mpx_010`
(365 immutable decisions). No method change, simulation, prediction/GT/oracle
regeneration, operational threshold fitting, `K_confirm`, Gate-U change,
combined U x R rule, online STOP, confirmation scoring, or full-PASS claim is
authorized.

## Planned durable outputs

- evaluator/tests under `mapex_lab/analysis/d1/`;
- logs and generated artifacts under
  `mapex_lab/analysis/d1/results/gate_r_v2/`;
- completion note at
  `mapex_lab/docs/D1_W034_GATE_R_OFFLINE_COMPLETION.md`.

## Milestones and stop conditions

1. Input identity, 10-run/365-row, one-to-one join and NA-semantics preflight.
2. Coherent evaluator plus focused synthetic/contract tests.
3. One-fold/one-run smoke with formula inspection.
4. Full deterministic 10-fold scoring, artifact/hash and no-input-mutation
   verification.
5. Push technical PR and return exact head to W029 for independent review.

Hard-stop on identity/join/provenance mismatch, method ambiguity requiring a
scientific choice, or usage exhaustion. Resume by inspecting this branch,
`git status`, the latest milestone commit, and persisted result logs before
rerunning any command.
