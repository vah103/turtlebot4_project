# D1 W034 Gate R Historical Offline Completion

Status: **COMPLETE_PENDING_REVIEW**

- Task: `D1-R-OFFLINE` / H084
- Executor: W034 / CODEX
- Independent checker: W029 / Chat 2 successor
- Host: DELL (`hostname = dell`)
- Usage mode: LOW
- Branch: `d1-gate-r-w034`
- Implementation commit: `ad659e5ae944f49ad5894653d42c0c8365b9eb67`
- Accepted oracle head / technical start: `4f674a81add0c14eb15cc0ab95754130c5923ead`
- Frozen method commit: `c883b0ed1e335cad08036d658d305f721ac56429`
- Oracle decisions blob: `9ae8f0479cf37a2b6aa9426a158f120efdc655cf`
- Shared evidence blob: `3e80124d545ea37b2791594dd775a7d953368bea`

## Verification

- focused tests: **28/28 PASS**;
- one-fold / `mpx_001` production-data smoke: PASS;
- full LORO: **10 folds / 10 runs / 365 unique decisions**;
- one-to-one oracle/shared join: PASS;
- accepted input identities and SHA-256 hashes: PASS;
- NA preservation and exact-oracle-row/no-neighbour substitution: PASS;
- no-input-mutation check: PASS;
- deterministic full rerun: **17/17 artifacts byte-identical**;
- no simulation, prediction/GT/oracle regeneration, retuning, operational STOP,
  `K_confirm`, Gate-U change, U x R combination, or confirmation scoring.

## Frozen-method result

Exact historical classification:

`HISTORICAL_NEW_ROOM_FEASIBILITY_SUPPORTED_PENDING_CONFIRMATION`

This is the strongest historical-development label allowed by V2. It is not a
full Gate-R PASS and does not change canonical Gate-U FAIL.

- fold selection: `RemainingFraction` in **10/10** folds;
- `NO_ADMISSIBLE_C`: **0/10** folds;
- final all-ten development candidate: `RemainingFraction`;
- held-out finite-pair support: 218 decisions across 10/10 runs;
- held-out rho: median `0.9972756779445706`, mean `0.9940425376383356`;
- TRR50: mean `0.0752550066455658`;
- FC25: mean `0.0`, defined in 9 runs;
- OracleFC10 events: `0`;
- signed calibration bias: run-median of run means
  `-0.031055623742458988`;
- exact OracleStop_5 candidate support: **7/10** runs;
- exact OracleStop_5 median development percentile:
  `0.4312315006517905`.

All frozen support criteria recorded in `gate_r_summary.json` evaluate true;
none of the frozen FAIL criteria fires. These facts remain subject to W029
independent implementation/result review.

## Outputs

Root: `mapex_lab/analysis/d1/results/gate_r_v2/`

- `gate_r_decisions.csv`, `gate_r_runs.csv`, `gate_r_folds.csv`;
- `gate_r_false_completeness.csv`, `gate_r_oracle_local.csv`;
- `gate_r_progress_bins.csv`, `gate_r_future_gain.csv`;
- `gate_r_summary.json`, `artifact_manifest.json`, `tests.log`;
- seven mandatory descriptive PNG figures.

Durable execution log:
`mapex_lab/analysis/d1/gate_r_v2_run.log`.

Next action: push the exact implementation/delivery head and technical PR,
then W029 independently checks method fidelity, code, identities, metrics,
classification and deterministic artifacts before any Chat 1 interpretation.
