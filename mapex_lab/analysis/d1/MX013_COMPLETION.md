# MX013 frozen online STOP recognizer — engineering completion

Status: **COMPLETE_PENDING_INDEPENDENT_RESEARCH_QA**  
Scope: deterministic development-only replay on `mpx_001..010`; no confirmation, robot STOP, or deployment.

## Result

The exact MX012 V2 implementation selected the full-development primary pair:

- `tau_R = 3%`;
- `K = 1`;
- stop coverage `10/10`;
- positive-saving coverage `10/10`;
- full-development premature stops `0`;
- full-development SevereFalseStop10 events `0`;
- median non-premature delay `1` decision;
- mean saved progress `46.98%` (median `46.61%`);
- mean decisions saved `16.7` per run (`167` total);
- mean accepted-timing saving `387.46 s` per run.

However, LORO selected `tau_R=4%, K=1` when `mpx_001` was held out and fired at decision 20 versus OracleStop_4 decision 21. This is one premature held-out stop. The frozen zero-premature gate therefore fails.

Final development classification:

`NO_STABLE_ONLINE_RECOGNIZER_CANDIDATE`

## Stability criteria

| Criterion | Result | Evidence |
|---|---|---|
| S1 fold solvability | PASS | 10/10 folds admissible |
| S2 threshold stability | PASS | 9 folds at 3%, one at 4%; within frozen limits |
| S3 persistence stability | PASS | all folds K=1 |
| S4 held-out false-stop safety | **FAIL** | one premature stop: `mpx_001`, decision 20 vs 21 |
| S5 held-out usefulness | PASS | 10/10 coverage and positive saving; median delay 1 decision |
| S6 resolution sensitivity | PASS | 3.5%, K=1; within 1 pp |
| S7 widened-boundary sensitivity | PASS | 3.5%, K=1; within 2 pp and interior |

## Ablations

- A0 primary, A1 R-only, and A2 K=1 are behaviorally identical at the selected pair on these runs.
- A3 U-veto stops on 8/10 runs, saves less progress, and has median delay 6 decisions.
- Retrospective topology metrics are logged at stop rows as evaluator-only, non-gating diagnostics.

## Implementation and evidence

- recognizer: `mx013_online_stop.py`;
- replay driver: `run_mx013_online_stop.py`;
- focused tests: `test_mx013_online_stop.py` — 12/12 PASS;
- result package: `results/mx013_online_stop_v1/`;
- full primary/sensitivity traces are deterministically gzip-compressed;
- artifact manifest SHA-256: `01415a47c045a1b9afa5486d564405736947366f2a209cfc8b44dc758aed4f42`;
- summary SHA-256: `96f56a7fe6c1a8a06e14ddc367d8ef91c3dbdd944c080c0b91937dfdd62a3306`;
- two complete reruns produced byte-identical manifested artifacts.

The recognizer API contains no OracleStop_4, structural GT, future/final map, or retrospective topology fields. Truth is joined only by `evaluate_run` after recognizer execution.

## Next gate

Independent Research QA must review the exact code/result revision. No result may be promoted to confirmation, live STOP, or deployment from this maker completion.
