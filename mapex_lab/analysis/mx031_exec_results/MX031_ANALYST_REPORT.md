# MX031 Analyst04 deterministic result candidate

Status: COMPLETE_PENDING_IR1_RESULT_QA

Frozen method: c0130eb8bdbdba2a2b144c2d62e16e1bea00ef46
Method QA ACCEPT: 2d4590e0474be9c004704dcac21c8c4d34c35ec1

No stable New Room multi-signal STOP candidate satisfies the frozen V3 gates. No retuning is performed.

## Integrity

- New Room mpx_001..mpx_010: 365/365 accepted decisions.
- MX026 R replay blob: 8e73f4441b1c852f90d801c20f993216bbd1e78a.
- MX027 per-decision blob: dab247d44578d728c44ffc2245ee306d7375d9cb.
- MX027 source-parity blob: 629560dba455a91c95f2f636d4d532016fd5a4c1.
- W032 oracle_decisions blob: 9ae8f0479cf37a2b6aa9426a158f120efdc655cf.
- W032 oracle_decisions SHA-256: 2ca31f97ff4abfeb305667539dd0616af0caabfc5b11743c9bd8347e2f949cf8.
- R004 topology implementation blob: ff031cb826908aafd3bc039ca67a499ed092cbc5.
- Shared online-source implementation blob: c48de3322c4dcaefa63b07f8469cdfec2d740f1e.
- Frontier state counts: {'B_SELECTED_FRONTIER_AVAILABLE': 285, 'A_NO_RUNTIME_SELECTED_FRONTIER': 59, 'C_FRONTIER_INDETERMINATE': 21}.
- TopoValid reasons: {'outside_trajectory_range': 10, 'VALID': 219, 'SOURCE_NOT_PREDICTED_TRAVERSABLE': 95, 'SOURCE_OUTSIDE_RUNTIME_GRID': 41}.
- No Hospital data were scored.

## Outer LORO

- Solved folds: 10/10.
- Held-out fires: 10/10.
- Held-out positive-saving runs: 10/10.
- Held-out premature stops: 2.
- Held-out SevereFalseStop10: 0.
- Held-out median non-premature delay: 3.0 decisions.

## Stability

S1=PASS, S2=FAIL, S3=PASS, S4=PASS, S5=PASS, S6=PASS, S7=NA, S8=PASS

Final frozen development status: NO_STABLE_MULTI_SIGNAL_STOP_CANDIDATE

P/U and retrospective topology fields in MX031_STOP_CONTEXT.csv are joined only after stop decisions are fixed. They are evaluator context, not recognizer inputs.

Engineer remains blocked. No model/simulation rerun, Hospital retuning, weighted/composite score, P/U threshold promotion, online STOP implementation, or deployment is authorized.
