# MX037 Analyst04 retrospective result candidate — executed under MX038

Status: COMPLETE_PENDING_INDEPENDENT_RESULT_QA
Frozen Method V2: ac5b7a4e2b31a33aaaed1172d3b42f5fdf80862e
Classification: **MX037_INVALID_EXECUTION_OR_EVIDENCE**

## Gates
- S1: **FAIL** — blob_fail=0;hard=45
- S2: **FAIL** — matched_support=10/10;failures=mpx_002:COVERAGE_FAIL;mpx_003:COVERAGE_FAIL;mpx_004:COVERAGE_FAIL;mpx_005:COVERAGE_FAIL;mpx_007:COVERAGE_FAIL;mpx_008:COVERAGE_FAIL;mpx_009:COVERAGE_FAIL;mpx_010:COVERAGE_FAIL
- S3: **FAIL** — TC_fired=10;failures=mpx_002;mpx_003;mpx_004;mpx_005;mpx_007;mpx_008;mpx_009;mpx_010
- S4: **PASS** — fired=10;positive=10;median_delay=2.0
- S5: **FAIL** — accounting=True;components=True;component_residual=True;early_TC=False;hard=45
- S6: **FAIL** — A_pass=12/13

## TC STOP
- fired: 10/10
- positive saving: 10/10
- premature: 0
- SevereFalseStop10: 0
- median non-premature delay: 2.0 decisions

## Integrity
- merged decisions: 365
- hard source/parity failures: 45
- source blob failures: 0
- A1-A13: 12/13 PASS
- corrected TC stop earlier than BASE: False

Execution was parallelized by fixed run only. All scientific functions/thresholds,
truth domains and gates remained frozen. No Hospital, prospective collection,
Engineer implementation, deployment or robot STOP was performed.
