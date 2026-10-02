# MX037 Analyst05 bounded corrective result candidate — executed under MX038

Status: COMPLETE_PENDING_IR2_FOCUSED_RESULT_R2
Frozen Method V2: ac5b7a4e2b31a33aaaed1172d3b42f5fdf80862e
Classification: **NO_DEFENSIBLE_ONLINE_HIDDEN_FREE_RISK_PROXY_METHOD**

## Gates
- S1: **PASS** — blob_fail=0;hard=0
- S2: **FAIL** — matched_support=10/10;failures=mpx_002:COVERAGE_FAIL;mpx_003:COVERAGE_FAIL;mpx_004:COVERAGE_FAIL;mpx_005:COVERAGE_FAIL;mpx_006:COVERAGE_FAIL;mpx_007:COVERAGE_FAIL;mpx_008:COVERAGE_FAIL;mpx_009:COVERAGE_FAIL;mpx_010:COVERAGE_FAIL
- S3: **FAIL** — TC_fired=10;failures=mpx_002;mpx_003;mpx_004;mpx_005;mpx_006;mpx_007;mpx_008;mpx_009;mpx_010
- S4: **PASS** — fired=10;positive=10;median_delay=2.0
- S5: **PASS** — accounting=True;components=True;component_residual=True;early_TC=False;hard=0
- S6: **PASS** — A_pass=13/13

## TC STOP
- fired: 10/10
- positive saving: 10/10
- premature: 0
- SevereFalseStop10: 0
- median non-premature delay: 2.0 decisions

## Integrity
- merged decisions: 365
- hard source/parity failures: 0
- source blob failures: 0
- A1-A13: 13/13 PASS
- corrected TC stop earlier than BASE: False

This is the USER/PM-authorized bounded R1/R2 correction of predecessor candidate
8a866b3ef1780ea4e38ec2cc882619d34439165e. Online topology now uses the exact accepted MX031
runtime-grid representation; canonical 0.05 m projection is evaluator-only. A2
was repaired without changing the TC criticality rule. All Method V2 thresholds,
truth domains and gates remained frozen. No simulation/model prediction rerun,
retuning, Hospital, prospective collection, Engineer implementation, deployment
or robot STOP was performed.
