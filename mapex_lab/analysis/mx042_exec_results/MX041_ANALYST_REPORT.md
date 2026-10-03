# MX042 Analyst05 — MX041 Method R1 historical development replay result candidate

Status: COMPLETE_PENDING_INDEPENDENT_RESULT_QA
Frozen Method R1: 1f655c3a77b485369ac463479544ccbf728b4dc0
Classification: **MX041_NO_DEFENSIBLE_RUNTIME_HIDDEN_FREE_SURROGATE**

## Independent arm results
- F1: **ARM_NO_DEFENSIBLE_RUNTIME_SURROGATE**
  - A-S1=1 A-SV=0 A-S2=0 A-S3=1 A-S4=1 A-S5=1 A-S6=1
  - solved folds=10/10; held-out rate/bridge/area undercoverage=2/0/1
  - fired=10/10; positive saving=10/10; premature=1; severe=0; GT-under=0
  - median nonpremature delay=1.0; tau_full=0.06; outer tau matches=9/10; saving concentration=0.11585365853658537
- F2: **ARM_NO_DEFENSIBLE_RUNTIME_SURROGATE**
  - A-S1=1 A-SV=0 A-S2=0 A-S3=1 A-S4=1 A-S5=1 A-S6=1
  - solved folds=10/10; held-out rate/bridge/area undercoverage=1/0/0
  - fired=10/10; positive saving=10/10; premature=1; severe=0; GT-under=0
  - median nonpremature delay=1.0; tau_full=0.06; outer tau matches=9/10; saving concentration=0.12048192771084337

## Candidate set
None

A1-A28: 28/28 PASS
Hard execution/provenance failures: 0

Interpretation: historical development falsification conditional on the inherited same-cohort tau universe {5%,6%,8%,10%}.
F1 and F2 remain independent. No ranking or combination is produced.
No U/F3/F4/MX028/MX038 rescue, Hospital, prospective confirmation, deployment or robot STOP is authorized.
