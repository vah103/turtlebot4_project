# MX044 Analyst05 — MX043 Method R1 direct utility historical replay result candidate

Status: COMPLETE_PENDING_INDEPENDENT_RESULT_QA
Frozen Method R1: 333f64887f66794da7fcd6aba1ac547d6cd999d4
Classification: **MX043_NO_DEFENSIBLE_DIRECT_UTILITY_BOUND**

## Independent arm results
- F1: **ARM_NO_DEFENSIBLE_DIRECT_UTILITY_BOUND**
  - A-S1=1 A-SV=0 A-S2=1 A-S3=0 A-S4=0 A-S5=1 A-S6=1
  - solved folds=0/10; UtilityUnder violations=0
  - fired=0/10; positive saving=0/10; premature=0; severe=0; utility-false=0; utility-unverifiable=0
  - median nonpremature delay=nan; tau_full=nan; outer tau matches=0/10; saving concentration=1.0
- F2: **ARM_NO_DEFENSIBLE_DIRECT_UTILITY_BOUND**
  - A-S1=1 A-SV=0 A-S2=1 A-S3=0 A-S4=0 A-S5=1 A-S6=1
  - solved folds=0/10; UtilityUnder violations=0
  - fired=0/10; positive saving=0/10; premature=0; severe=0; utility-false=0; utility-unverifiable=0
  - median nonpremature delay=nan; tau_full=nan; outer tau matches=0/10; saving concentration=1.0

## Candidate set
None

A1-A31: 31/31 PASS
Hard execution/provenance failures: 0

Interpretation: historical development falsification conditional on inherited same-cohort tau universe {5%,6%,8%,10%}.
No F1/F2 ranking or combination. No H/B_U/model/margin/SupportBox/tau/K retuning.
No U/F3/F4, MX041/MX042 rescue, Hospital, prospective confirmation, deployment or robot STOP.
