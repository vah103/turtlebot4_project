# R004 CODEX pickup blocker — prediction support versus frozen domain

Date: 2026-09-23
Task: R004-OFFLINE / H037
Owner: CODEX maker
Outcome: BLOCKED_PENDING_METHOD_REVIEW

## Finding

R004 V3 freezes the decision scoring domain as:

`UnknownAtDecision ∩ KnownInFinalObserved`

The historical ensemble-mean prediction NPZ for each decision covers only the
dynamic runtime raw-map footprint at that decision. The canonical observed and
final maps are fixed-canvas artifacts. As the SLAM map grows, many fixed-canvas
cells satisfy the frozen domain but lie outside the saved prediction footprint.
No decision-time prediction exists for those cells.

This is not a missing-file issue: all ten runs have one canonical final snapshot
and all 365 decision rows have their raw map, observed canvas and mean-prediction
artifact. It is a support-definition mismatch.

## Cohort audit

| Run | Decisions | Decisions affected | Frozen-domain pairs | Unsupported pairs | Unsupported fraction |
|---|---:|---:|---:|---:|---:|
| mpx_001 | 35 | 19 | 1,604,316 | 462,450 | 28.8254% |
| mpx_002 | 37 | 27 | 1,316,261 | 392,433 | 29.8142% |
| mpx_003 | 41 | 28 | 1,697,545 | 463,924 | 27.3291% |
| mpx_004 | 36 | 16 | 1,386,533 | 397,939 | 28.7003% |
| mpx_005 | 34 | 26 | 1,399,817 | 427,777 | 30.5595% |
| mpx_006 | 36 | 10 | 1,515,868 | 433,016 | 28.5655% |
| mpx_007 | 35 | 14 | 1,277,873 | 337,584 | 26.4176% |
| mpx_008 | 35 | 23 | 1,412,722 | 402,197 | 28.4696% |
| mpx_009 | 36 | 26 | 1,468,724 | 431,443 | 29.3754% |
| mpx_010 | 40 | 10 | 1,510,042 | 422,178 | 27.9580% |
| **Total** | **365** | **199** | **14,589,701** | **4,170,941** | **28.5883%** |

Concrete example: `mpx_001` decision 1 has 197,712 cells in the frozen domain,
but only 63,516 are inside saved prediction support; 134,196 cannot be scored.

## Why CODEX stopped

Each apparent workaround changes the frozen method:

- intersecting the domain with `PredictionSupportAtDecision` changes its
  denominator and introduces a new support-selection condition;
- excluding every affected decision removes 199/365 decisions and changes
  progress/bin coverage;
- filling outside-footprint values invents predictions that were never saved;
- falling back to a later prediction leaks future information.

H037 says to stop and return a concrete blocker when V3 cannot be implemented
exactly from historical artifacts. Therefore CODEX did not implement scoring,
run aggregate analysis, inspect outcomes, or tune any rule.

## Required decision

Return to the method maker (Chat 1 / W013) and independent checker (Chat 2 /
W016). They must freeze and review one explicit resolution, including its
selection-bias limitation, before CODEX resumes. The most direct candidate is
to redefine the primary domain as:

`UnknownAtDecision ∩ KnownInFinalObserved ∩ PredictionSupportAtDecision`

but CODEX is not authorizing that change. Alternatives must likewise specify
how affected decisions enter progress bins and sensitivities.

## Resume condition

Resume only after an updated method receives independent ACCEPT and the Hub
issues a revised engineering handoff. On resume, inspect this branch and these
checkpoint documents before writing or running the evaluator.
