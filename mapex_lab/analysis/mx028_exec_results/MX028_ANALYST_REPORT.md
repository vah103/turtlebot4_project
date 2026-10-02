# MX028 Analyst04 — Full-365 P×U result candidate

Status: **COMPLETE_PENDING_IR2_FOCUSED_RESULT_R2**

Frozen method: 429733307f760a5efd9e9641834146e71b927428
Frozen technical source: 4899ee95c85640965befeaf98c20f156c0d3181a

## Execution integrity

- 10/10 New Room runs; **365/365** unique decisions.
- Saved original-MapEx variance parity vs np.var(G1,G2,G3, ddof=1): **365/365 PASS**.
- Structural Gate-P evaluator blob: c5bf4e3e6f45c81afef135166fc96e088719658e.
- Structural GT blob: a5653a7ec7f4550287a3ebeb05b922dbc7a5bc3a.
- Runtime exact p == 0.5 unknown-cell count across all decisions: **0**.
- Decision-relative Q25/Q75 collapsed decisions: **0/365**.
- Later-observed decisions with first-later target support: **286/365**.

## Descriptive trajectory checks

Median per-run Spearman rho versus normalized progress:
- OccLowU_share_unknown: **0.969682**
- PredOcc_share_unknown: **0.956701**
- PredOcc_median_U: **-0.733262**
- Structural OccPrecision_LOW: **0.934986**
- Structural OccPrecision_HIGH: **0.980379**

These are descriptive maker-side checks only. IR2 independent result QA owns acceptance/revision.

## Interpretation guard

LOW/MID/HIGH are relative quartiles within each decision's runtime unknown domain.
A change in OccLowU_share_* is a relative-composition trend; it is not an
absolute confidence, calibration, or safety claim. Structural and Later-observed
truth populations remain separate.

No model/simulation/prediction rerun, threshold optimization, pseudocount,
imputation, composite, winner, STOP/deployment rule, calibration claim, or
causal claim was introduced.

## Bounded support-semantics correction — successor 05

The predecessor Analyst04 scientific/numerical result remains frozen. Successor05 corrected only the Oracle/late support representation required by the IR2 corrective review.

- 330/330 summary metric `value` fields, Oracle decisions, bin membership, valid-decision counts and candidate-decision counts are preserved exactly from the predecessor summary.
- `OccLowU_share_unknown` now carries `D_U_count` support.
- LOW/HIGH occupied precision carries its corresponding stratum support.
- LOW-minus-HIGH precision exposes LOW and HIGH supports separately.
- wrong-minus-correct median-U gap exposes correct-U n and wrong-U n separately.
- Structural and Later-observed supports remain separate.
- 270/330 rows required corrected or expanded support semantics; 60/330 already had the correct single-support mapping.
- No model/simulation/prediction rerun, retune, metric-value change, threshold/bin/Oracle change, figure change, or new scientific claim was made.

Predecessor authorship/provenance for the frozen scientific result remains unchanged; this bounded package correction is attributable to Data & Evidence Analyst successor 05 and is pending IR2 successor01 focused R2.
