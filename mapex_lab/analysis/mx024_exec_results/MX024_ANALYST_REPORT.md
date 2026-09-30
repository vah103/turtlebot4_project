# MX024 Analyst Report — MapEx-original-first U around Oracle-4

Method revision: 6fe651e383b3d88d1a824558d6ec398fc97eef1c
Technical base: 4899ee95c85640965befeaf98c20f156c0d3181a

## A — MapEx U near Oracle-4
- MAPEX_U_mean: PRE/ANCHOR/POST run-macro medians = 0.0426422924 / 0.0420899344 / 0.0359417137; POST-vs-PRE directions = 7 decreasing, 3 increasing, 0 exact-zero of 10.
- MAPEX_U_p95: PRE/ANCHOR/POST run-macro medians = 0.32952658 / 0.310155224 / 0.30587346; POST-vs-PRE directions = 9 decreasing, 1 increasing, 0 exact-zero of 10.
- Direction is descriptive only; no usefulness threshold or STOP rule is inferred.

## B — Ensemble agreement
- MAPEX_U_disagreement PRE/ANCHOR/POST = 0.0521223603 / 0.0537183939 / 0.0491690249; directions = 6 decreasing, 4 increasing, 0 exact-zero.
- Disagreement is secondary diagnostic, not original MapEx U.

## C — Does high U identify prediction errors?
- STRUCTURAL_GT W10 equal-run medians: ErrorEnrichment95=1.22347463 (n=10), ErrorCapture95=0.0611995524 (n=10), MissedFreeCapture95=0.0961150965 (n=10), FalseOpenCapture95=0.0446244235 (n=10).
- LATER_OBSERVED W10 equal-run medians: ErrorEnrichment95=2.92512063 (n=10), ErrorCapture95=0.14697155 (n=10), MissedFreeCapture95=0.150231915 (n=10), FalseOpenCapture95=0.107646935 (n=10).
- Structural-GT and later-observed denominators remain separate.

## D — Where are high-U cells?
- Generated 10/10 deterministic Oracle-4 spatial panels.

## E — Planning context
- No new topology/path criterion was introduced. D1 R_union U remains secondary per-decision context.

## F — Bounded synthesis / integrity
- Saved variance parity passed 365/365; primary Q1 evaluable 365/365; mean/Q2 source evaluable 365/365.
- mpx_003 d18–20 primary Q1: d18=PASS, d19=PASS, d20=PASS
- No LaMa rerun, new simulation, prediction regeneration, retuning, winner/composite, causal claim, or online STOP.
- Scope: New Room development cohort around retrospective Oracle-4 only.
