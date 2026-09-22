# R002 Reference-B visual audit — mpx_001

Status: **H012 evidence package / pending independent H013 review**

This directory records the lightweight evidence for the visualization-only audit
of the already accepted R002 Reference-B semantics.

## Frozen sets shown

```text
B_GT_t   = GT_surface ∩ U_t
B_PRED_t = PredictedOccupiedBoundary_t ∩ U_t
```

The renderer imports the accepted projection, threshold, boundary extraction
and 0.10 m one-to-one matching implementation from
`r002_gt_semantics.py`. It does not redefine R002 metric semantics.

## Image layout

Each local `decision_XXXXXX_reference_b.png` has two panels:

1. **Literal layers**
   - lower/reference layer: `B_GT_t` in dark gray/black;
   - upper/prediction layer: `B_PRED_t` in semi-transparent red.

2. **Accepted 0.10 m matching diagnostic**
   - green: GT/pred endpoints participating in the accepted one-to-one match;
   - dark gray/black: unmatched GT target;
   - red: unmatched predicted boundary;
   - faint green segment: matched GT/pred cells when their cells differ.

The right panel is only a visualization of the already-frozen matching.
The official quantitative Reference-B result remains the evaluator output.

## Executed scope

- run: `mpx_001` only;
- decision images generated: **35/35**;
- renderer exit code: **0**;
- pre-render validation: Python compile PASS, existing R002 unit suite **12/12 PASS**;
- accepted first-score decision-level B parity: **PASS**;
- prediction threshold: imported frozen value `0.5`;
- primary B tolerance: imported frozen value `0.10 m`;
- provenance warnings: none.

The accepted first-score comparison source was:

```text
mapex_lab/analysis/d1/results/r002_mpx_001_h009_firstscore/r002_decisions.csv
```

## Git retention policy

The 35 PNGs are generated audit artifacts and are intentionally **not committed**
to keep the repository lightweight. Their deterministic names and per-decision
metrics are recorded in `manifest.json` and `index.csv`.

Renderer source:

```text
mapex_lab/analysis/d1/r002_render_reference_b.py
```

Implementation branch:

```text
r002-b-vis-h012
```

Initial renderer commit:

```text
af1c450870689c6ded675b2c9f20db68b0a488f2
```
