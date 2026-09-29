# MX016 Hospital transfer sanity replay — maker completion

## Bound scope

- Run: existing `mapex_lab/experiments/mapex/hpx_001` only; no simulation or new truth generated.
- Method: exact accepted MX015 V2; frozen `tau_R=0.03`, `K=1`, `TopoValid` required, U non-gating.
- Recognizer: accepted MX013 revision `20edfd1527c63ade813203af04f4a773313f65c9`, module blob `8c4d993be1218d5273c4f6c3b57483dae78d5e0b`.
- Phase A and Phase B remain separate entry points. Phase B refuses to run without a verified sealed Phase-A manifest/hash.

## Maker result pending independent QA

- Phase A: `STOP_CONSIDER` at decision 30 (`t=907.998655796 s`), where `R_hat=0.013645553577355984` and `TopoValid=TRUE`.
- Descriptive saving if acted on: 26 of 56 decisions, `1151.000638962 s`, progress fraction `0.4642857142857143`.
- Sealed trace SHA-256: `47989ceba883f5d9ef6874225806e747a185bfbd48da5f9679f3c52d8efed751`.
- A full Phase-A rerun after truth inspection was byte-identical (`cmp` exit 0; same SHA).
- Oracle validity: G1 PASS; G2 FAIL because exact `hospital_structural_gt_v1` artifact SHA `7db2d4...b9e9` recorded by the run is unavailable on DELL; G3–G7 cannot be established without it. G8 legacy ROI audit is recorded as the required non-fatal geometry-difference warning.
- Hospital Oracle-4, early/on-target/late, true residual, premature consequence and retrospective topology impact are not evaluable.
- Exact top-level label: `TRANSFER_SANITY_INCONCLUSIVE_ORACLE_INVALID`.
- This label says nothing about transfer quality, does not change the accepted negative MX013 result, and does not authorize confirmation or deployment.

## Verification

- `python3 -m unittest mapex_lab.analysis.d1.test_mx013_online_stop mapex_lab.analysis.d1.test_mx016_hospital_transfer` — 21 tests PASS.
- `python3 -m py_compile mapex_lab/analysis/d1/mx016_hospital_transfer.py mapex_lab/analysis/d1/run_mx016_hospital_transfer.py` — PASS.
- Deterministic trace comparison — byte-identical PASS.
- Result package: `mapex_lab/analysis/d1/results/mx016_hospital_transfer_v1/`.

Maker execution is complete and must now be reviewed by Independent Research QA; this document is not a self-QA verdict.
