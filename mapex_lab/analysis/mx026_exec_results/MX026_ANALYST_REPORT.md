# MX026 Analyst Report — full-trajectory P/U/R survey

Accepted methodology: f5144e2727993b920d172fa106ad5477a9c6fbe6
Frozen technical base: 4899ee95c85640965befeaf98c20f156c0d3181a

## 1. Inventory and evaluability
- Master trajectory: 365/365 unique decisions.
- P structural: 365/365; P later-observed: 286/365; P topology: 168/365.
- U primary: 365/365.
- R map-only: 365/365; reachable-R secondary: 218/365.

## 2. Whole-run direction summaries
- P_STRUCT_FreeRecall: final-minus-first 10 negative / 0 positive / 0 zero; run-macro median delta=-0.875218587; median valid rho(progress)=-0.899286 (n=10).
- P_STRUCT_FreePrecision: final-minus-first 10 negative / 0 positive / 0 zero; run-macro median delta=-0.950054032; median valid rho(progress)=-0.983899 (n=10).
- P_STRUCT_MissedFreeRate: final-minus-first 0 negative / 10 positive / 0 zero; run-macro median delta=0.875218587; median valid rho(progress)=0.899286 (n=10).
- P_STRUCT_OccupiedRecall: final-minus-first 0 negative / 10 positive / 0 zero; run-macro median delta=0.218440188; median valid rho(progress)=0.185493 (n=10).
- P_STRUCT_FalseOpenRate: final-minus-first 10 negative / 0 positive / 0 zero; run-macro median delta=-0.218440188; median valid rho(progress)=-0.185493 (n=10).
- U_MAPEX_U_mean: final-minus-first 10 negative / 0 positive / 0 zero; run-macro median delta=-0.0358322025; median valid rho(progress)=-0.875286 (n=10).
- U_MAPEX_U_p95: final-minus-first 10 negative / 0 positive / 0 zero; run-macro median delta=-0.212318769; median valid rho(progress)=-0.958892 (n=10).
- R_MapRemainingFraction: final-minus-first 10 negative / 0 positive / 0 zero; run-macro median delta=-0.917977557; median valid rho(progress)=-0.989892 (n=10).
- R_A_map_mean_m2: final-minus-first 10 negative / 0 positive / 0 zero; run-macro median delta=-133.526671; median valid rho(progress)=-0.976287 (n=10).

## 3. Six frozen cross-family associations
- U_MAPEX_U_p95 vs P_STRUCT_MissedFreeRate: valid rho 10/10; run-macro median rho=-0.889071; negative/positive/zero=10/0/0; NA reasons={}.
- U_MAPEX_U_p95 vs P_STRUCT_FalseOpenRate: valid rho 10/10; run-macro median rho=0.185744; negative/positive/zero=1/9/0; NA reasons={}.
- U_MAPEX_U_p95 vs P_STRUCT_FreePrecision: valid rho 10/10; run-macro median rho=0.959159; negative/positive/zero=0/10/0; NA reasons={}.
- R_MapRemainingFraction vs P_STRUCT_MissedFreeRate: valid rho 10/10; run-macro median rho=-0.89291; negative/positive/zero=10/0/0; NA reasons={}.
- R_MapRemainingFraction vs P_STRUCT_FalseOpenRate: valid rho 10/10; run-macro median rho=0.192459; negative/positive/zero=1/9/0; NA reasons={}.
- R_MapRemainingFraction vs U_MAPEX_U_p95: valid rho 10/10; run-macro median rho=0.963213; negative/positive/zero=0/10/0; NA reasons={}.

## 4. Exact p=0.5 boundary audit
- Q1_STRUCTURAL_GT: scoreable rows 365/365; exact-boundary rows 0; exact-boundary cells 0; affected identities: none.
- Q1_LATER_OBSERVED: scoreable rows 286/365; exact-boundary rows 0; exact-boundary cells 0; affected identities: none.

## 5. Interpretation boundary
- Full 365-decision trajectory is primary; fixed progress bins are secondary equal-run summaries.
- Oracle-4 is marker only and did not select rows, bins, metrics, pairs, or conclusions.
- Family-specific missingness remains separate; no cross-fill, interpolation or smoothing.
- Six pairwise rhos are descriptive only; no p-values, significance labels, composite, winner, causal claim, retuning or online STOP.
