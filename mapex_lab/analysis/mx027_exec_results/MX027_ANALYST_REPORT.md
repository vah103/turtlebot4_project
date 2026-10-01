# MX027 Analyst Report — full-trajectory IG + Coverage/Stagnation

Accepted methodology: 6925611773ced0722ffc8bf43279084dd01fad75
Frozen technical base: 4899ee95c85640965befeaf98c20f156c0d3181a

## Inventory
- 365/365 decision rows; IG evaluable 285/365; legitimate runtime no-selection IG NA 59/365; nonblank-policy rows with no unique runtime-selected frontier = 21; Coverage evaluable 365/365.
- Candidate lookup attempted only for the 306 nonblank-policy decisions; all 59 blank-policy no-selection rows skipped candidate lookup. Candidate-table rows without a unique selected frontier remain IG NA fail-closed; no candidate value is reconstructed or substituted.
- Five-family joined view contains 365 exact accepted MX026 keys and preserves MX026 P/U/R values read-only.
- IG reason counts: {'OK': 285, 'NO_RUNTIME_SELECTED_FRONTIER': 59, 'SELECTED_FRONTIER_CARDINALITY_NE_1': 21}. Policy-score reason counts: {'OK': 285, 'NO_RUNTIME_SELECTED_FRONTIER': 59, 'SELECTED_FRONTIER_CARDINALITY_NE_1': 21}. Coverage reason counts: {'OK': 365}.

## Whole-run descriptive direction
- IG_selected: final-minus-first negative/positive/zero = 10/0/0; run-macro median delta=-66.236409; median rho(progress)=-0.850159 across 10 finite run rhos.
- IG_visible_unknown_cells: final-minus-first negative/positive/zero = 10/0/0; run-macro median delta=-1699.5; median rho(progress)=-0.802732 across 10 finite run rhos.
- IG_density: final-minus-first negative/positive/zero = 5/5/0; run-macro median delta=-0.00982513689; median rho(progress)=0.0970405 across 10 finite run rhos.
- IG_policy_score: final-minus-first negative/positive/zero = 10/0/0; run-macro median delta=-33.5348581; median rho(progress)=-0.903366 across 10 finite run rhos.
- KnownArea_m2: final-minus-first negative/positive/zero = 0/10/0; run-macro median delta=493.885015; median rho(progress)=0.992869 across 10 finite run rhos.
- DeltaKnownArea_m2: final-minus-first negative/positive/zero = 10/0/0; run-macro median delta=-67.180002; median rho(progress)=-0.902198 across 10 finite run rhos.
- KnownAreaRate_m2_s: final-minus-first negative/positive/zero = 10/0/0; run-macro median delta=-9.59965567; median rho(progress)=-0.931368 across 10 finite run rhos.

## Signed coverage/stagnation context
- Negative DeltaKnownArea decisions: 10; negative KnownAreaRate decisions: 10. Negative values are retained, not clamped.

## Boundaries
- No-selection IG is semantic NA, never IG=0.
- Coverage/Stagnation is independently computed from exact raw_map/resolution/time_s and does not depend on IG evaluability.
- No binary stagnation threshold, STOP rule, cross-family correlation matrix, composite, winner, causal claim, retuning, interpolation, model rerun or new simulation is produced.
