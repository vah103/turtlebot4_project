# MX025 Analyst Report — Map-only R around Oracle-4

Accepted method revision: fc0ea05313dc02e6b8a957552c31f952de0897d6
Frozen technical base: 4899ee95c85640965befeaf98c20f156c0d3181a

## A — Primary map-only R near Oracle-4
- MapRemainingFraction: PRE→POST directions = 10 decreasing, 0 increasing, 0 exact-zero across 10 runs with both sides; run-macro median POST-minus-PRE = -0.0569474943.
- R_MAP_A_mean_m2: PRE→POST directions = 10 decreasing, 0 increasing, 0 exact-zero across 10 runs with both sides; run-macro median POST-minus-PRE = -26.1233341.
- Oracle-4 is used only as a timing anchor. MapRemainingFraction is not treated as true remaining fraction or directly calibrated to Oracle-4.

## B — Reachable-R comparison and missingness recovery
- Primary R-map is valid on 365/365 decisions; exact raw/G1/G2/G3 alignment passes 365/365.
- Reachable-R comparator is valid on 218/365 decisions.
- In W10, R-map is valid on 72/72 rows versus reachable-R 39/72; recovered map-only values where reachable-R is NA = 33.
- W10 paired rows = 39; median area difference map-minus-reachable = 12.3233337 m²; median area ratio map/reachable = 2.26416359.
- W10 median normalized-fraction difference map-minus-reachable = 0.0193105401; median fraction ratio map/reachable = 1.76810046.
- The area subset invariant A_map_j >= A_reachable_j passed for every paired valid decision/member.

## C — Exact mpx_004 W10 sanity table

| decision | band | delta_p | A_map_mean m² | KnownFree_map m² | MapRemainingFraction | reachable valid | A_reachable_mean m² | ReachableFraction | reachable reason |
|---:|---|---:|---:|---:|---:|---:|---:|---:|---|
| 13 | PRE | -0.085714 | 60.320002 | 399.590012 | 0.131156 | 1 | 38.903334 | 0.110194 |  |
| 14 | PRE | -0.057143 | 47.113335 | 413.630012 | 0.102255 | 0 | NA | NA | source_not_known_footprint_safe |
| 15 | PRE | -0.028571 | 42.306668 | 419.250012 | 0.091661 | 1 | 26.303334 | 0.073446 |  |
| 16 | ANCHOR | 0.000000 | 22.513334 | 439.830013 | 0.048694 | 1 | 9.943334 | 0.027560 |  |
| 17 | POST | 0.028571 | 21.313334 | 441.500013 | 0.046052 | 0 | NA | NA | source_not_known_footprint_safe |
| 18 | POST | 0.057143 | 14.856667 | 447.060013 | 0.032163 | 1 | 5.113333 | 0.014094 |  |
| 19 | POST | 0.085714 | 13.876667 | 448.360013 | 0.030021 | 1 | 4.643333 | 0.012774 |  |

## D — Interpretation boundary
- Removing reachability increases data availability and measures a different map-level quantity; this does not show that reachability is unnecessary for planning.
- Reachable-R remains a separate accepted historical metric. MX025 does not reopen or relabel Gate R.
- No LaMa/model rerun, simulation, prediction regeneration, interpolation, retuning, threshold change, composite, or online STOP rule was used.
- Scope is the frozen New Room development cohort around retrospective Oracle-4 only.
