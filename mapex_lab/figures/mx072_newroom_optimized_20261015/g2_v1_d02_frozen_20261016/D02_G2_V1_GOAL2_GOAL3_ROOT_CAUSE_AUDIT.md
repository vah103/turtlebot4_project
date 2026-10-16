# Audit: Why did frozen G2-V1 Run04 D02 choose goal 2 and goal 3?

Date 2026-10-16. **This is diagnostic only.** No planner code or source input modified. D02 observed + mean + variance + R4.1 semantic only. GT, D03+, post-VP1 new observations not used here.

## Original frozen G2-V1 output

Default G2Config 128 planning rays, 199 reachable candidates, beam_width=5, branch_width=12; route from current robot (11.1,14.7)m via:
- Goal1 core (171,138), x13.8m y17.1m, 5.1m, marginal weighted 3063.67;
- Goal2 core (180,150), x15.0m y18.0m, +2.1m, marginal weighted +301.38;
- Goal3 core (183,153), x15.3m y18.3m, +0.6m, marginal weighted +32.25;
- Default objectives: after 1 175.27, after 2 141.16, after 3 139.68 seconds-equivalent.

The plotted teal/green area is **projected probability of observing unknown cells**. It is **not actual LiDAR or observed-free evidence**.

## 1. Ray density sensitivity: re-evaluate *frozen* 3-goal sequence with unchanged cost and LaMa/R4.1

| Planner beams (sensitivity only) | Marginal weighted Goal2 | Marginal weighted Goal3 | G3 minus G2 objective |
|---:|---:|---:|---:|
| 128 | 301.38 | 32.25 | -1.48 s-eq (G3 apparently better) |
| 250 | 146.30 | 18.44 | +0.59 s-eq (G3 worse) |
| 720 | 136.30 | 15.30 | +1.02 s-eq (G3 worse) |
| 2,500 | 135.03 | 15.17 | **+1.11 s-eq (G3 worse)** |

The 128 ray model overstates expected marginal utility of Goal2 and Goal3 because sparse beam sampling creates view holes. But Goal2 has residual probability gain even at 2500 rays. This is *sensitivity rescoring of the fixed route*, not full optimizer rerun.

The hard binary occupancy threshold scenario (same D02 mean map with unknown blocker mean>=0.5, 2500 beams, 0.25-cell ray step, scans on path) yields **3,868**, **+5**, **+2** newly visible **predicted-free unknown** cells at G1,G2,G3; total **3,875 of 3,876**. This model assumes the LaMa predicted free geometry is correct and ignores probabilistic uncertainty, so it does not settle true future sensing.

## 2. What Goal2/Goal3 gains mean under 2500-ray probabilistic visibility

- Goal2 weighted +135.03: **128.09 weighted units** are increases in probability of cells with previous visibility probability >0.05; just **4** unknown cells move from p<0.05 to p>0.5. Spatially most of the increase (125.73) is on the **upper right** unknown area. Mean scenario gain 118.59, pessimistic scenario gain 173.38.
- Goal3 weighted +15.17: **15.14 weighted units** boost cells already partly visible, 0 new cells go p<0.05 to p>0.5. All 15.17 units are upper-right. Mean scenario +13.46, pessimistic +19.16.
- The independent-cell occupancy attenuation / pessimistic LaMa variance hypothesis makes it possible for a new *viewing angle* to increase the probability of seeing a region that already looks nominally empty. This is a hypothesis, not empirical calibration. G2 treats this confidence boost as if it were the same category of information gain as revealing entirely new region, weighted by 0.14 s-equivalent per cell.

## 3. Full 2500-ray planner rerun demonstrates search-pruning failure

Unchanged code/weights, only G2Config.planning_beams modified from 128→2500:
- Full optimizer selected **3 goals** core (168,123)→(171,138)→(183,153), total 7.8m, cost **129.08 s-equivalent**, 16.315s planner time.
- But a **feasible two-goal chain** from original G2 run (171,138)→(180,150), already evaluated with *the same 2500 probabilistic ray model and default costs*, costs **128.39 s-equivalent**, strictly better. This shows the full 2500-beam search misses a lower-cost solution.
- Root cause verified: the G2 source sorts preliminary candidates by **stationary** gain / travel cost, keeps only the top 12 (branch_width) before evaluating route-integrated pathwise visibility. With 128 beams original G1 (171,138) preliminary rank=**8**, admitted. At 2500 rays the same goal is rank=**40**, rejected. Original manually preferred VP1 (168,135) is rank 26, also rejected at 2500. Candidate (171,138) actually yields pathwise full first-leg cost **139.54 s-equivalent**, vastly below top one-goal discovered in dense rerun **191.62**. Thus the greedy prefilter and beam width cause missed solutions.

## Scientific conclusion

The extra goals are mechanically explainable but *not yet scientifically justified as relatively optimal*:
1. **Goal2**: some hypothetical benefit from reducing uncertain occlusion remains. But when mean-map geometry is taken literally, only +5 distinct predicted-free unknown cells are newly projected visible; the extra +135 expected weighted units are mostly a confidence boost, not new territory. Its real utility is uncalibrated.
2. **Goal3**: at >=250 beams, same route is worse than ending at goal2 under G2's own objective. Its original choice at 128 rays is not robust.
3. **Search**: current G2-V1's 12-branch stationary prefilter can discard pathwise-good goals; full densified run selecting three goals while a known two-goal solution has lower cost proves this is a practical failure.
4. No actual GT scan was used to pick/score goals in this audit. Need separate predeclared GT offline validation for real sensing effectiveness, and multi-decision controlled comparisons to assert gains.

## Minimum repair before accepting G2-V1

- Candidate pruning must include a **route-integrated** benefit proxy, not exclusively endpoint stationary gain. Keep structurally diverse candidates, bounded search plus a 1/2-goal fallback that is evaluated exactly, so an available simple chain is never dominated without explanation.
- Increase beam resolution for final choice verification (possibly coarse-to-fine) and check stability of included goals, not just positions.
- Distinguish **new region discoverability** from **confidence increase in already partly visible cells**; tune/scientifically justify weights, stop overhead and risk penalties independently of D02.
- Replan after actual new LiDAR reveals walls; no frozen plan execution past a changed G1/LaMa snapshot.

All results are strictly offline D02 projections; the original native Run04, frozen 128-beam G2 plan, and G2-V1 planner source remain unchanged.
