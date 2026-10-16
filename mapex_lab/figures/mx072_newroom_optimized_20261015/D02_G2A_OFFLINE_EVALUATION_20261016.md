# Run 04 / D02 — Frozen G2-A VP1→VP2 offline evaluation

Date: 2026-10-16. Original Run 04 and previously-selected viewpoint plans remain untouched. The original planned viewpoints were committed earlier to branch `mx072-g2a-run04-d02-viewpoints-20261016` before this GT evaluation.

## Source and causal rule

Original snapshot: native Run 04 decision 2 (tick 12), Room 1, robot padded row,col (647,611), room-ROI from G1 `semantic==1` connected to robot, 203×263 core at 0.1m/cell. Inputs: `room_r41/inference/decision_000002_input.npz` (observed 0 free, 1 unknown, 2 occupied; LaMa mean/variance), `decision_000002_semantic.npz`, root `gt_map.png` used ONLY as offline simulator occupancy, not as policy input. Original D02 Room 1 comprises 6,564 cells, including 2,675 observed free + **3,889 unknown**.

Frozen selection from the previous report: VP1 core (168,135), native (668,635), x=13.5m y=16.8m; VP2 core (171,132), native (671,632), x=13.2m y=17.1m. Both observed knownfree at D02. Selection predates this evaluation; no D03+ actual observations/results inform the goal positions. VP1 was estimated to see 3,840 / 3,876 predicted-free unknown cells at D02 using a nominal mean-map visibility check. It was NOT a native or scientifically certified prediction.

## Evaluation method

GT raster from `gt_map.png` cropped [500:703,500:763], white occupied. Use **2,500 full-circle rays**, max **20 m** range, 0.1m grid; trace GT occupancy to first wall hit with 0.25-cell samples along ray, record free-before-hit and wall first-hit. Mask newly observed cells against **frozen D02 unknown** and only report Room 1 cells for room metrics. This is a counterfactual discrete raycast approximation; **not source-exact native `range_libc` + polygon/flood-fill scan**, and not a model-driven robot rollout. Observed-free vs GT disagree in 18 source-grid cells globally, so small-cell results have raster/alignment uncertainty.

### A. Stationary scan at the selected points (NO scans while traveling)

| Stage | New unknown observed in Room 1 | Free | Wall | Residual unknown |
|---|---:|---:|---:|---:|
| D02 | — | — | — | 3,889 |
| VP1 | **2,311** | 2,298 | 13 | **1,578** |
| VP2 after VP1 | **+443** | 437 | 6 | **1,135** |
| Total | **2,754** | 2,735 | 19 | **1,135** |

VP1 individually resolves about 59.4%, both about 70.8%. Sensitivity to step: at 1.0 cells VP1 2,317 + VP2 429; at 0.5 cells 2,325 + 425; at 0.25 cells 2,311 + 443. Differences are within roughly 1.2% of original room unknown.

### B. Scan along the path (G2 pathwise comparison)

Compute shortest 4-connected routes on D02 observed knownfree within Room 1, with 2-cell square clearance inflation from any observed occupied/unknown. Route R→VP1 **4.5m**, VP1→VP2 **0.6m**. All route grid cells happen to be GT free. Simulate a scan **every 0.5 m traveled**, including endpoint. Initial R scan not repeated because D02 snapshot is post-observation.

| Stage | Number of simulated scans | New room unknown observed | Residual room unknown |
|---|---:|---:|---:|
| R→VP1 | 9 | **2,868** | **1,021** |
| VP1→VP2 | 2 | **+28** | **993** |

After scans along R→VP1, the majority of the additional static viewpoint VP2 visibility has already been revealed, so VP2 adds only 28 more room cells (0.72% of D02 room unknown). This is *not* a reason to claim the actual robot would do exactly this, as new observations should trigger G1/G2 replanning and potentially change the route before reaching VP1. Never interpret an offline precomputed route as physical safety certification.

## Why nominal vs actual differ

At D02, LaMa mean>=0.5 flags only **13 / 3,889** room unknown cells as predicted wall. The GT evaluator shows **382** of those unknown cells are actually walls; just **3** of the flagged 13 overlap those true unknown-wall cells. This thresholded mean map severely misses occluding structure in this snapshot, explaining why nominal predicted visibility from VP1 was an overestimate. This is a model/threshold diagnostic, not calibrated accuracy of LaMa.

## Implication

At this snapshot, **VP1 is useful, but does not nearly complete the room**. Do not pre-commit VP2 based on D02. Instead, execute or simulate the first bounded leg, continuously update scans, and **re-evaluate Room Node + remaining information before further goals**. Different objective functions and route assumptions would choose different next viewpoints. The test gives no measured total time, no native G2/full MapEx run, and no proof of globally optimal viewpoint selection.
