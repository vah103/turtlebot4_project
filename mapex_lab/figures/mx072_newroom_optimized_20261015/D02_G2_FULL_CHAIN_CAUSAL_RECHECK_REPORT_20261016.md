# Run 04 / D02 — independent full-sequence G2 planning recheck

_Date: 2026-10-16. Original MapEx Run04 untouched. Branch created from `mx072-newroom-real-25-decisions-ipad-20261015`, source commit `e8b1c8a2b2672e5e39d4fd56b931c9f97c58de98`._

## What was corrected
The earlier D02 experiment selected a best **single viewpoint** and called it a minimal sequence, without optimizing over sequence lengths. This independent rerun solves 1/2/3-goal variants, allowing a one-goal solution if that actually qualifies. Crucially, every selection uses **only D02** observed map, mean, variance and V4-R4.1 room segmentation. NO ground truth, D03+, offline VP1 LiDAR, or post-VP1 LaMa/R4.1 influenced this search. Ground-truth-based retrospective evaluations are separate and were not read by the planning scripts.

## Inputs and assumptions
- Native current robot row,col (647,611) = (x=11.1,y=14.7)m; occupancy resolution 0.1m/cell; observed D02 Room1 robot-connected semantic==1 mask with 6,564 cells (2,675 observed free, **3,889 unknown**). Of these unknown, **3,876** predicted free by LaMa mean<0.5; **13** predicted occupied. Neither class is yet observed.
- Sample known-free viewpoints on 3-cell grid, 2-cell exclusion inflation around unknown/occupied, must be reachable from robot by 4-neighbor knownfree routing. This more conservative rule yields **200 candidate goals**, including original VP1 core(168,135).
- Search score = union of predicted ray-visibility along each tentative shortest knownfree route, scanned every 0.5m. Block predicted ray by observed occupied, mean>=0.5 unknown, or Room1 ROI boundary. Source LiDAR settings max 20m, 360°; **720 rays at 0.5-grid-cell sampling** used for search; key comparisons then separately confirmed using **2500 rays at 0.25-cell sampling**. No goal traversal through merely predicted-free unknown.
- All 200 one-goal candidates evaluated; 22,327 two-goal and **1,815,614 three-goal** combinations explored under a 9.3m reference path-length upper bound, with the start pose excluded as a goal in corrected search. This is exhaustive **within sampled, cost-bounded combinations only**; not global over all possible viewpoints / all arbitrary-length chains. Scans at route coordinates are approximate and not source-exact native range_libc LiDAR. No measured travel time or certified navigation.

## 2500-ray validated options at D02

| Plan | Goal sequence in (x,y)m | Number of goals | Known-free grid path | Nominal newly visible predicted-free unknown |
|---|---|---:|---:|---:|
| **A: ~99.5%** | **(13.5,16.8)** [original VP1] | **1** | **4.5m** | **3,858 / 3,876 = 99.54%** |
| **B: ~99.9%** | **(13.5,16.8) → (13.8,17.4)** | **2** | **5.4m** | **3,873 / 3,876 = 99.92%** |
| Alternative 2-goal B | (13.5,17.1) → (13.8,17.4) | 2 | 5.4m | 3,873 / 3,876 |
| One goal reaching B coverage | (14.1,17.7) | 1 | 6.0m | 3,873 / 3,876 |
| C: verified 100% *nominal*, not minimum-cost certified | (16.5,19.5) | 1 | 10.2m | 3,876 / 3,876 |

Comparison A→B: **+0.9m** length to gain **15 additional predicted-free unknown** by continuing from original VP1 to an additional goal. B is a real two-goal sequence chosen **at D02**, not a goal tuned with any later outcome. A genuinely has only one goal, not an arbitrary forced one-goal planner.

Shorter/weaker-coverage tradeoff: 95% threshold has a 2-goal route of 3.9m vs best 1-goal 4.2m. For 99% and 99.5% thresholds, one-goal original VP1 is the shortest route found among 1/2/3-goal sampled searches. For 99.9% nominal threshold, the two-goal B route is 0.6m shorter than a one-goal route reaching the same 3,873-cell projected coverage. 720-ray presearch suggested exact 100% on a 9.3m one-goal route; 2500-ray check saw **3,875 / 3,876**, showing that one-pixel claims near 100% are fragile. C is a verified example achieving nominal all predicted-free unknown, but its 10.2m is **not a proven shortest exact-100% route**.

## Interpretation for G2
There is no single mathematical optimum until an exploration-completion target and a travel/time/per-goal cost model are set. With a pragmatic near-complete **99.5% nominal** target, the original 1-goal VP1 choice is validated *within this heuristic and candidate pool*. With a stricter 99.9% target, two goals can be worthwhile. A predicted 99.5% does **not** mean the real room is 99.5% explored; 13 predicted-occupied unknown cells remain, and LaMa could be wrong about free/obstacles. Hence every proposed chain is **conditional**: if LiDAR reveals new obstacles, immediately reconstruct observed, rerun LaMa + V4-R4.1, and optimize the **full remaining sequence**, not only the next goal. It is invalid to continue a frozen chain through a changed map.

No source Run04/G2 native navigation, ground-truth measurement, physical safety certification or actual time savings are claimed.

## Artifacts
- `D02_G2_MINIMAL_CHAIN_COMPARISON_20261016.png`: source D02 Room Node panel 3 with both A/B paths, plus before/after **predicted** coverage maps.
- `D02_G2_CHAIN_ANNOTATED_ORIGINAL_PANEL3_20261016.png`: direct original panel overlay.
- `D02_G2_CHAIN_COMPACT_EVIDENCE_20261016.json`: 200-point scoring summary, 1/2/3 goal receipts, dense projected-visibility comparison.
- Full unabridged run-local search receipts remain separate on authorized DELL at `/tmp/MX072_D02_G2_CHAIN_REPLAN_20261016/`.
