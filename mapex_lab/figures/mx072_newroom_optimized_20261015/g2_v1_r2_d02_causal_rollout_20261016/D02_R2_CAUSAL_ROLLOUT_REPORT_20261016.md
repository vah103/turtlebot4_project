# G2-V1 R2 — Run04 Decision02 causal offline continuation (2026-10-16)

**Verdict: PARTIAL, NOT ROOM COMPLETE.** Original Room1 at D02 had 3,889 unknown cells, and 389 remain after two conditional G2-R2 waypoints.

## Timeline, using only data available before each choice

| Event | G2 chosen goal (x,y) m | New scanned original-Room1 cells | Original Room1 unknown left | Current G1 Room1 unknown |
|---|---|---:|---:|---:|
| D02 | (15.0,18.0) | — | 3,889 | 3,889 |
| Reach G1, 7.2m, 15 scans | reached (15.0,18.0) | 3,408 = 3,322 observed-free + 86 wall | 481 | 207 |
| G2 replan from G1 | (15.0,19.8) | — | 481 | 207 |
| Reach G2, 1.8m, 4 scans | reached (15.0,19.8) | +92 = 84 observed-free + 8 wall | 389 | 125 |
| G2 replan from G2 | **[] / DEFER_NO_POSITIVE_SAFE_CHAIN** | — | **389** | **125** |

Total 9.0m, 19 scan locations, 3,500/3,889 initial unknown observed = **90.0%**. Frozen original D02 Room1 area 6,564 cells. Original R2 default-config one-goal result is used unchanged as first leg. D01 and D02 exact source G1 R4.1 replay had **zero semantic-pixel differences**.

## Offline sensor and new predictions

This is **not** native range_libc/polygon LiDAR or robot navigation. Hidden GT occupancy grid used ONLY by a synthetic first-wall-hit LiDAR sensor (2,500 rays, 360°, max 20m, 0.25 grid-cell step), scanned every 0.5m on each originally planned knownfree path. Only ray-observed unknown become observed-free/observed-occupied; map GT is not passed to planner. After each reached goal, recalculate LaMa mean and variance from original three models using source default_map_eval normalization/padding, and feed this with observed map to exact persistent V4-R4.1 tracker (D01 → D02 → stage1 → stage2); then run G2-V1 R2 on current robot pose and updated maps, with no parameters tuned. A GT collision was checked in the sensor executor, but neither route crossed a GT obstacle. Unlike a G3 event-based native controller, this experiment replans only after reaching each goal, not after every scan along the route.

## Outcome and why the room is not complete

- D02-original Room1 still contains **389 unobserved cells**. In a **separate hindsight-only GT evaluator**, they are **101 actual-free and 288 actual-wall cells**. This truth was never exposed to G2 when planning.
- The updated Room Node has **125 unknown**, but **272** of the original residual unknown are outside the new Room1 hypothesis; Room Node boundary changes are not the same as observed coverage. Most remaining original unknown belong to three clusters: 169 upper-interior, 116 far-right, 104 left-interior.
- At final robot pose (15.0,19.8)m, G2-R2 generated **240 current knownfree reachable candidate viewpoints** but its optimal-policy score chose none. Weighted current-room unknown was **143.2332**, baseline penalty **20.0526 s-equivalent** from uncalibrated 0.14s/weighted-cell. One 9.3m rightward detour would cost 37.2s equivalent at the assumed 0.25m/s speed, so planner defers; this does NOT mean no remaining exploration opportunities.
- GT-hindsight-only diagnostic: from 240 reachable observed-free vantage candidates, the union of 2,500-ray scans could observe **133 more** of the 389 unknown, including **all 101 actual-free unknown**. The other 256 GT-wall unknown remain unseen within that sampled set. A point (20.4,15.9)m could see 91 newly unknown (75 actually free) but is not a G2-selected goal and must NOT be treated as an online recommendation.

## Scientific interpretation

**G2-R2 successfully re-plans a new one-goal sequence from observation, but the two-stage run does not complete the room.** It terminates in a cost-driven DEFER, with meaningful remaining free unknown. Completion, cost weighting, semantic reclassification and possible deliberate additional confirmation of overlooked room pockets need method revision and controlled tests; do not retrofit weights using GT at D02. The four-panel image shows initial D02, actual simulated observations after G1/G2 and final residual hindsight GT labels. Ground truth was never used by G2 to select a goal.

Files: `D02_R2_CAUSAL_4PANEL_COVERAGE.png`, `D02_R2_ROLLOUT_SUMMARY.json`, `D02_R2_ROLLOUT_EVENTS.json`, `D02_R2_REPLAN_AFTER_G1.json`, `D02_R2_REPLAN_AFTER_G2_DEFER.json`, `D02_R2_ORACLE_DIAGNOSTIC.json`, `source/` reproducible adapters. Original Run04, G1, LaMa, and G2-R2 source unchanged.
