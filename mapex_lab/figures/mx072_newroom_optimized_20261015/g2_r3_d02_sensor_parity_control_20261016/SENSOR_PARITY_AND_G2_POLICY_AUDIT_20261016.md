# G2-R3 Run04 D02 — LiDAR sampling parity + policy loop audit (2026-10-16)

## Core scientific conclusion
An earlier apparently favorable G2-R3 three-viewpoint result was **not** a causally continuous D02 run: its stage-two input was the prior hand-selected VP1 at (13.5,16.8)m, rather than the actual G2-R3 D02 goal (13.5,17.1)m. The favorable 3-goal plan was also executed **FROZEN** to test its sensor outcome; the new causal controller executes one currently selected goal and reruns LaMa3, V4-R4.1 and R3 after each arrival. Therefore a dynamically changed sequence is expected in principle.

There was **also an actual comparability bug in the independent test harness**: its synthetic 2500-beam LiDAR angles used `np.linspace(0,2*pi,2500,endpoint=False)` whereas the original manual benchmark's raycaster used the default `endpoint=True`. On the **same manual VP1 route** from D02, the two versions differed by **78 observed pixels**. Correcting the angular endpoints and float dtype to the original benchmark produced **0 differing observed pixels**, thus passing exact held-route sensor parity. This is a sensor **sampling convention mismatch**, not a proven coordinate-axis error, LaMa preprocessing error, or Room Node state replay error: R4.1 original D01/D02 warm replay was 0 semantic pixel mismatches in both experiments.

## Frozen-code controlled comparison

Both runs begin with identical frozen R3 plan at D02, goal (13.5,17.1)m. G2-R3 `sequence.py` SHA256 `ae9de4983086000a94237ef4b511cfc937a3e42de7b87b0434ae496f10d3bab3` is unchanged. The GT occupancy map is inaccessible to G2 and LaMa, used solely inside the hidden first-hit raster sensor and in a separate **post-hoc evaluator**.

| Metric | Previous angular setting (endpoint=False) | Corrected benchmark-compatible setting (endpoint=True) |
|---|---:|---:|
| Executed goals | 9 | 7 |
| Distance | 36.4m | 31.8m |
| LiDAR scan poses | 77 | 66 |
| Original D02 room unknown left | 280 | 267 |
| GT-free unknown left (EVAL ONLY) | 11 | 5 |
| GT-wall unknown left (EVAL ONLY) | 269 | 262 |
| Termination | STALLED_NO_NEW_EVIDENCE | STALLED_NO_NEW_EVIDENCE |

Corrected plan history, all goals self-chosen in causal sequence:
- Goal 1: [13.5, 17.1]m; cumulative 4.8m; original room unknown 891
- Goal 2: [12.3, 18.3]m; cumulative 7.2m; original room unknown 619
- Goal 3: [19.1, 19.5]m; cumulative 17.2m; original room unknown 324
- Goal 4: [20.7, 17.9]m; cumulative 20.6m; original room unknown 270
- Goal 5: [12.7, 16.3]m; cumulative 30.2m; original room unknown 267
- Goal 6: [12.3, 16.7]m; cumulative 31.0m; original room unknown 267
- Goal 7: [12.7, 16.3]m; cumulative 31.8m; original room unknown 267

**Even after sensor parity correction, G2-R3 still revisits nearby left-side viewpoints with no real new LiDAR evidence and stops only under the supervisor's `STALLED_NO_NEW_EVIDENCE` guard.** This guard is not G2's own room-completion condition.

## Causal geometric model fragility
- At original prior run after G3, one 52-cell region was classified `POSSIBLE_OPEN_SPACE` on the current LaMa map, giving G2 an estimated gain of approximately 47-50 cells from nearby candidate routes. A separate GT-only audit found all 52 cells were actually **occupied wall**. This is evidence of a mistaken predicted hypothesis, not data accessible to G2.
- The planner lacks visit/viewpoint history and realized-versus-predicted information-progress feedback, hence a costly sequence can repeatedly revisit regions after null scans. Historical Room Mission Memory preserves unresolved unknown but does not encode tried views, failure likelihood or route hysteresis.
- A 0.3m difference in initial VP1 plus different scan poses changes the newly observed map and LaMa/R4.1. Between manual VP1 and self-selected R3 first-goal in the earlier run: 136 D02-room observed pixels differed, 440 semantic map pixels differed, 162 of 889 shared remaining unknown crossed LaMa's 0.5 occupancy threshold, so the structural regions changed markedly.

## Method and boundaries
- This corrected run uses the old reference **synthetic** LiDAR sampling convention; neither version reproduces native MapEx range_libc/polygon rasterizer exactly.
- No GT/future observation used to choose any goal. Offline GT-free residual labels are strictly AFTER the run.
- Fully causal only **at goal arrivals**, not a G3 event-triggered mid-leg robot controller. Graph shortest-path knownfree travel is not real-world certified safe.
- The exact 7 vs 9 goal difference shows **sensitivity** to sensor sampling and subsequent predictions; it does not prove G2-R3 sound, robust or near-optimal.
- R3 engineering synthetic tests remain 31/31 PASS; scientific room completion and anti-loop behavior FAIL in both runs.
- Recommended next changes: canonical sensor parity test as gate; R3 mission-progress & failed-viewpoint memory, spatial hysteresis/visited suppression, after-scan hypothesis revision and justified DEFER, then locked-method unseen-map testing.

## Artifacts
- `D02_R3_FULL_AUTONOMOUS_SCROLL.html`: seven actual goals, evidence maps and hindsight.
- `summary.json`, `GT_RETROSPECTIVE_FINAL_EVALUATION.json`, `events.json`.
- `AFTER_GOAL_*.png`, `FINAL_HINDSIGHT_GT_ONLY.png`.
- `source/compatible_sensor.py` : benchmark-compatible hidden GT sensor (endpoint inclusive)
- `source/corrected_control_driver.py`: original frozen R3 code run with corrected sensor and independent output.
