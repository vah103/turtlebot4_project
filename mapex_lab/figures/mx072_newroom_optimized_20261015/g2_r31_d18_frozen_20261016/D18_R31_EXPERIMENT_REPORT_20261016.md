# G2-R3.1 — Independent Decision 18 test (Run04 Room4)
Date: 2026-10-16. **Frozen method. No tuning for D18.**

## Verdict
**Causal offline pipeline PASS; exploration of every originally unknown GT-free cell by third goal; efficiency/commitment policy FAIL**. Two later goals cost 17.2 m but yielded zero new map cells. These were not adjusted or excluded from results.

## Inputs and execution
Native Run04 decision_000018, robot core (53,40) = xy (4.0,5.3)m, Room4 robot-connected V4-R4.1 component 4,039 cells, 3,206 initially observed free and 833 unknown. Initial mean/variance from 3-model LaMa and semantic from source D18. R4.1 state warm replay D01–D18 reproduced **18/18 exact semantic maps (0 different pixels)**. New decisions use chronological IDs 19–23. Source G2-R3/commitment SHA256 unchanged throughout, same as D02:
- R3 Sequence: ae9de4983086000a94237ef4b511cfc937a3e42de7b87b0434ae496f10d3bab3
- R3.1 Commitment: 290a03ec536a12b95ed525d496037f7042240fffd38c77d5e7e1c4b12174f370
- R3 Structural: a09e956dd58c935bb9177f0f888779a494bfafbfbc21c4d03b844fa3a86f7471

The initial G2-R3.1 plan automatically selected **6 conditional goal candidates**, nominal 19.6m: (4.8,2.5) → (0.4,6.5) → (0.4,6.1) → (0.4,5.7) → (0.8,5.3) → (4.0,8.1)m. No hand-selected D18 positions were injected. It executed one goal at a time, scanned synthetic LiDAR, updated three LaMa models, original R4.1, R3 Structural Mission Memory, and retained or replaced the remaining conditional goals.

The hidden-GT first-hit sensor was separated from planner. Reference-compatible raster simulation: 2,500 rays, 360°, max20m, .25cell sampling, one scan each .5m on observed-free route. GT was used as hidden environment only, and in the post-run evaluation; never as a goal-choice input. This is **not native range_libc LiDAR, physical robot control, or mid-leg event-triggered G3**.

## Results
| Actually reached | Goal xy (m) | Cumulative travel | Cumulative scans | New D18 room unknown observed | Remaining unknown | GT-free unknown remaining* | Controller action |
|---|---|---:|---:|---:|---:|---:|---|
| Initially | (4.0,5.3) | 0 | 0 | — | 833 | 760 | initial 6-goal plan |
| G1 | (4.8,2.5) | 3.6m | 8 | 682 | 151 | 78 | KEEP |
| G2 | (0.4,6.5) | 12.0m | 25 | 56 | 95 | 26 | REPLAN |
| G3 | (1.2,2.1) | **17.2m** | 36 | 35 | **60** | **0** | REPLAN |
| G4 | (3.6,8.5) | 26.0m | 54 | **0** globally | 60 | 0 | **KEEP despite 0 gain** |
| G5 | (4.8,1.3) | **34.4m** | **71** | **0** globally | **60** | **0** | DEFER_STALLED_WITH_UNRESOLVED_MISSION |

*GT-free residual values are from a strictly retrospective evaluator, not known by G2-R3.1 when choosing goals.

Final: of 833 original unknown, 773 observed (92.80%); 60 still unknown and all GT-wall. All 760 original unknown GT-free cells were observed already after **goal 3**, then robot traveled **17.2m for zero new information**. Stopping status is NOT robot-side ROOM_COMPLETE, only DEFER after two consecutive zero-progress legs. No global-optimality or scientifically proven minimal goal count.

## Failure mechanism
After G3, the planner again produced a sequence. It did not certify that the 60 residual unknown were walls, nor should it have used GT to do so. G4 traveled 8.8m with zero global new evidence. The controller's next-step check then preserved G5 on the current-map *predicted* visibility rather than comparing the **whole remaining sequence** against the recent failed observation. G5 cost 8.4m and again found zero global new cells, causing DEFER. This is a policy generalization weakness relative to D02 (where four goals were efficient).

The initial six-goal sequence was dynamically revised: the robot actually executed five different goals. R3.1 is not hardcoded to four or six goals. The failure suggests revising the general commitment validity rule to incorporate failed pathwise observation and per-region evidence rather than keeping all goals meeting a fixed 16-cell predicted-visibility threshold.

## QA and provenance
43/43 synthetic tests PASS unchanged after the trial. An initial aborted harness preflight replayed R4.1 through D18 then accidentally requested decision id 3; R4.1 correctly rejected non-monotonic IDs. The aborted attempt has been archived and **is not reported as a scientific outcome**. New harness assigns 19–23, restarts from original D18 and passes 18/18 preflight replay. No G2-R3/R3.1 code or method parameter changed.

Next: lock the D18 failure as validation evidence; improve the generic hypothesized-region progress model and remaining-chain utility on a separate development set. Do not tune R3.1 on this same D18 and then claim D18 as held-out validation.

Files: D18_R31_CAUSAL_SCROLL.html, D18_GT_ONLY_EVALUATION.json, rollout/summary.json, per-goal structure and committed-plan JSON, 2D map snapshots and source harness.
