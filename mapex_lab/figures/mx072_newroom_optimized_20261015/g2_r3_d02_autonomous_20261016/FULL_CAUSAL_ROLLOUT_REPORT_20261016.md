# G2-R3 — Independent full causal D02→termination offline exploration

Date: 2026-10-16 · Source: native New Room Run04 D02 · **Negative room-completion outcome**.

## Verdict

**G2-R3 R3 Stage A + Stage B successfully executes an uninterrupted sequence of sensing, new LaMa G1/G2/G3 inference, persistent Room Node V4-R4.1, persistent Structural Room Mission Memory, then independent current-snapshot replanning.** No manually chosen D02/VP1/G1/G2/G3 goals or future map were fed to the robot planner.

But **the room was NOT fully explored**, and the policy fell into a left-side viewpoint oscillation. The simulator's predeclared safety guard stopped it after two successive legs with no new observed cells anywhere. This **`STALLED_NO_NEW_EVIDENCE` is an external test harness stop**, NOT a G2-R3 room completion verdict.

- **9 goals**, **36.4m** knownfree-grid travel, **77 LiDAR scan poses**.
- Original D02 Room1: **6,564 cells**; initially **3,889 unknown**.
- After end: **280 original-room unknown**; **3,609/3,889 observed = 92.80%**.
- **GT-only retrospective evaluator:** 11 of 280 are actually free, 269 are walls. Of the 3,507 originally unknown GT-free cells, **3,496 became observed = 99.6863%**, but 11 remain unobserved: **NOT all free completed**.
- Remaining GT-free unknown form 4 cells in the small left obstacle pocket (bbox core r172–178 c107–117) and 7 in right pocket (bbox r183–189 c194–203). The upper interior cluster r169–173 c149–177 comprises 145 GT occupied unknown. These are evaluator-only labels.

## Trajectory — all goals chosen by G2-R3 at the time

| Reached goal # | (x,y)m | Total distance | Scans cumulative | NEW observed in originally accepted D02 Room1 | Remaining original Room1 unknown | Next plan from G2-R3 |
|---|---|---:|---:|---:|---:|---|
| Initial D02 | (11.1,14.7) | 0 | 0 | — | **3889** | (13.5,17.1) |
| 1 | (13.5,17.1) | **4.8m** | 10 | **2996** (2944 free, 52 wall) | **893** | (11.5,18.7) → (19.9,19.5) |
| 2 | (11.5,18.7) | **8.4m** | 18 | **275** (262 free, 13 wall) | **618** | (19.9,19.5) → (11.9,16.3) → (20.7,14.7) |
| 3 | (19.9,19.5) | **20.4m** | 42 | **329** (285 free, 44 wall) | **289** | (12.7,16.3) |
| 4 | (12.7,16.3) | **30.8m** | 63 | **9** (5 free, 4 wall) | **280** | (12.3,16.7) → (12.3,17.1) |
| 5 | (12.3,16.7) | **31.6m** | 65 | **0** | **280** | (12.7,15.9) → (12.7,17.1) |
| 6 | (12.7,15.9) | **32.8m** | 68 | **0** | **280** | (12.3,16.7) → (12.3,17.1) |
| 7 | (12.3,16.7) | **34.0m** | 71 | **0** | **280** | (12.7,15.9) → (12.7,17.1) |
| 8 | (12.7,15.9) | **35.2m** | 74 | **0** | **280** | (12.3,16.7) → (12.3,17.1) |
| 9 | (12.3,16.7) | **36.4m** | 77 | **0** | **280** | Not recomputed: safety guard fires after two consecutive legs with **zero GLOBAL new evidence** |

The last **5.6m** of travel (after arrival #4) did not reduce original-room unknown. A smaller amount of global map information was collected on legs 5–7, but legs 8 and 9 yielded zero new cells anywhere, triggering the stopped status.

## Strict causal and implementation boundary

1. **Initially frozen planner:** G2-R3 Run04 D02 stage B, goal core=(171,135), xy=(13.5,17.1). Source file `sequence.py` SHA-256 pinned to **`ae9de4983086000a94237ef4b511cfc937a3e42de7b87b0434ae496f10d3bab3`** and checked after each replan. It was **never edited during this test**.
2. A separate long-lived **hidden-GT synthetic LiDAR subprocess** reads `gt_map.png` and applies 2,500 360° rays, 20m range, sampled at 0.25-cell increments; robot scans at every 0.5m on the **already G2-chosen knownfree route**, including endpoint. The subprocess only returns new LiDAR-observed raster labels, not the GT map or future regions. G2, R4.1 and LaMa subprocesses never read GT. All nine attempted knownfree routes had no hidden-GT wall collision. These are simulated grid routes, not physical safety guarantees.
3. After each reached goal, run all **three original LaMa models** with the correct `default_map_eval` transform, then persistent original V4-R4.1 tracker. D01 and D02 tracker warm-replay produced **0 mismatched semantic pixels** versus frozen original source.
4. Advance previous persistent `mission_memory` through actual observed updates. Never clear unknown because the new Room Node shrinks or because LaMa says wall. R3 structural reasoning and multi-goal optimizer receive **only the current snapshot**, with same-snapshot hash verification, and produce a full new conditional goal sequence.
5. Execute **the first goal only**, then replan; do not blindly continue frozen remaining goals after new observations. This is autonomous *goal-arrival* replanning, **not** event-triggered replanning after every LiDAR scan during a leg.
6. Freeze all nine choices and observed maps; a **separate post-hoc GT evaluator** then classifies remaining unknown free/wall solely for scientific comparison, not for plan selection.

### Runtime on DELL (not native robot)
- LaMa three-model inference total: **164.65s**.
- R4.1 detection total: **3.95s** after 8 updates.
- Structural/memory stage total: **21.86s**.
- G2-R3 planning total: **165.46s**.
- Driver total ~364s elapsed, excluding retrospective evaluator/render. Robot travel *time* not measured; 36.4m is map-grid length only. Significant optimization required before on-robot execution.

### Why R3 failed to finish

The R3 structural mission retained **288 unknown** at the last full replan (63 in current accepted room, 225 outside it). Geometry still reported a 52-cell `POSSIBLE_OPEN_SPACE` region with 176 conditional current-map witness viewpoints. The planner repeatedly alternated nearby viewpoint routes to satisfy its nominal structural-coverage objective, even though actual LiDAR revealed no new original-room evidence (and ultimately no global evidence). It does not yet use **realized information gain, recently visited viewpoints, hysteresis, no-progress penalty, or a robust loop breaker inside R3**. Hence high predicted satisfaction can become endless local re-visits.

The true-free residual (4 left, 7 right) means actual exploration was **near-complete but NOT complete**. The original manually selected D02→VP1→G1→G2→G3 offline experiment reached 0 GT-free unknown over 34.2m; our G2-R3 causal run used 36.4m and left 11. These are single-instance descriptive comparisons, not a controlled proof of superiority/inferiority.

## QA / next revision (not performed during this run)

Engineering suite **31/31 synthetic PASS** after the run, unchanged source hash. Correct next research step is R3-R2: add **measured per-region information progress + visited viewpoint/tour memory**, detect oscillation before expensive return legs, require revised confidence/novelty for repeated viewpoint tasks, and retain deferred obligations rather than deleting them. Re-test on D02 and other New Room/Hospital snapshots with **frozen method and separate scientific QA**, never fit these thresholds to GT from this test. Also integrate actual G3 event-driven replanning and native sensor fidelity.

## Files

- `D02_R3_FULL_AUTONOMOUS_SCROLL.html`: contiguous page of D02 and every reached goal, with images.
- `AFTER_GOAL_*.png`, `FINAL_HINDSIGHT_GT_ONLY.png`: original D02 room mask, colored new simulated observations, routes and final evaluation-only GT labels.
- `summary.json`, `events.json`, `GT_RETROSPECTIVE_FINAL_EVALUATION.json`: machine-readable receipts.
- `stage_XX_g2r3_plan.json`, `stage_XX_structural_regions.json`, `stage_XX_postscan.npz`: current-snapshot path choices, structural reports and observed evidence.
- `source/g2_r3_d02_full_causal_driver.py`: independent rollout supervisor.
- `source/g2_r3_hidden_gt_sensor.py`: isolated raster-GT sensor.
- `source/g2_r3_d02_posthoc_audit_and_gallery.py`: hindsight-only evaluator.

**Verdict: CAUSAL PIPELINE PASS, ROOM FREE-SPACE COMPLETION FAIL (11 GT-free cells remain), REPLAN LOOP CONTROL FAIL.** This result is never to be presented as successful full-room exploration or a source-native MapEx run.
