# MX072 Run04 D02 → VP1 → G1 → G2 → G3: complete frozen-chain offline replay

**2026-10-16. Outcome: all 3,507 initially-unknown GT-free cells in the original D02 Room1 were observed by the end, but 256 originally-unknown occupied cells remain unobserved. Thus free-space completion PASS only by retrospective GT check; full occupancy-grid observation and robot-side room-complete certification NOT achieved.**

## Hypothesis tested

The earlier visually motivated G2-A strategy (chosen before this counterfactual rollout) is: D02 single-goal **VP1** world (13.5,16.8)m, then following fresh LaMa-ensemble/Room Node inference at VP1, frozen **[G1 (10.2,18.0), G2 (17.4,19.2), G3 (20.7,14.7)]**. Tests whether *executing exactly these previously frozen paths* reveals all unknown free parts of initially accepted Room1.

**Causal cutoff:** D02 current observed/mean/variance/R4.1 determines VP1; the previously committed after-VP1 snapshot and predictions determine G1–G3. None of these goal positions uses this test's future LiDAR returns or GT. In the present trial **the G1–G3 chain is kept frozen** and executed to completion after VP1 even if intermediate LiDAR updates would warrant a real G3 planner to reconsider. This is a predeclared trajectory evaluation, NOT a deployed dynamic replanner or native MapEx robot control.

## Sensor method

- New Room native P1 occupancy core 203×263 cells, 0.1m grid; robot initial core (147,111).
- Hidden GT from original Run04 `gt_map.png` crop rows500:703 cols500:763 used only as simulator's environment; artificial full-360° LiDAR: 2,500 rays, max 20m, 0.25-cell sampling to first occupied hit. Each path receives a scan every 0.5m, including end waypoint. Only ray-visible free cells and first-hit wall cells become observed. No direct GT mask supplied to LaMa, Room Node, or goal selector.
- VP1 initial shortest route on robot-connected D02 observed-free with 2-cell nonfree inflation; subsequent paths `VP1_DYNAMIC_G2_3GOAL_HIRES_MASKS.npz` had been selected/committed at VP1 in earlier branch. All four paths traversed observed-free cells at stage start and had 0 retrospective GT occupied-route intersections.
- **VP1 observed raster parity exactly 0 mismatched cells** vs earlier offline D02→VP1 frozen reconstruction. This checks that the first causal stage reproduced previous available information.
- Subsequent LaMa inference and original V4-R4.1 tracker are rerun after every reached goal for assessment. R4.1 D01/D02 replays show **0 differing semantic pixels** vs source; VP1 new semantic also has **0 pixel differences** vs the previously committed VP1 report. These outputs were not used to alter the frozen G1–G3 paths.

## Actual counterfactual LiDAR result

| Point reached | Goal (x,y)m | Additional leg (m) | Travel total (m) | LiDAR scans this leg | Newly observed D02 Room1 unknown | Remaining D02 Room1 unknown | Of those, GT-free unknown |
|---|---|---:|---:|---:|---:|---:|---:|
| D02 | (11.1,14.7) | 0 | 0 | — | — | **3,889** | **3,507** |
| VP1 | (13.5,16.8) | 4.5 | 4.5 | 9 | **2,868** (2,822 free; 46 wall) | **1,021** | 685 |
| G1 | (10.2,18.0) | 4.9 | 9.4 | 10 | **98** (80 free; 18 wall) | **923** | 605 |
| G2 | (17.4,19.2) | 11.2 | 20.6 | 23 | **589** (540 free; 49 wall) | **334** | 65 |
| G3 | (20.7,14.7) | 13.6 | **34.2** | 28 | **78** (65 free; 13 wall) | **256** | **0** |

Total: **34.2m**, **70** LiDAR scan locations. New observed originally-unknown cells **3,633 of 3,889** (**93.4173%**). Of the original unknown, GT labels **3,507 true free** and **382 occupied**; all 3,507 true-free were observed (100% free-unknown recall in this simulator). **126 wall cells** were hit by LiDAR; other **256 wall cells** remain unknown because occluded. These wall labels are **hindsight evaluation ONLY**.

## Updated G1 V4-R4.1 results (not ground truth)

| New observation at | Current R4.1 Room1 cells | Unknown in NEW current room | Unknown in original D02 Room1 | Original unknown outside CURRENT room |
|---|---:|---:|---:|---:|
| VP1 | 6,398 | 759 | 1,021 | 269 |
| G1 | 6,445 | 726 | 923 | 205 |
| G2 | 6,371 | 112 | 334 | 230 |
| G3 | 6,375 | **51** | **256** | 213 |

At G3, all 51 residual unknown in current R4.1 Room1 (including 43 in old Room1, 8 outside it) happen to be GT occupied in a retrospective evaluator. The robot does not know this directly. A further additional hypothesis exists in R4.1's report. Do **not** infer zero unknown or algorithmic COMPLETE from GT truth.

## Interpretation and limits

- This comparison **supports the previously conceived chain** as a method of revealing all hidden free cells in the originally accepted room on this one geometry. It does NOT prove route/goal count optimality, travel/time efficiency, or future robot autonomy. In particular, **the 13.6m G2→G3 leg was necessary to reveal the last 65 GT-free unknown cells for this frozen route**, which may or may not be efficient versus alternative routes.
- A related G2-V1 R2 causal rollout on D02 (different first goal and controller) stopped after 9.0m with 101 GT-free unknown remaining; this is descriptive, **not controlled evidence** of 34.2m manual route superiority in distance/time.
- Dynamic intended G2 should consider replanning the remaining sequence when new obstacles are observed; the present test intentionally keeps the preselected G1–G3 chain for clean offline comparison.
- Raytracing is on a discrete GT raster, not original MapEx native range_libc/polygon scan or actual robot sensor. Route safety (including real footprint, controller uncertainty) is not certified.
- A scientifically defensible robot-side leave-room criterion must use observed data, predicted completion uncertainty, potential future free space and the cost of confirming residual hypotheses — **not access GT**.

## Source and deliverables

All arrays, scripts and receipts remain on DELL under `/home/dell/mx072_g2_v1_r2_standalone_20261016/D02_MANUAL_G2_FULL_CHAIN_20261016/`. The original dataset and all previously frozen manual view positions remain unchanged. See `D02_VP1_G1_G2_G3_CAUSAL_6PANELS.png`, `fullchain_sensor_summary.json`, `fullchain_lama_r41_report.json`, `source/` for replay scripts, and `stage_XX_postscan.npz` for actually observed rasters.
