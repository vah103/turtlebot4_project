# G2-R4 (goal-chain) native 2D on COM1 — Run 07 matched against Run 04/05

**Date:** 2026-10-17. **Result:** Run 07 **SOURCE_TERMINAL**, return code 0, 1,193 native steps, 284.199 seconds worker elapsed, no reported collision/fatal error. **Independent IR1/QA not performed**; this is an independently inspectable maker experiment, not evidence of a universal improvement. Failed experimental Run 06 is preserved.

## Experimental controls

- Exact New Room P1 input fixture `/work/com1/mx072_com1_work1_ready_r1_20261014/fixtures/p1_raw.npy` (203×263), source native padding 500 cells and starting world/grid pose **(631,631)**.
- **Unmodified** native MapEx LiDAR, 2D first-hit scan, original A* and pseudo trajectory controller, 3-model CPU LaMa G1/G2/G3, Room Node V4-R4.1, source terminal logic and native timestep scanning from optimized Run 04. No Gazebo, no physical motion, no hidden GT provided to planner.
- Same shared source file SHA-256: `fast_native_lama_3model.py=f396346129f3d24de63678e07cced6cee4394222f7f9d9026b83d881b878036d`; `room_native_integration.py=566a449560a03638af7fc0d736be031bac8e2c84fc4cdab059bf7f64a913354c`. Old G2 `sequence.py=ae9de4983086000a94237ef4b511cfc937a3e42de7b87b0434ae496f10d3bab3`, `commitment.py=290a03ec536a12b95ed525d496037f7042240fffd38c77d5e7e1c4b12174f370`, plus original G2-R4 `r4_adaptive.py` and `r4_episode.py`.
- Only **LOCAL goal-selection policy** changed: separate Python 3.12 sidecar consumes exactly the current snapshot saved by original native Room Node `room_r41/inference/decision_*_input.npz` and `*_semantic.npz`; it calls G2-R3.1 structural sequence and G2-R4 whole-chain value/prefix screening. Native worker Python 3.6 remains responsible for movement/sensor and Room Node gate. If R4 defers, the native worker falls back to original MapEx GLOBAL scoring, rather than faking room completion.
- **Source equivalence at D02 tick 12: 0 differing observed occupancy cells** across all three arms, matching frozen D02 source hash `de96490d2c94ca0b10ff7f359b2b8fe3b4bc9a8d5656194b9ceec8c13c539b23`. Evaluation ROI fixed to frozen D02 original Room Node mask, **6,564 cells**, of which **3,889 UNKNOWN initially**; mask used only for evaluation.

## Primary endpoint: first reaching <=270 UNKNOWN in original D02 room

The target represents **93.057% of initially UNKNOWN D02 room cells resolved**, not 93.1% of all 6,564 room cells. Each arm reaches essentially the same endpoint (remaining 269, 259 and 268, respectively).

| Arm | First tick | Steps since D02 tick 12 | Distance since D02 | Driving at assumed 0.3 m/s |
|---|---:|---:|---:|---:|
| Run 04 — Room Node + MapEx, local WFD | 102 | 90 | **24.861 m** | 82.870 s |
| Run 07 — Room Node + G2-R4 old goal-chain; MapEx fallback | 625 | 613 | **173.205 m** | 577.350 s |
| Run 05 — plain global MapEx | 767 | 755 | **213.513 m** | 711.711 s |

**G2-R4 Run 07 does NOT outperform Run 04.** Run 07 needs **6.97×** Run 04 distance to hit the same D02 room coverage; global MapEx Run 05 travels 8.59× Run 04 distance. Run 07 drives less than plain Run 05, but much farther than the currently accepted MapEx + Room Node local WFD policy.

## CPU/planning overhead and exploratory time estimate to same D02 endpoint

| Arm | Native model time | Room Node + native MapEx score time | G2-R4 policy time | Native per-step elapsed recorded through endpoint | Assumed travel + all recorded model/score/G2 compute |
|---|---:|---:|---:|---:|---:|
| Run 04 | 4.098 s | 2.701 s | — | **11.844 s** | **89.669 s** |
| Run 07 | 17.535 s | 14.921 s | **113.738 s** | **184.108 s** | **723.544 s** |
| Run 05 | 32.194 s | — | — | Not available in equivalent source log | **743.905 s** |

The last column uses `distance / 0.3 m/s + recorded model/room/G2 planning`, **not an actual physical robot stopwatch**; scan, saving, control-loop overhead and dynamics are excluded. An alternative total on the two Room Node-instrumented arms using **all native per-step wall time** is **94.714 s for Run 04** versus **761.458 s for Run 07** (movement time assumption plus recorded native step wall; neither constitutes physical timing). Run 05 has no equivalent full per-step wall log, so avoid claiming precise relative real-time ordering of Run 05 vs Run 07. All arms use the native scan cadence and identical movement step controller, **not a newly imposed exact 0.5 m scan cadence**.

## Why old R4 underperforms here

Run 07's D02 decisions were causal: at native ticks **12,25,54**, G2-R4 proposed LOCAL goals. At **tick 95**, G2-R4 returned **DEFER_ROOM** when the fixed original D02 room still had **320 UNKNOWN**. The robot then carried on with MapEx GLOBAL, doing other work before reaching <=270 at tick **625**. This leaves just 50 more unresolved original-room cells than the benchmark target; the large observed detour makes the opportunity cost of premature defer measurable.

The prior D02 R3.1 offline shadow study's **18.8 m / 39 scans** is NOT an equal-engine native closed-loop result and MUST NOT be quoted as if Run 07 achieved it. R4's conservative/no-positive-chain gate and its rejoin proxy need further examination; do not tune thresholds on D02 GT in hindsight.

Native LiDAR produces some real **observed FREE → OCCUPIED** corrections; a strict old G2 memory rejects those. Fresh Run 06 stopped after 25 steps on 4 such cells (execution failure preserved). A narrowly audited monotonic-adapter change accepts **only FREE → OCCUPIED** and records corrected pixel coordinates on each R4 decision, while continuing to forbid OCCUPIED → FREE, KNOWN → UNKNOWN or GT-driven clears. Run 07 ran normally with that adapter; observed corrections were retained in the per-decision JSON. This amendment belongs to the native adapter, **not to the original R3/R4 models**.

## Whole-map terminal diagnostics

| Arm | Total native steps | Total distance from original start | Worker elapsed | End core 203×263 UNKNOWN |
|---|---:|---:|---:|---:|
| Run 04 | 709 | 199.108 m | 95.905 s | 2,085 |
| Run 07 | 1,193 | 340.182 m | 284.199 s | 2,084 |
| Run 05 | 1,298 | 369.696 m | 129.359 s | 2,081 |

All three ended with native `MAPEX_NO_SELECTABLE_FRONTIER_NOT_GT`; not proof of fully explored GT. Core observed-map remaining unknown is similar but snapshot pixel-level parity is not claimed at the end.

## Machine artifacts and reproducibility

**COM1 source/work directory:** `/work/com1/mx072_g2_r4_native_20261017/`.

- Source: `g2_r4_policy_sidecar.py`, `g2_native_integration.py`, `explore_g2_r4_lama3_smallroi_run06.py` (same policy-bearing explorer used by Run 07), `native_g2_r4_3model_worker_run07.py`, `guard_g2_r4_run07.py`, plus pinned unmodified copies of shared Run 04 files.
- Result: `native_g2_r4_full_run07/G2_R4_GOALCHAIN_RUN07_visvarprob/`, including `global_obs/*.png`, `odom.npy`, `room_r41/*`, `g2_r4/decisions/*.json`.
- Outcome: `evidence/guard_run07_g2r4/terminal_receipt.json` (`SOURCE_TERMINAL`, return code 0), `START_MANIFEST.json`, `native_stdout.log` and resource-guard receipt.
- Analysis: `compare_run07_d02.py`, `evidence/RUN07_D02_SAME_COVERAGE_COMPARISON.json`, frozen source `evidence/reference/D02_G2_R31_CAUSAL_EVIDENCE.zip`.
- Failure case not overwritten: `native_g2_r4_full_run06/` and `evidence/guard_run06_g2r4/`.

**Limitations:** One fresh Run 07 vs earlier independently completed Run 04/05; same host and exact shared backend, but not randomized replicate runs or independently QA-reviewed. Old G2-R4 includes original MapEx scoring before replacement by G2 in LOCAL; that redundant computation is measured, not hidden. The shared LaMa pipeline is the Run04/05 3-model cropped optimized variant, not the historical 4-model MapEx. Inferred room masks/mission ROI are not ground truth and G2 never gets GT. Do not claim physical robot safety from 2D A* clearance.

**Scientific conclusion:** The old goal-based G2-R4 is **not competitive with Run 04** on this controlled D02 time-to-coverage test. Address premature DEFER and avoid expensive whole-chain recomputation, then validate R4.1/R4.2 separately, without tuning on the held-out evaluation data.
