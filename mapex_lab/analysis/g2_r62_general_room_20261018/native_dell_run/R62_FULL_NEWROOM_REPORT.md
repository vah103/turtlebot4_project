# G2-R6.2 R2 — DELL native New Room full-run audit

**Date:** 2026-10-18. **Run:** `NR-G2R62-03`. **Comparison:** `NR-RN-04`, which previously passed DELL vs COM1 25/25 parity.  
**Status:** `SOURCE_TERMINAL`, 0 exit code, the original MapEx frontier-based terminal gate. **NOT ground-truth-certified exploration completion.**  
**Independent conclusion:** G2-R6.2 fixed the R6.1/R62-R1 dead-end/oscillation and is native runnable, but its **whole-map distance is WORSE than the RN-04 baseline** at almost identical observed coverage.

## 1. Full native run at comparable actual LiDAR coverage

| Outcome | RN-04 base | G2-R6.2 R2 |
| --- | ---: | ---: |
| Native outcome | SOURCE_TERMINAL | SOURCE_TERMINAL |
| Native motion steps | 709 | 846 |
| Observed P1 coverage (known / 53,389) | 51304 (96.0947%) | 51308 (96.1022%) |
| Native travelled distance | 199.108 m | **236.903 m** |
| DELL guard wall-time (single trial) | 370.850 s | 351.463 s |
| Peak process-tree PSS | 2119.7 MiB | 2246.1 MiB |
| Model/RoomNode scored decisions | 25 | 19 |

**Delta in whole-map distance: +37.796 m (+18.98%)** for G2-R6.2.  
Wall time delta: **-19.387 s** (R6.2 faster in this single run). This is not sufficient to prove performance reproducibility, as run CPU load/runtime noise was not controlled with repeated trials.

Terminal P1 occupancy state differs in only **33 cells** (including small explored/occupied differences); G2-R6.2 knows only **4 additional cells** by the final tick. This supports close final observed-coverage comparison, not identical ground-truth quality.

## 2. Equal measured coverage of the frozen initial D02 ROOM

The D02 room is fixed from the same initial observed and Room Node mask in both arms; it has **6564 cells** and **3889 initially UNKNOWN cells**. All milestones below were recomputed from the actually saved **global_obs PNG at every native scan**, not a speculative G2 raycast, and *never from GT*.

| D02 room UNKNOWN <=270 | RN-04 | G2-R6.2 R2 |
| --- | ---: | ---: |
| First native tick satisfying criterion | 102 | 98 |
| Real remaining UNKNOWN | 269 | 268 |
| Travelled distance after D02 tick 12 | 24.861 m | **22.832 m** |

G2-R6.2 savings at that fixed local milestone: **2.029 m (8.16%)**. This local positive result does **not** imply whole-map superiority.

**Accounting caveat:** comparing only saved decision snapshots yields a coarser 23.432m for R6.2; per-scan LiDAR snapshots detect its crossing earlier at 22.832m. Use the per-scan result above for a fair matched-coverage comparison.

## 3. Where the whole-map distance diverges

At fixed P1 known-cell milestones:

| P1 observed milestone | RN-04 path distance | R6.2 R2 path distance | R6.2 extra |
| --- | ---: | ---: | ---: |
| 90pct (48051 known) | 152.870 m | 193.853 m | **+40.983 m** |
| 95pct (50720 known) | 192.290 m | 230.762 m | **+38.472 m** |
| 96pct (51254 known) | 196.408 m | 234.580 m | **+38.172 m** |
| reference_final_P1_observed (51304 known) | 198.208 m | 236.603 m | **+38.396 m** |

This proves that the new policy's early-room local gain does **not** survive its later room/global trajectory. It does not yet identify which exact planner choice causes the extra distance; do not attribute every extra metre to Watchman without a decision-by-decision matched analysis.

## 4. Events and safety

- R6.2 structural event counts: S1_KEEP=102, S2_LIGHT_REPLAN=48, S3_STRUCTURAL_REBUILD=3
- Heavy MapEx/LaMa+Room Node epochs: 19 vs 25 baseline; G2-R62 watchman initial proposals: 10.
- Zero actual-progress viewpoint triggers: 2; native LOCAL-WFD residual fallback activations: 1.
- R62 R1 `NR-G2R62-02` was intentionally stopped (worker SIGTERM) after repeated one-cell goal oscillation. It must not be compared as a completed run. The canonical R62 R2 branch fixes it by subtracting predicted sight at the current pose and requiring **real** observed UNKNOWN reduction before another speculative local continuation.
- No S0 event in the R2 logs does not prove physical obstacle safety; native A* still checks executable FREE routes. Actual robot dynamics beyond 2D are not modeled.
- Preflight/LaMa 3-model model-smoke PASS, real native Python3.6 pyastar2d + Python3.12 G2 sidecar smoke PASS, 13/13 focused regression tests PASS. D01/D02 pre-choice observed/LaMa tensor/Room Node parity to RN04: `D01_D02_NUMERIC_PARITY_PASS`.
- No GT/future-snapshot planner input. Ground-truth occupancy is **not** used in this coverage report.

## 5. Scientific verdict and blockers

**R6.2 R2 = implementation and native closed-loop feasibility PASS; entire New Room travel-distance objective NOT MET.** It is useful as a controlled negative/diagnostic result and a better base for future redesign; not ACCEPT as a more efficient exploration policy yet.

Next review should analyze why later choices add roughly 38-41m by 90% observed map coverage, rather than tune arbitrary thresholds against New Room. Hospital/office/school room/corridor tests must use the **same exact config** before any generalization claim. Compute/time must be repeated, not inferred from one paired run.

## 6. Durable evidence locations on DELL

- Frozen New Room baseline parity: `/home/dell/DELL_NATIVE2D/runs/NR-RN-04/DELL_VS_COM1_FULL_PARITY.json`
- New runtime source, hashes, logs: `/home/dell/DELL_NATIVE2D/runs/NR-G2R62-03/{RUN_MANIFEST.json,worker.stdout.log,DELL_PARITY_AUDIT.json,RUN_RECEIPT.json}`
- Machine-readable per-scan P1 comparison: `/home/dell/DELL_NATIVE2D/runs/NR-G2R62-03/R62_FULL_P1_OBSERVED_AUDIT.json`
- Executed causal trajectory: `native_g2_r62_general_r2_run01/G2_R62_GENERAL_R2_RUN01_visvarprob/odom.npy` and `global_obs/00000000.png .. 00000845.png`
- R62 R2 source: `/home/dell/DELL_NATIVE2D/build/g2_r62_general_structural_r2_20261018/`