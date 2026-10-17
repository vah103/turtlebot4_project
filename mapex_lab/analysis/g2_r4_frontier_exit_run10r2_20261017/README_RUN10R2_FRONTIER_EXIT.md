# Run 10-R2 — Frontier is an exit criterion, NOT a navigation goal (COM1 2D)

**Date:** 2026-10-17 · **Status:** native 2D source-terminal, maker tested, **not** independently QA ACCEPT. Run 09 and baselines are frozen and unchanged.

## User-requested rule

In `LOCAL`, preserve Run 09's LaMa/RoomNode/G2-R4 predicted-UNKNOWN viewpoint feature and exact goal arrival. Apply Run 04's LOCAL completion semantics: **return to MapEx GLOBAL only when the original RoomNode-scoped WFD finds no reachable LOCAL frontier**. While WFD frontier exists, G2 decides the movement goal, never the WFD center.

### Implementation details

- `room_native_integration.RoomRun.score` unchanged from Run 09; it confirms LOCAL only with a reachable native-WFD frontier in the current accepted room and returns GLOBAL with `LOCAL_WFD_EXHAUSTED_RETURN_GLOBAL` when none exists. This is a frontier-*presence test*.
- R4's `GOAL`: same route/viewpoint/LaMa predictions as Run 09.
- R4's `DEFER_ROOM` while RoomNode is LOCAL: **do not call `recover_global`**, and do not set a MapEx WFD goal. Use `g2_local_backstop.generate_local_viewpoint` over native-knownfree positions inside the RoomNode mask. Generate candidates independently of WFD points or scores, evaluating probabilistic predicted rays, actual connectivity and travel/visit penalties. If all scored rays show zero marginal visible UNKNOWN, use a separate G2 uncertainty-proximity geometry fallback (still from native-knownfree connected positions, without WFD candidates). The accepted G2 goal is validated again by native A* and reached exactly.
- If no G2 viewpoint can be produced while WFD still exists: **fail closed with `LOCAL_VIEWPOINT_UNAVAILABLE_WITH_REACHABLE_WFD`**, rather than falsely declaring the room complete or silently switching GLOBAL. This is deliberate.
- The independent original Run 10 R1 probe stopped at native tick 128 on this fail-closed condition: R4 DEFER, one reachable RoomNode WFD, but no positive predicted ray score at any candidate. The R2 fix adds the separate uncertainty-proximity viewpoint source. The frozen Run 10 R1 error log was preserved. R2 passes the exact R1 failure snapshot and the standard source/safety invariants. **Do not erase R1 or imply that its termination was normal.**
- RoomNode, LaMa three-model input, MapEx global controller and global 1m arrival threshold, native 2D raycast/A* and P1 map untouched. G2 exact goal arrival remains as in Run08/09.

## Run 10-R2 native COM1 result

| Metric | Run 04 (RoomNode + MapEx) | Run 08 (G2 exact knownfree) | Run 09 (G2 predicted, DEFER→GLOBAL) | **Run 10-R2 (G2 predicted, WFD exit only)** |
|---|---:|---:|---:|---:|
| Whole-map path length (m) | **199.108** | 310.183 | 282.224 | **277.293** |
| Native steps | 709 | 1099 | 1006 | **988** |
| Worker computation + simulation wall (s) | **95.905** | 222.820 | **265.030** | 289.170 |
| Observed core UNKNOWN final | 2085 | 2086 | 2083 | **2083** |
| D02 original-mask threshold first tick | 102 | 105 | 756 | **123** |
| D02 path to <=270 initially UNKNOWN remaining (m) | **24.861** | 25.079 | 208.173 | **30.508** |

**D02:** Frozen original room mask contains 6,564 cells; 3,889 initially UNKNOWN at native tick12. All evaluation arms match the initial observed snapshot bit-for-bit. Endpoint first reaching <=270 original-room UNKNOWN remaining: Run 10-R2 tick123 and 30.508m travel since D02, compared to Run 09 tick756 and 208.173m. Improvement in *this one room* is enormous; Run04 and Run08 remain about 5.5m shorter. Do NOT claim 100% room completion or statistical generalization.

**Whole map:** Run 10-R2 saves **4.931m (1.7%)** over Run 09 but takes **24.140s (9.1%)** longer COM1 worker wall. Observed map core UNKNOWN final 2,083 in both. Run04 still has much shorter global path; the new room rule alone is not a general trajectory win.

## Every LOCAL/GLOBAL gate, and who chose the goal

- **32 primary decisions**: 24 LOCAL goals, 8 GLOBAL MapEx goals.
- **24/24 LOCAL goals were produced by G2** and checked against same-snapshot LiDAR observed FREE and the native RoomNode-restricted walkability mask; all **24/24 were reached exactly in the saved odometry**.
- **6 R4 DEFER events became valid G2 backstop LOCAL goals** (5 cases with positive modeled ray value, one via zero-ray-gain uncertainty proximity).
- **0 G2 DEFER causing GLOBAL**.
- **6/6 LOCAL→GLOBAL transitions** carried `LOCAL_WFD_EXHAUSTED_RETURN_GLOBAL`, with exactly **0 eligible reachable RoomNode LOCAL WFD candidates** at those epochs: Run10-R2 primary decisions D08 tick186, D11 tick249, D16 tick408, D19 tick534, D23 tick666 and D31 tick895. All these GLOBAL goals are ordinary MapEx; frontier-as-goal is allowed only after leaving LOCAL, as in Run04.
- Predicted UNKNOWN viewpoint feature remains active: 6 new predicted targets and 5 continue-committed decisions in 24 G2 decisions. Predicted targets are not commanded as unsafe UNKNOWN traversals.

**Nuance:** The R2 variant changes more than a Boolean exit condition: it must introduce a G2-only backstop to provide an actual LOCAL movement goal after R4 says DEFER. Without this, the robot has no legal next target, as the R1 failed probe showed. Consequently the measurable gains reflect **both** frontier-based exit gating **and** new G2 fallback viewpoint selection. The backstop can choose a position near an unknown boundary geometrically, but never accepts WFD frontier coordinates or ranking as movement targets.

## Provenance and audit limits

- Run 10-R2 source-bound check: native terminal `SOURCE_TERMINAL`, returncode 0, 988 scanned/recorded poses, 289.17 worker wall.
- Five static tests PASS on Run 09 frozen D06/D24/D27, no UNKNOWN execution, G2 owns goals; additional exact source snapshot from Run10 R1 D07 PASS after uncertainty-proximity repair.
- Run 10-R2 final independent source audit `RUN10R2_LOCAL_EXIT_POLICY_AUDIT.json`: every local executed goal is G2; same-epoch observed-free and native scope PASS; route endpoint reached; all six exits from LOCAL have no reachable WFD at that time.
- Same Run09 baseline full comparisons, original D02 frozen ROI and initial-snapshot hash. Raw and processed data are preserved.
- **No use of hidden GT in online policy.** GT is part of simulation environment for LiDAR physics only. No full suite independent IR1 review, no multi-seed, no physical collision safety guarantee; WFD exhaustion is not 100% GT coverage.

## Source and artifacts

**COM1:** `/work/com1/mx072_g2_r4_frontier_exit_run10r2_20261017`

**DELL:** `/home/dell/mx072_g2_r4_frontier_exit_run10r2_20261017`

- Full native `native_g2_r4_frontier_exit_run10r2/G2_R4_FRONTIER_EXIT_RUN10R2_visvarprob/` — same-epoch images, observed/mean/semantic, per-tick planning, G2 and room decisions, odometry.
- `evidence/guard_run10r2_frontier_exit/terminal_receipt.json`.
- `reports/RUN10R2_LOCAL_EXIT_POLICY_AUDIT.json` and `evidence/RUN10R2_FRONTIER_EXIT_COMPARE_RUN04_05_07_08_09.json`.
- `reports/RUN10R2_G2_R4_FRONTIER_EXIT_FULL_SCROLL.html` — self-contained 32-decision longitudinal gallery with observed map, RoomNode overlay, G2 chosen goal, predicted viewpoint when present, mode and G2-backstop annotations. Original G2 goal not replaced with WFD on LOCAL.
- Source code: `g2_local_backstop.py`, `g2_r4_policy_sidecar.py`, `explore_g2_r4_lama3_smallroi_run06.py`, pinned native worker and guard.
