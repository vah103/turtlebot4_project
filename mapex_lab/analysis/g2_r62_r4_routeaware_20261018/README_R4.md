# G2-R6.2 R4: room-exit and route-aware Watchman

Date: 2026-10-18. Controlled experimental design, not a validated better planner.

## Target

Retain native G2-R6.2 R2 (236.903m, 96.1022% LiDAR), correct causes found in R3 (279.211m).

- A: Recheck LOCAL after native inflated start recovers. At R3 D10/t264, 3 WFD frontier clusters pass MapEx size>10, but robot is on a natively inflated-blocked cell although observed FREE; original G1 A* source-validation rejects all. R4 marks the exit provisional while the existing GLOBAL native route remains in control. After >=0.5m movement, a newly safe native starting cell triggers a fresh G1 WFD and LaMa decision. No inflation relaxation or arbitrary emergency trajectory.
- B: Preview room visibility along plausible GLOBAL routes before an extra LOCAL goal. R3 D16 GLOBAL candidate (11,107) would cover nominal 626 predicted room unknown cells over 7.7m; the existing LOCAL first leg to (18,126) covers nominal 730 over 8.9m. GLOBAL preview is diagnostic only; the actual MapEx GLOBAL goal is not yet known at LOCAL decision.
- C: Swap candidate only with predicted *set* visibility dominance, not just increased cell count. R3 D03 alternative gains but loses 165 cells visible only from the R2 goal; D05 loses 2. The R4 strict offline gate rejects these substitutions. Native initial R4-A does not activate speculative candidate swapping.

## D10 root cause

At R3 D10 tick264 the current G1 room has 553 UNKNOWN (628 under fixed R2 room2 ROI). The observed FREE frontier cluster sizes are 42,28,15,9. Three qualify in native WFD. The robot core pose (10,156) fails the native 3x3 inflated planner-start finite check. At tick266, after moving along the existing native GLOBAL route, all three WFD centers become reachable from a valid starting cell. R4-A requests a fresh G1 decision rather than treating temporary inability to A* as a completed room.

## Implementation

- code/r4_exit_guard.py: causal and Python3.6-compatible exit guard.
- code/native_r4_room_integration.py and code/native_r4_explore.py: native policy overlay.
- code/r4_visibility_gate.py: conservative visibility-set dominance certificate.
- code/r4_offline.py and code/test_r4_offline.py: checkpoint probes and tests.
- evidence/R4_OFFLINE_CAUSAL_A_B_C.json: causal calculations (no future or GT used for online decisions).
- Deployment scripts are SHA pinned and run in separate immutable DELL Native2D run directories.

Initial QA: 12 focused offline tests PASS; Python 3.6 import/compile PASS, preflight PASS, 3-model LaMa smoke PASS. Full native outcome must be separately checked. Other runs remain unchanged.

## First native outcome: R2 + R4-A regression experiment

Run NR-G2R62R4-02: SOURCE_TERMINAL, 801 native steps, 222.4686 m, 51,308 observed P1 cells (96.1022%), D01-D02 numerical parity PASS. Local 109.9981 m, GLOBAL 112.4705 m. Crucially the clearance recheck fired **zero times**; the difference from R2's 236.9033 m cannot be credited to R4's exit logic. The R4 trajectory diverges at room 2 (D09 tick 200 vs R2 tick 196) despite D01-D02 parity; causal effect on performance from the new guard remains unproven.

## Completed native 2D comparison (2026-10-18)

| Run | Origin | Distance | LOCAL | GLOBAL | Decisions | Observed P1 | Exit-recheck fires |
| --- | --- | ---: | ---: | ---: | ---: | ---: | ---: |
| NR-G2R62-03 | R6.2 R2 | 236.903 m | 104.250 m | 132.654 m | 19 | 51,308 | n/a |
| NR-G2R62R3-02 | R6.2 R3 | 279.211 m | 120.339 m | 158.872 m | 20 | 51,308 | n/a |
| NR-G2R62R4-02 | R4-A on R2 | 222.469 m | 109.998 m | 112.471 m | 20 | 51,308 | **0** |
| NR-G2R62R4A-02 | R4-A on R3 | 280.348 m | 95.384 m | 184.963 m | 19 | 51,308 | **0** |

All four native runs ended with SOURCE_TERMINAL and P1 observed 96.10219%; neither of the two R4 runs activated the new exit-recheck rule. Both runs preserved D01-D02 numerical parity but diverged from the corresponding R2/R3 reference earlier than the target D10. Consequently, **do not claim any distance improvements caused by R4-A**. The 222.469m observation is a valid realized run, not an isolated causal effect of R4-A.

Exact R3 D10 replay through the real Python3.6 RoomRun.r4_recheck_due method was PASS:
- Tick264 pose (10,156): native inflated start invalid, no forced recheck.
- Tick266 pose (9,161): valid native inflated start, R4 issued RECHECK_OFFICIAL_WFD and wrote an event receipt.
- The replay uses later scans only when replay time reaches them; no future observation or GT was fed into the initial decision.

### Scope and next gate

- Native R4-A implements fix **A only** (provisional exit and movement-triggered G1 recheck), without weakly relaxing native safety.
- B remains an offline route preview; unknown actual future GLOBAL goal cannot be assumed.
- C remains an offline exact predicted visibility-set dominance filter; no candidate swaps are enabled in the native R4-A policy.
- Additional deterministic, event-triggering closed-loop evidence is required before accepting R4 as superior to R6.2 R2.

See ../../figures/g2_r62_r4_routeaware_20261018/ for the two real path scroll galleries.

## Final results and actual-path galleries

See FINAL_R4_RESEARCH_VERDICT.md for four terminal run distances, mask details, guard activation verdict and next stages. Two original native actual-path HTML galleries are embedded under evidence/ for R4-on-R2 and R4-on-R3. Source code and replay receipts are preserved.
