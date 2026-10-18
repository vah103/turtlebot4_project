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
