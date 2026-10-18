# G2-R6.2 — General Room Watchman + Structural-Event Monitor

**R2 native candidate, 2026-10-18.** Experimental MapEx+Room Node V4-R4.1+Watchman for arbitrary room/corridor environments. Same algorithm and defaults for New Room and Hospital; *Hospital performance has not been measured*. No GT information is passed to G2.

## Scientific objective
Reduce travelled distance and elapsed time **to the same measured LiDAR coverage** compared with the original native 2D Room Node baseline, rather than returning a nice-looking proposed route. All execution goals must remain within observed FREE and native-knownfree-inflated A* scope. Local room termination is still RoomNode's reachable LOCAL WFD empty condition, not projected coverage.

## Critical R6.1 failure evidence
Frozen DELL Native2D `NR-G2R61-03` stopped at native step 44 when current ROOM had 3 reachable WFD frontiers, but the greedy planner rejected all 20 candidate viewpoints because the best expected gain/m was 10.87, below a hard-coded 12 minimum. At the last saved snapshot, actual initial-D02-room UNKNOWN=403; distance from D02=8.907m; failed to reach <=270.

## R6.2 R1 implementation (run NR-G2R62-02, CLOSED/FAILED)
- Remove arbitrary `minimum_new_weight=18` and `minimum_value_per_meter=12` gates from greedy goal acceptance; rank any finite, **positive marginal gain**.
- Rebuild heavy LaMa + RoomNode only for structural/topological changes (observed 4-connected bridge, supported portal or robot leaving accepted ROOM), not simply one FREE cell outside old room mask.
- Keep immediate S0 on **native active route unsafe**, but do not mislabel observed FREE→WALL corrections far outside the active safe route.
- If G2 gives no useful viewpoint with eligible ROOM-WFD frontiers, fall back to current native RoomNode local WFD goal (not global MapEx and not predicted UNKNOWN) with the original native A* safety check.
- First R62 native run reached matching D02 ROI coverage but later oscillated between two adjacent goal cells (195,197) and (195,198). The run was deliberately terminated and frozen; never call it successful. Source path `/home/dell/DELL_NATIVE2D/runs/NR-G2R62-02`.

## R6.2 R2 fix (current canonical code in this folder)
- Greedy pathwise visibility starts with predicted sight **from current robot position** already covered, not zero. This avoids granting a new nearby goal credit for speculative visibility that the robot could supposedly obtain while standing still.
- Every confirmed reached viewpoint compares the latest **ACTUAL observed** UNKNOWN→known cells in the accepted ROOM against the previous reached viewpoint's observed map.
- If no additional room cell was actually observed, don't blindly replan another speculative neighbor: return `RESIDUAL_ROOM_WFD`. The sidecar marks pending heavy; the native adapter enters `force_room_wfd=True` and uses the current RoomNode-scored LOCAL frontier as the next native goal. Native A* still validates it.
- The room-scoped fallback remains active for that ROOM identity; switching to a fresh Room Node room identity re-enables Watchman. The local WFD empty exit gate is unchanged.
- **Zero** actual new observed cells is a logical failure-to-learn signal, not a tuned absolute threshold. R6.2 still has other design constants (5 goals, 20 candidates, coarse 80-ray visibility, minimum topological evidence); therefore it is **not literally threshold-free**.

## Fixed execution conditions and QA
- Native 2D MapEx, New Room P1 native 203x263, 0.10 m/cell, 500-cell border, LaMa 3-model crop and original Room Node V4-R4.1. Python3.6 native runner plus Python3.12 isolated G2 sidecar.
- DELL native source and dependency SHA-256 preflight PASS, original `run_guard` memory/disk protection, no hard-coded RAM startup floor.
- Frozen DELL baseline: `NR-RN-04/DELL_VS_COM1_FULL_PARITY.json` confirms full parity with COM1. G2-R62 is a **different policy**: only pre-goal D01/D02 observed/mean/variance and RoomNode mode/pose can be parity checked.
- Pure-Python 13/13 focused tests pass on DELL, including D11 residual-goal availability and an artificial no-observation arrival. Python3.6 real-A* / Python3.12 sidecar process test PASS, including checked `RESIDUAL_ROOM_WFD` signal.
- R62 R2 native attempt: `/home/dell/DELL_NATIVE2D/runs/NR-G2R62-03`. Never overwrite the two older frozen run directories.
- **No scientific superiority claimed** until receipt state, actual coverage and full-scene metrics are confirmed. Never infer actual coverage from predicted visibility.

## Native observation and comparison
Start at D02 for every candidate trajectory, freeze **the SAME accepted ROOM mask at D02**, compute LiDAR UNKNOWN actually left inside it. At threshold <=270, compare distance since D02 and full elapsed wall clock, not predicted path length. Other ROOMs and Hospital require same settings and new controlled tests. Count LaMa re-inferences, R62 local replan events, native A* failures, fallback invocations, total mission distance and terminal status.

## Files
- `g2_r6_watchman.py`: greedy pathwise visibility with native FREE candidate scope; currently uses R62 R2 coverage baseline.
- `r61_structural_events.py`: structural events S0 safety, S1 keep, S2 light replan, S3 geometrical rebuild, S4 native LOCAL WFD exit; revised conservative triggering.
- `r61_plan_manager.py`: causal versioned/pose-validated plan manager, mandatory viewpoint-arrival checkpoint, measured observation-progress fallback.
- `g2_r61_sidecar.py`: isolated sidecar CHOOSE, UPDATE, CHECKPOINT, STATUS and CLOSE. Retains previous filenames for compatibility; protocol policy is G2-R62.
- `g2_r61_native_integration.py`: Python3.6-compatible native A* adapter with sticky per-room residual WFD fallback, no predicted UNKNOWN movement.
- `explore_g2_r62_lama3_smallroi.py`: separate native MapEx worker fork.
- `native_g2_r62_worker_template.py`: DELL Native2D Python3.6 launcher input (resource-protected run guard stages the worker).
- `test_g2_r62_regression.py`, `test_g2_r62_antiloop.py`: focused reproducibility and safety/behavioral unit tests.

Original `g2_r6_watchman.py` and `r61_*` dependencies are preserved under this new branch; please review changed semantics, native fail-closed behavior and observed coverage before claiming a method acceptance. The observed cell floor and structural consistency thresholds may need a non-environment-specific sensor-uncertainty rationale. Do not tune against D11 just to make this case pass.