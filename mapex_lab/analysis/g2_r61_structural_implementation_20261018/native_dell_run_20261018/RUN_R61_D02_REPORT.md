# DELL native G2-R6.1 Run NR-G2R61-03 — scientific execution audit

**Status: FAILED / incomplete; no MapEx win claimed.**
Date 2026-10-18. User asked for full run on DELL using the Native2D mirror validated by `NR-RN-04/DELL_VS_COM1_FULL_PARITY.json`.

## Host and parity
- DELL Native2D, original frozen MapEx, same 3-model small ROI LaMa, Room Node V4-R4.1, New Room P1 map, 2 CPU threads.
- Pre-goal D01/D02 observed, LaMa mean/variance and Room Node mode/pose numerical parity to NR-RN-04: **D01_D02_NUMERIC_PARITY_PASS**. Do not expect G2-R61 future movement to match RN-04.
- All policy goal navigation must pass real native pyastar2d A* on observed FREE; no GT input to G2.

## Native outcome
- Run receipt: `FAILED` (return code 1), guard wall 98.06s, peak process tree PSS 2229.1 MiB.
- Terminal status `mission_complete=False` at step 44. Worker log exception: `LOCAL_VIEWPOINT_UNAVAILABLE_WITH_REACHABLE_WFD`.
- D11 Room Node still had **3 LOCAL WFD frontier candidates** but G2-R61 had no goal passing the greedy utility thresholds.
- At D11 planner computed **20 viewpoints** and 200 UNKNOWN in its CURRENT room mask, but no score >=12. Best ratio 10.87 (58.7 weighted gain over 4.6m plus 0.8m regularizer). This is a **policy threshold dead-end**, not proof that navigation targets do not exist.

## Actual observed coverage, frozen initial D02 mask
- Frozen D02 room mask: **6564** cells; initial UNKNOWN **3889**.
- At D11 (tick 45), frozen ROI UNKNOWN **403**.
- Actual path since D02: **8.907m**.
- Comparison reference NR-RN-04/COM1: reach <=270 UNKNOWN after **24.861m** since D02. This G2-R61 run **did not reach <=270 UNKNOWN**; cannot compare completion distance or declare savings.
- DO NOT confuse R61 D11's changing current-room mission UNKNOWN=200 with the fixed D02 ROI UNKNOWN=403.

## Replanning behavior
Structural event counts: {"S2_LIGHT_REPLAN": 1, "S3_STRUCTURAL_REBUILD": 3, "S0_SAFETY_HOLD": 6, "S1_KEEP": 4}.
Frequent S0/S3 events caused repeated LaMa+RoomNode re-inference and fresh Watchman plans within a few native steps. This is opposite the intended cheap cruise/continuous route refinement property.
The source had only offline tests; this full native result is a **negative integration/science finding** and must be addressed before another 'optimized R61' result.

## Next revision proposal (NOT applied to this frozen run)
1. Distinguish actual unsafe native A* routes from conservative stale-inflated-scope or FREE->WALL corrections outside active route; do not weaken real safety.
2. A positive-coverage but ratio-below-threshold viewpoint should trigger an explicit reviewed 'residual coverage' state, not unconditionally terminate LOCAL while WFD remains. This would be a **new algorithm revision** and must have a new run ID and independent tests.
3. Audit S3 false positives caused by 1-2 new FREE cells outside the *cached* Room mask; retain semantic and topology evidence. Keep Room Node/LaMa refresh event-driven and scarce.
4. Before a new full run, test real native Python3.6 A*, source hash, model smoke and same D01/D02 parity gate. No D02 handpicked goals or GT.

## Immutable artifact locations
- `/home/dell/DELL_NATIVE2D/runs/NR-G2R61-03/RUN_RECEIPT.json`
- `/home/dell/DELL_NATIVE2D/runs/NR-G2R61-03/DELL_PARITY_AUDIT.json`
- `/home/dell/DELL_NATIVE2D/runs/NR-G2R61-03/worker.stdout.log`
- This report and its machine-readable JSON audit, without altering NR-RN-04 or the failed NR-G2R61-02 run.