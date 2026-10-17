# G2-R6 R1 — Room Watchman Planning
Date: 2026-10-17. Experimental LOCAL policy for native MapEx 2D + Room Node V4-R4.1.

## Status
- IMPLEMENTED: CPU room-watchman planner, greedy pathwise coverage, bounded 2-opt, incremental no-LaMa scan update and route refinement, independent JSONL sidecar and patched native worker fork.
- VERIFIED: synthetic/frozen New Room D02 offline, adapter subprocess smoke. Nine tests passed on DELL.
- NOT VERIFIED: new native closed-loop run, physical safety, scientific improvement, Hospital generalization. COM1 is offline and DELL lacks the native MapEx/pyastar2d installation.
- Not changed: frozen NR-RN-01, NR-G2R4-STRUCT-01, journal, original native worker.

## Mechanism
1. Reuse current LiDAR-observed FREE/UNKNOWN/OCCUPIED plus same-epoch LaMa 3-model mean/variance and accepted Room Node V4-R4.1.
2. All selectable viewpoints and A* routes must remain inside connected ROOM and native-certified observed FREE. Predictions never open UNKNOWN for motion.
3. Pick <=20 spatially diverse reachable viewpoints, favoring areas with predicted unresolved visibility.
4. Select <=5 by greedy incremental not-yet-covered visibility along paths divided by added travel metres. Cached vectorized 80-ray nominal and stress predictions (0.2-m ray step) are only heuristic.
5. Improve order by bounded 2-opt, preserving nearly all estimated visibility; only native A* may certify executable paths.
6. Each ~0.5-m scan: update actual observed progress, drop redundant future goals, improve their order and detect walls / changed structure. Independent sidecar thread avoids blocking LiDAR/motion; skip updates rather than queue unbounded tasks.
7. Re-run LaMa/Room Node if the route is exhausted but LOCAL WFD remains, when a committed route is unsafe or there is material contradictory structure. Exit to GLOBAL exactly when reachable LOCAL WFD is empty, as in NR-RN-01.

## Frozen causal D02 result (offline, no GT in planner)
Input same native starting snapshot as the comparison runs, shape 203x263, robot core [147,111], room mission 3889 UNKNOWN.
SHA observed: de96490d2c94ca0b10ff7f359b2b8fe3b4bc9a8d5656194b9ceec8c13c539b23
SHA mean: 13bcb244d448f09fc7a76b7eaaefd6a61add1447de1e77256cc37bbd944deb04

R1 plan: 17 proposals, 5 viewpoints, 0.37 s G2-only, 15.5 m predicted route (16.5 before 2-opt), 3687 nominal/3111 stress UNKNOWN predicted visible out of 3889. A no-change scan update ~0.005 s, no LaMa. All selected goals and path cells within certified known-FREE.

CRITICAL: 15.5 m and 3687 cells are hypothetical model projections, NOT actual observed coverage. Invalid to claim superiority to NR-RN-01 at its measured 24.861 m equal-coverage threshold. No full native G2-R6 run has been performed.

## DELL tests
At /home/dell/mapex_g2_r6_watchman_20261017 :
  OPENBLAS_NUM_THREADS=1 python3 -m unittest -v test_g2_r6 test_sidecar test_native_adapter
  OPENBLAS_NUM_THREADS=1 python3 trial_d02.py

Adapter smoke uses a labeled pyastar2d stub on DELL only; actual COM1 worker MUST use original real pyastar2d.

## Native 2D handoff: PENDING (NO SCIENCE GO)
Source worker cloned to explore_g2_r6_lama3_smallroi.py, patched to take cached viewpoints without heavy LaMa and queue scan updates. Integration file g2_r6_native_integration.py and g2_r6_sidecar.py included. A COM1-only launcher native_g2_r6_worker_template.py awaits exact workspace transfer and preflight at /work/com1/mx072_g2_r6_watchman_20261017 with frozen native MapEx, weights and New Room P1 fixture.

Obtain independent native QA before running. Compare same-start full-closed-loop D02 to NR-RN-01 with exact frontier exit, wall times, actual distance-to-same-observed-coverage and map safety. No hand-selected goals or GT in planner. Conservative fail-closed if LOCAL WFD remains but no safe R6 viewpoint is found; this fallback requires explicit QA.

Thresholds (unvalidated): >=5% room semantic change, >=32 new walls that contradict strongly FREE LaMa, invalid known-FREE A* route. More cross-room evaluation required.

## Files
g2_r6_watchman.py — CPU watchman core.
g2_r6_sidecar.py — process API CHOOSE, UPDATE, ACK_GOAL, STATUS, CLOSE.
g2_r6_native_integration.py — native A* adapter and asynchronous updates.
explore_g2_r6_lama3_smallroi.py — forked MapEx native 2D exploration.
native_g2_r6_worker_template.py — COM1-only launch template.
test_g2_r6.py, test_sidecar.py, test_native_adapter.py — tests.
trial_d02.py, D02_G2_R6_OFFLINE.json — causal D02 trial and readout.
room_native_integration.py, fast_native_lama_3model.py, sidecar.py, v41src/ — copied frozen compatibility dependencies.

No prior runs or source evidence overwritten. This is an implementation and offline feasibility result, not a science-quality win.