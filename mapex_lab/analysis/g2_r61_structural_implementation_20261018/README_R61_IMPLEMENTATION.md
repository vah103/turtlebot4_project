# G2-R6.1 — Structural Event-Driven Watchman R1
Date: 2026-10-18. Status: IMPLEMENTED AND TESTED OFFLINE ONLY. NO NEW NATIVE RUN.

## Changes relative to G2-R6 R1
- r61_structural_events.py observes real LiDAR occupancy transitions UNKNOWN->FREE/WALL, geometric 4-connected bridges, new FREE branches, supported portal candidates, disagreement with old LaMa, and loss of route safety.
- r61_plan_manager.py holds a versioned route. It rejects stale scan proposals, checks the robot has reached the active waypoint, blocks advancing an obsolete future-goal chain, and creates fresh light Watchman plans from the CURRENT pose.
- g2_r61_sidecar.py exposes CHOOSE / UPDATE / CHECKPOINT / STATUS / CLOSE. It does not control motion or provide ground truth.
- g2_r61_native_integration.py forks R6's native adapter. It captures every latest scan, processes background policy every ~0.5m, and uses a mandatory CHECKPOINT at goal arrival. Only the native observed-FREE inflated A* can validate an actual commanded segment.
- explore_g2_r61_lama3_smallroi.py forks the native 2D worker. S3 triggers the existing heavy LaMa + RoomNode + ROOM-WFD branch on the current observed epoch; LOCAL exit remains native ROOM-WFD exhausted.
- Frozen G2-R6, NR-RN-01, MapEx original, and journal were NOT changed.

## Event meanings
S0: immediately stop using an unsafe route. FREE->WALL correction or native invalid path overrides cooldown.
S1: routine predicted observation progress; keep useful safe route with cheap gain checks.
S2: important new FREE branch / tentative portal / discarded future-goal value; light Watchman rebuild, no new LaMa.
S3: verified observed-space connection, supported portal, new FREE outside accepted ROOM, or strong contradictory walls; do not dispatch next old goal; request coherent fresh LaMa+RoomNode.
S4: native accepted-ROOM / reachable local WFD says exit LOCAL. Never claim room completion from predicted visibility.
Tiny diagonal-only connectivity is NOT a valid observed bridge.

## D02 frozen causal offline evidence
Inputs are Run11 native D02 (same starting LiDAR observation as Run04), robot core [147,111], room 3889 UNKNOWN, original R6 plan 5 viewpoint goals, projected 15.5m route.
A no-change scan monitor call took around 0.0057 seconds on DELL in one trial: S1_KEEP.
Synthetic 40 new WALL cells in previously predicted FREE area produced S3_STRUCTURAL_REBUILD and the viewpoint checkpoint returned WAIT_STRUCTURAL_REFRESH, with zero future-goal ACKs.
Synthetic arrival at viewpoint 1 with unchanged map invoked a fresh light replan, NOT blind continuation. In that artificial scenario it selected the same next waypoint. No actual R61 post-viewpoint LiDAR exists yet.
These are diagnostic offline cases. The 15.5m is NOT actual room exploration distance, and projected coverage is not observed coverage.

## Test / preflight
Working directory: /home/dell/mapex_g2_r61_structural_20261018

    OPENBLAS_NUM_THREADS=1 python3 -m unittest -v test_r61_structural test_r61_manager test_r61_native_adapter
    OPENBLAS_NUM_THREADS=1 python3 probe_d02_r61.py
    python3 -m py_compile r61_structural_events.py r61_plan_manager.py g2_r61_sidecar.py g2_r61_native_integration.py explore_g2_r61_lama3_smallroi.py native_g2_r61_worker_template.py

Native adapter smoke uses a clearly labeled pyastar2d STUB ONLY in the DELL test harness because DELL lacks the actual COM1 native installation. Do not infer physical path-safety verification from that stub.

## Limitations and next gate
Thresholds are uncalibrated engineering defaults, not tuned to D02 goals. Official ROOM/CORRIDOR reclassification needs a new coherent LaMa+RoomNode epoch (light planner only treats stale mean as a tentative prior). Background execution has not been resource-priority or deadline audited on COM1. New native source fork is syntax-tested and adapter contract-tested but NOT run closed-loop. COM1 was offline during implementation.
Before any native science: source/interface QA, real pyastar2d verification, runtime and memory preflight, controlled launch from frozen same initial D02, and tests of event worker load, fresh room semantics, WFD mode exit and actual scan-driven route changes. Never use Gazebo, GT in planner or future LiDAR.
For scientific comparison, NR-RN-01 actual D02 distance to <=270 UNKNOWN in same frozen ROI was 24.861m. Do NOT compare this directly to R61's hypothetical route length. Re-measure real distance and total elapsed wall-time to same observed-coverage threshold only after a valid closed-loop run.