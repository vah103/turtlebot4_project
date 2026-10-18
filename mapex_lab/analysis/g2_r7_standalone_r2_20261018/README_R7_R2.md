# G2-R7-R2 — Standalone Structural Viewpoint Planning

**2026-10-18. Status: SOURCE IMPLEMENTED, 8/8 OFFLINE TESTS PASS, NATIVE INTEGRATION PENDING.**

## Goal and project boundary

Develop a genuine independent R7 planner, NOT R4+R6.2 nor MCTS. Reuse ONLY:
the current-time MapEx LiDAR observed occupancy, three-model LaMa prediction
mean/variance, accepted G1 RoomNode semantic map, a native-reachable
known-FREE mask, and the native execution safety authority.
The planner never sees future observations, simulator GT, manually marked
user goals, or fixed D02 goal coordinates.

Input frames: cropped 203 x 263 New Room P1 grid, 0.1 metres/cell.
Masks: observed 0 FREE, 1 UNKNOWN, 2 WALL; RoomNode semantic 1 ROOM.
No Gazebo, no ROS launch, no robot motion in this prototype.

## Algorithm (all newly written)

1. **Structural chamber identification:** room-specific predicted-FREE mask,
   multi-scale erosion of distance transform. First persistent geometric
   bottleneck determines chambers; geodesic propagation assigns each predicted
   FREE cell to a chamber. Portal candidates come from chamber boundaries.
   No fixed "two rooms", D02 wall x-coordinate or doorway position.
2. **Obstacle vs exterior:** separate exterior enclosure components from
   internal observed occluders. Sample each interior occluder's surface.
3. **Complementary view obligations:** for each chamber, cover UNKNOWN
   targets; for each internal occluder, observe from distinct directions.
   Credit both visible faces and circular angular separation up to 180 degrees.
   The whole approach path receives LiDAR credit; endpoint coverage is not
   added again if already seen during travel.
4. **Candidate generation:** chamber core, portals, principal extreme
   positions near room corners and object opposing sides. Keep diverse candidates
   in each chamber; goal count is NOT predetermined.
5. **Whole-plan beam search:** compare variable-length joint sequences by
   chamber satisfaction + opposing-surface coverage - normalized route and
   checkpoint costs. No R4/R5/R6 import or greedy R6.2 candidate policy.
6. **Plan intent versus motion:** hypothetical routes can use predicted FREE
   for structure reasoning only. Actual candidate motion requires LiDAR
   observed FREE, supplied native-reachable mask, and native inflated A*
   at dispatch. The output *never directly issues a robot command*.
7. **Dual-plan supervisor:** separately compute Plan2 from *newly observed*
   structure while preserving currently committed leg. At arrival,
   reject stale structure, invalid room, blocked predicted segments and
   unsafe execution. G1 room exit remains native reachable LOCAL-WFD empty.

## D02 same-time frozen observation

Read the exact D02 same-epoch snapshot at
/home/dell/mapex_g2_r6_watchman_20261017/D02_R6_CAUSAL_MAP_ONLY.npz
Observed SHA256:
de96490d2c94ca0b10ff7f359b2b8fe3b4bc9a8d5656194b9ceec8c13c539b23

R7 independently inferred:
- **Two chambers**, a structural neck at 7 grid cells of erosion.
- **Portal candidate (169,132)** core P1.
- **Two interior observed obstacle structures** after excluding outer walls.
- **Plan 1: (168,133) -> (199,129)** core P1; predicted 7.8 m of route.
- Predicted structural-obligation quality approximately 0.9962.
- G2 planning time approximately **4.9–5.4 seconds** on this DELL CPU,
  excluding LaMa, RoomNode and native controller.
- Both proposed endpoints have a connected observed-FREE grid path in the
  supplied native-reachable mask, but **native inflated A* has not verified
  the actual route** and this is NOT an executable robot command.

This is a meaningful advance over R7 R1, which produced multiple
near-neighbor G2 goals. However, **Goal 2 still lies near the lower interior
divider rather than at the user's desired deep lower-left corner**.
Therefore R7-R2 does not yet completely reproduce the requested viewpoint
selection behavior.

## Later RN04 historical snapshots — NOT R7 outcomes

D03 snapshot: (196,206) -> (156,104) -> (196,96)
D04 snapshot: (196,206) -> (157,101) -> (196,96)

At D03/D04, the planner proposes a distant bottom-right viewpoint,
consistent with seeking a complementary view of newly exposed objects.
But **these snapshots belong to the previous RN04 trajectory**, not to a
trajectory produced by R7 Plan1. They cannot establish that R7 would have
generated this Plan2 when the robot reached its own Goal1 or Goal2.
The D03/D04 known FREE mask is a proxy (observed FREE only), not the native
inflated reachability certification. Do not claim safety parity from images.

## Tests and artifacts

Folder on DELL:
/home/dell/mapex_g2_r7_standalone_r2_20261018

Test command:
OPENBLAS_NUM_THREADS=1 python3 -m unittest -v test_r7_standalone

**8/8 PASS**: D02 multi-chamber/portal, D02 planner deterministic, opposing
bearing evidence, predicted UNKNOWN does not grant execution, synthetic
opposing-view geometry, no hardcoded D02 goal, background Plan2 supervisor
and unsafe path veto, empty-room no spurious obstacles.
Run: OPENBLAS_NUM_THREADS=1 python3 r7_snapshot_gallery.py

Outputs:
- r7_standalone.py — all-new geometry, sightline and joint sequence planner
- r7_plan_manager.py — async Plan2 preparation and guarded handoff
- test_r7_standalone.py — 8 unit/synthetic tests
- r7_snapshot_gallery.py — frozen chronological RN04-only renderer
- evidence/R7_R2_D02_D04_SCROLL.html — standalone self-contained gallery
- evidence/R7_R2_D02_D04_SNAPSHOT_AUDIT.json — detailed metrics and provenance
- evidence/R7_R2_UNIT_TESTS.log — full 8/8 test transcript

## Remaining limitations / next gates

1. **Actual R7 closed-loop 2D has NOT run.** The goal arrangement is
   hypothetical under current LaMa/RoomNode snapshots, NOT real final coverage.
2. **Plan2 parallelism is unit-tested**, not measured as concurrent with
   native robot motion. Must integrate native adapter/sidecar checkpoint.
3. **No independent QA/IR1**, no measured improvement over NR-RN-04.
4. Predicted mean uncertainty is not calibrated, and line-of-sight discretization
   and fixed CPU budgets require cross-map sensitivity testing.
5. Distinguish walls, dividers and objects better in complex Hospital layouts.
6. Opposite-corner hypothesis should be verified by *real sensor rays from
   held-out observed scans*, not hand-picked coordinates or hindsight.
7. A native DELL run must preserve frozen MapEx+G1 engine/configuration,
   LaMa 3-model, 0.5-m scan cadence, known FREE A*, no-GT-in-planning,
   and use a new run_id without modifying existing runs.
