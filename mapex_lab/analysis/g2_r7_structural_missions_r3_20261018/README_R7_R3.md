# G2-R7-R3 — Structural Missions from Prediction + Simple Geometry

**2026-10-18. Status: OFFLINE IMPLEMENTED / UNIT-TESTED (10/10 PASS).**
**No R7 native MapEx 2D run. No IR verdict and no science gain claim.**

## User's D02 intention (NOT hardcoded)

Plan 1 when entering D02:
- first at the doorway/neck between predicted chambers to look into the
  apparently empty room on the right;
- second at the *deep bottom-left* corner, behind the observed/predicted
  central obstacle in the left chamber, opposite the entry approach.
  Do not stop near the divider when the rear-side obligation remains.

On new LiDAR evidence that the predicted empty right chamber has unexpected
obstacles, preserve still-safe useful Plan1 and prepare Plan2 asynchronously:
- first deep bottom-right of right chamber because there are more
  still-UNKNOWN cells in that half at the newly observed snapshot;
- then deep top-right for opposing/back-side views that the first leg misses.
The second corner is omitted when simulated whole-route visibility adds
neither a new unresolved UNKNOWN sample nor an unseen object surface.

No fixed 4-goal count. No D02 P1 goal-coordinate constants in planner.
This is one input example, not environment-specific parameter fitting.

## Algorithm

A. Accepted G1 RoomNode semantic + LaMa mean + observed occupancy define
   connected predicted-FREE room hypothesis.

B. Multiscale geometric erosion from frozen standalone R7-R2 room-geometry
   helper (NOT R4/R5/R6 planning logic) splits room into chambers at bottlenecks.
   It locates the connecting neck and safe-inset corner candidates.

C. Classify which observed structural segments are:
   - outer enclosure (not indoor object);
   - inter-chamber separator/divider (not an independent obstacle);
   - true interior occluder.
   Important: G1 ROOM label intentionally excludes walls/obstacle cells.
   Detection uses previous accepted room bounds, not only current ROOM cells.

D. Every goal fulfills a **discrete structural obligation**, not an
   Information-Gain ranking:
   - PORTAL_SURVEY: observe predicted adjacent chamber through neck.
   - ENTRY_REAR: go past entry-chamber obstacle and reach far-side deep corner,
     preferably far from the chamber-separating divider.
   - OTHER_CHAMBER_OPPOSING_FLANK: after *actual LiDAR* reveals unexpected
     occluders there, select geometrically opposing far-wall inset corners.
     Order distinct flanks by still-UNKNOWN cells observed NOW, not prewritten
     "lower-right first" rules. Drop redundant flank when no additional
     obscured surfaces/UNKNOWN are expected along the approach path.

E. Future hypothetic route may cross LaMa predicted FREE for mission
   *intention* only. Native motion is NEVER commanded by R7. Verified
   observed-FREE mask and the native inflated FREE A* remain required to
   dispatch each leg; unknown route is never a safety certificate.

F. Dual Plan manager stores the entry zone as a persistent first-LIDAR
   anchor (not current robot chamber), tracks new observed blockers against
   *earlier RoomNode ROOM and earlier LaMa mean*, and launches Plan2 on a
   background thread. Plan1 goal remains committed unless native path unsafe
   or the RoomNode WFD gate requires an exit. The successor is re-evaluated at
   handoff and remains only a goal intention pending actual native planning.

No GT, no Gazebo, no future snapshot used in planner.

## Actual frozen D02 input (New Room P1, 0.1 m/cell)

source:
 /home/dell/mapex_g2_r6_watchman_20261017/D02_R6_CAUSAL_MAP_ONLY.npz

observed map SHA256:
 de96490d2c94ca0b10ff7f359b2b8fe3b4bc9a8d5656194b9ceec8c13c539b23

Output (core P1 row, col):
- room has TWO automatically inferred chambers, portal (169,132)
- obstacle internal to LEFT chamber centered around (169,113)
- right border column ~130 is classified as DIVIDER, not another obstacle
- PLAN1: (169,132) -> (196,96), and the latter is behind the object and
  far from the divider (bottom LEFT rather than bottom near-divider).
- both intended endpoints have saved native-reachable observed-FREE path,
  but native inflated A* not yet executed by an R7 worker.
- R7-only mission-computation ~0.12–0.15 s (on this DELL; excludes LaMa,
  G1, native map input creation, subsequent controls and safety checks).

D03 and D04 are **frozen RN04 historical trajectory snapshots** and
NOT R7's actual closed-loop LiDAR after Plan1:
- D03: RIGHT lower (196,206) -> RIGHT upper (148,206).
  The right lower half has 366 UNKNOWN cells versus 126 upper half at D03.
- D04: RIGHT lower (196,206) -> RIGHT upper (148,206).
- D03 and D04 FREE-masks are the observational proxy (observed==0) because
  current native inflated reachability was not saved for these snapshots.
  These do NOT certify robot navigation or source parity.
- Plan2 anticipation unit test uses D03 evidence injected only after
  original D02 Plan1 is formed: 57 newly observed obstacle cells against
  D02 predicted FREE in the prior G1 ROOM; active Goal2 stays committed
  as Plan2 computes asynchronously. This is a deterministic **offline
  contract test**, not a true robot experience.

R7-R3 expected goals match the qualitative two screenshot demonstration,
but this is ONE dataset. This does NOT prove generalization; there are
geometry heuristics and a need for validation on different Hospital rooms.

## Tests

**10/10 PASS**.

  cd /home/dell/mapex_g2_r7_structural_missions_r3_20261018
  OPENBLAS_NUM_THREADS=1 python3 -m unittest -v test_r7_r3
  OPENBLAS_NUM_THREADS=1 python3 render_r7_r3.py

Tests cover (1) room structure/portal and two obligations, (2) left-rear
opposite-side geometry, (3) D03 right lower/upper ordering, (4) preserved
entry-zone identity while robot crosses chamber, (5) horizontal reflection
(not left-coordinate hardcoding), (6) empty room no forced 4 goals,
(7) no direct movement command, (8) structural event with *prior* G1 ROOM,
(9) Plan1 unaffected while Plan2 builds, safety hold,
(10) no hardcoded D02 goal coordinates.

## Files on DELL

/home/dell/mapex_g2_r7_structural_missions_r3_20261018/

- r7_missions.py: independent task-oriented planner
- r7_dual_plan.py: asynchronous Plan1/Plan2 supervisor
- geometry_reference_r2.py: frozen standalone geometry/raycast utilities
  only; code includes unused old R7-R2 planner class but R7-R3 does
  not call its choice or scoring functions
- test_r7_r3.py: 10 automated tests
- render_r7_r3.py: gallery builder
- evidence/R7_R3_D02_D04_SCROLL.html: self-contained iPad gallery
- evidence/R7_R3_D02_D04_CAUSAL_SNAPSHOT_AUDIT.json: source and all tasks
- evidence/R7_R3_UNIT_TESTS.log: test transcript

## Remaining scientific and integration limits

1. **Not native integrated**, no R7-run odometry or actual observed coverage,
   no direct 2D robot replays, no IR signoff.
2. No actual Plan1 trajectory-induced Goal1 LiDAR; D03/D04 are observational
   alternatives produced by RN04. The real R7 Plan2 might differ.
3. The current detector uses geometric corner and divider heuristics that
   should be stress-tested with irregular, concave, Hospital and mirror maps.
4. The safety certificate belongs solely to native MapEx G1+2D A*;
   the current algorithm does NOT perform physical dispatch or validation.
5. Simulated visibility is still model-dependent; selecting a second corner
   based on predicted surface complement can be wrong if LaMa misses walls.
6. Native integration must reuse the frozen configuration and original
   0.5-m scan cadence, preserve old runs and log at least:
   hypothesis revision, active goal, background successor, safety veto,
   actual observed coverage and LOCAL/GLOBAL traveled distance.

Do not report hypothetical coverage/distance as science wins. Only a
completed matched native R7 closed-loop run can support that comparison.
