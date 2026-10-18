# G2-R7 R1 — Structural Occlusion-Aware Watchman + Successor Planning

**2026-10-18: research prototype implemented on DELL.**
**STATUS: OFFLINE TESTS PASS; NATIVE CLOSED-LOOP NOT RUN.**

## Research target

At D02 the robot should infer a short set of complementary room viewpoints
from its CURRENT structural prediction: ideally first observe the region near
the opening and then take an opposing viewpoint deep into the left-side room.
As later LiDAR reveals unforeseen right-side occluders, maintain useful
active objectives while preparing a Plan2 from complementary viewpoints.
No fixed D02 coordinates, object count, room-corner patterns or GT inputs.

## R4 + R6.2 + R7 mechanism

1. **R4 pathwise structural value**: visible occluder surfaces receive
   one credit across the ENTIRE travel path, not one per endpoint. Opposite
   approach directions have value if previously hidden faces become visible.
2. **R6.2 Watchman**: greedy incremental unknown coverage, nominal/stress
   LaMa visibility, bounded viewpoint sequencing and order improvement.
3. **R7 semantic fix**: G1 ROOM=1 explicitly EXCLUDES obstacles (OBSTACLE=3).
   Scan observed and predicted occluders NEXT TO the accepted room, not just
   cells labeled ROOM. Include G1 obstacle semantic.
4. **Structural candidates**: sample distinct sides of occluder geometry
   rather than requiring a hardcoded left/right/lower/upper corner.
5. **Two kinds of goal**: executable PROPOSAL only on actually observed FREE
   cells within caller-supplied native reachable FREE; non-executable VIRTUAL
   candidate may be predicted FREE but never becomes a robot movement command.
   Native inflated A* must still certify every dispatched leg.
6. **Plan1/Plan2**: R7Supervisor can prepare a successor anchored at an
   already-known-safe future endpoint while an active goal continues.
   At arrival it checks current LiDAR, room structural epoch, goal legality,
   native connected path, and newly redundant visits before promotion.
   Structural epoch drift or unsafe route vetoes successor activation.
7. **Room exit**: only G1's original LOCAL WFD reachable-empty condition,
   not predicted completion or negative R7 score.

## Reproduction on DELL

Directory: /home/dell/mapex_g2_r7_occlusion_20261018/

Commands:
- cd /home/dell/mapex_g2_r7_occlusion_20261018
- OPENBLAS_NUM_THREADS=1 python3 -m unittest -v test_r7
- OPENBLAS_NUM_THREADS=1 python3 r7_d02_replay.py

8/8 focused unit/synthetic tests PASS on DELL. Include complementary
view geometry, strictly observed-FREE candidate routes, virtual targets
not executable, async Plan2+goal-arrival checkpoint, structural epoch/safety
veto and G1 LOCAL WFD exit handoff.

Frozen D02 input:
 /home/dell/mapex_g2_r6_watchman_20261017/D02_R6_CAUSAL_MAP_ONLY.npz
Observed SHA256:
 de96490d2c94ca0b10ff7f359b2b8fe3b4bc9a8d5656194b9ceec8c13c539b23

D03–D05 snapshots belong to prior RN04 trajectory, NOT R7 closed-loop.
Their executable FREE proxy is deliberately non-native; cannot be considered
a native physical safety certificate. Each decision is planned from ONLY its
same-time snapshot, never another decision's future LiDAR or GT.
Every covered-cell number remains a PREDICTION, not actual sensor outcome.

## Negative finding and limitation

R7 R1 is NOT yet a successful reproduction of the user's desired D02 plan.
Initial D02 Watchman still assigns a series of goals near the right-hand
region rather than a short structure-balanced pair spanning the left
occluder and the middle/right aperture. This is a failure to prioritize
complementary obstacle-surface obligations sufficiently strongly compared
with local UNKNOWN gain and highly overlapping neighboring viewpoints.
Do not tune fixed threshold values to this single New Room snapshot.

Follow-up research needed:
- explicit sub-room and obstacle-face observation obligations across the
  active plan and prospective Plan2, without any manual D02 coordinates;
- reliable visibility simulation that does not reward beam-sampling artifacts
  at adjacent goal points, plus safety/clearance in corner candidates;
- plan successor computation using UPDATED current-time LaMa/RoomNode after
  contradicting evidence, without blocking robot motion;
- causal closed-loop R7 native MapEx 2D run and Hospital generalization;
- compare same measured coverage and total LOCAL/GLOBAL distance to RN04.

## Files

r7_structural_watchman.py: occluder surfaces, Watchman, speculative viewpoints.
r7_successor.py: asynchronous successor plan, safe arrival checkpoint.
test_r7.py: six tests, offline.
r7_d02_replay.py: D02–D05 historical snapshots only.
evidence/R7_RN04_SNAPSHOT_AUDIT.json: machine readable observations.
evidence/R7_RN04_D02_D05_SCROLL.html: iPad-viewable image gallery.
g2_r6_watchman.py + r61_structural_events.py: copies from frozen R6.2.

**STATUS: research candidate, not yet integrated with native worker, no
independent QA, not IR1 ACCEPT and no measured distance improvement.**
