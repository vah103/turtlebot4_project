# G2-R4.1 — Motion-Action Structural Coverage Search

Date: 2026-10-17 (ICT). Status: OFFLINE SOURCE-BOUND PROTOTYPE / MAKER TESTS PASS / NO INDEPENDENT QA OR NATIVE GO.

## What actually changed

R3 searches sequences of viewpoint candidates. R4 rescored their prefixes. R4.1 instead searches sequences of *movement actions* directly from the robot's current pose on known-free map. Neither past R3 goal nor hand-picked coordinates enter the R4.1 planner. This is Bouman-inspired motion/coverage planning on 2D grids, not Bouman MCTS or PLGRIM reproduction.

The robot state in the search is position, preceding heading, route cells, cumulative visibility bitsets (nominal and conservative LaMa), distance, turn cost, and return-to-anchor distance. The allowed actions are 5, 10 or 15 P1 cells (0.5/1/1.5 m) in one cardinal direction, including all centerline observed-knownfree cells. The search simulates LiDAR visibility at the intermediate scan positions (every 5 cells) along each primitive. Bitwise union does not award repeated cells twice.

After search, the best diverse candidate routes are rechecked using a denser raycast. Cost includes outward travel, turn proxy and known-free return distance. The answer is a *conditional route*, not a fixed itinerary or robot command. Only the first <=0.5-m route prefix may be proposed for the future execution controller. The return anchor for D02 is the present robot position, NOT certified MapEx rejoin.

No 3D Information Roadmap, MCTS, dynamic obstacle controller or calibrated occupancy probabilities. Room Node and LaMa influence predicted *value*, never traversal safety; UNKNOWN is impassable, no predicted-free shortcuts.

## Direct D02 snapshot results (not a closed-loop trajectory)

Stage 0 / first room entry: SHA de96490d2c94ca0b10ff7f359b2b8fe3b4bc9a8d5656194b9ceec8c13c539b23. Robot (147,111). R4.1 proposes ten movement actions (11.0 m hypothetical outbound), 751 successor expansions, nominal estimated visible-unknown union 3889 cells, conservative 3098 cells; first prefix (147,111) to (152,111), exactly 0.5 m. A diagnostic known-free return to robot start is 11.0 m. Wall time ~1.97 s on DELL, excludes LaMa forward.

Stage 1 / saved R3.1 post-goal-1 snapshot: robot (171,135), nine motion actions (10.5 m hypothetical outbound), nominal 609, conservative 479 cells, first prefix (171,135) to (171,140). Diagnostic return 3.5 m. Runtime ~2.09 s on DELL.

IMPORTANT: stage 1 comes from R3.1's real simulated scan trace, not R4.1's chosen stage 0 path. It is only an independent frozen input evaluation. The 11.0 m and 10.5 m candidate routes were not actually driven. Nominal 3889 is a model estimate of all mission UNKNOWN, not proof that those cells will actually be observed. R3.1's realized D02 first leg was 4.8 m and 2,998 scanned original-room cells: not comparable to R4.1's hypothetical values for scientific claims.

## Tests and limits

The 15 new R4.1 synthetic tests pass. The inherited 39 R4 tests also pass (54/54 maker tests). Tests include map-hash validation, no previous goals, no GT, observed-free traversal, occlusion-aware path sensing, union without double count, turn/return cost, bounded first prefix and DEFER when conservative value cannot pass. No independent QA yet.

Four cardinal movement directions and 5/10/15-cell action lengths are grid approximations; turning cost begins without real yaw and is not physically calibrated. Inflated free-space is not a complete continuous-robot collision certificate. The beam's max depth and expansion count are *search resource budgets*, not forced goal counts. The source R3 region generator/visibility implementation remains the exact frozen version.

No full MapEx handoff geometry, dynamic obstacles, native scan cadence validation, wall-clock timing comparison, real closed-loop replan or measured coverage savings. G2-R4.2 must connect the short prefix to a causal scan/compute/act simulation and make genuinely new observations along R4.1's **own** resulting path.

## Code and reproducibility

r41_motion.py: planner. r41_d02_demo.py: D02 source loader. test_r41_motion.py: synthetic unit tests. render_r41_d02.py: artifact rendering. D02_R41_ACTION_SEARCH_SCROLL.html: two snapshot visualization with reference R3 route drawn purple *only for comparison*.

From the DELL workspace (requires the frozen R3 package in PYTHONPATH):

    cd /home/dell/mx072_g2_r4_adaptive_20261017
    export PYTHONPATH=/home/dell/mx072_g2_r31_structure_commitment_20261016:.
    python3 -m unittest discover -s . -p 'test_*.py' -q
    python3 r41_d02_demo.py --stage 0 --width 16 --depth 11 --max-expansions 1700 --dense-beams 900 --output reports/r41_d02_initial_full.json
    python3 r41_d02_demo.py --stage 1 --width 12 --depth 9 --max-expansions 1100 --dense-beams 650 --output reports/r41_d02_stage1_replay.json
    python3 render_r41_d02.py

Source D02 evidence ZIP is already in the G2-R3.1 repository branch under mapex_lab/figures/mx072_newroom_optimized_20261015/g2_r31_commitment_d02_20261016/. The GitHub branch includes mapex_lab/mapex_g2_r3 for the R3 dependencies. The local script's default DELL absolute evidence path may be overridden with R41_D02_SOURCE_ROOT.
