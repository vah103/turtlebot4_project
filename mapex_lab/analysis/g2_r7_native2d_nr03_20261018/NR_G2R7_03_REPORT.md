# NR-G2R7-03 — Native2D New Room, G2-R7-R3 Structural Missions

**Date:** 2026-10-18. **Status:** SOURCE_TERMINAL, NOT independent QA/IR1 acceptance.
**Scope:** One autonomous fresh native 2D run on DELL, 3 LaMa models + RoomNode V4-R4.1, 0.5-m scan cadence, planning only in observed reachable FREE with native inflated A*, no planner GT. All prior run folders preserved.

## Result

| Metric | Control NR-RN-04 (MapEx + G1) | NR-G2R7-03 (MapEx + G1 + standalone R7) |
|---|---:|---:|
| Source terminal status | true | true |
| Actual tick count (odom positions) | 709 | 954 |
| Actual odometry distance | 199.108 m | **264.968 m** |
| Observed P1 cells out of 53,389 | 51,304 | 51,308 |
| Observed P1 fraction | 96.095% | 96.102% |
| First ≥51,304 observed P1 cells | 197.908 m | 262.873 m |
| Frozen D02 ROI (≤270 UNKNOWN), metres from tick12 | 24.637 m | 25.168 m |

R7 whole-map odometry is **65.860 m (+33.08%) LONGER** than the strong RN04 control, for essentially the same final observed coverage. At the exactly matched ≥51,304 known-cell threshold the increase is **64.966 m**. Both runs used the same source terminal condition, NOT GT oracle completion.

D02 alone was relatively close to baseline (+0.530 m to fixed ROI ≤270 UNKNOWN); most extra travel comes later. Approximate leg attribution by postscan scope: RN04 ~104.48 m LOCAL + 94.71 m GLOBAL; R7 ~131.49 m LOCAL + 133.36 m GLOBAL. Scope-boundary attribution is approximate, whereas odometry totals above are exact from odom.npy.

## Physical closed-loop sequence and evidence

- D01–D02 **bitwise-identical** input observed map, LaMa mean and variance, and G1 semantic vs RN04 (all mismatch counts = 0); same room context, so R7 goal outcomes are genuine policy differences.
- Tick 12: R7 Plan1 has **G1 (169,132), G2 (196,96)**, both native inflated known-FREE A* validated before dispatch.
- Tick 27: native ACK confirms actual physical arrival at **G1 (169,132)**. Worker forces a fresh three-model LaMa and RoomNode hypothesis refresh, while retaining still-useful **G2 (196,96)**. UPDATE evidence logs newly observed obstacles, background Plan2 preparation attempted during G2 travel.
- Tick 55: native ACK confirms actual physical arrival at **G2 (196,96)**.
- **Plan2 FAILED TO BECOME THE EXPECTED G3/G4 AT THAT POINT.** On refreshed native maps, the geometric bottleneck / chamber portal shifted toward the right. New occluders were assigned to the wrong local chamber, leaving R7 with no successor tasks. It therefore fell back to G1's native LOCAL WFD frontier. It did not immediately command the lower-right, upper-right goals that the user expected. Later native roaming does not cure this missing two-plan behavior.
- Later the run finished under MapEx's native no-selectable-frontier terminal condition at tick954. This is a completed source run, but not evidence R7 outperforms G1.

## Failure and hypotheses

The old R7-R3 retrospective D03/D04 image replay was drawn from **RN04's trajectory**, not the one produced by R7. It suggested plausible opposing right-corner goals, but the actual G1/LaMa observations after R7 Goal1 changed chamber partitioning and exposed a flaw in R7's structural invariance. Specifically the fresh inferred portal shifted away from initial D02's (169,132), so target-side objects were misassigned.

**Required next design:** maintain the initially accepted room-to-chamber hypothesis as a versioned structure and revise only contradicted walls/objects; assign new occluders to spatially stable chamber IDs, not label integers that change across snapshots. Trigger Plan2 from actual new-obstacle evidence, recompute latest predictions while moving, then validate against latest LiDAR before promotion. Remove the sticky local WFD fallback that can prevent R7 from resuming on later updated structure. Do not hardcode the user's D02 goals/corners or add New Room-specific thresholds.

## Reproducibility / artifacts

- Run: `/home/dell/DELL_NATIVE2D/runs/NR-G2R7-03`
- `RUN_MANIFEST.json`: immutable per-file source SHA-256/lineage, DELL CPU threads 2.
- `RUN_RECEIPT.json`: SOURCE_TERMINAL, rc0, 444.556s wall, peak worker PSS 2172.05 MiB; native mission wall 427.579s.
- `worker.stdout.log` and `g2_r7/decisions/arrived_tick_*.json`: native selection and actual goal-arrival proofs.
- `R7_NATIVE2D_TERMINAL_AUDIT.json`: exact odometry, same coverage, frozen D02 ROI, G1/LaMa parity, actual arrivals.
- `R7_NATIVE2D_REAL_PATH_SCROLL.html`: self-contained horizontal scroll/gallery of **real recorded path**, with hypothetical desired G3/G4 explicitly outlined and labeled as *not executed*.
- `r7_visuals/`: generated independent PNG checkpoints.
- Prior `NR-G2R7-02` FAILED at first native R7 choose due Python3.6 NumPy int64 JSON serialization; preserved as failed evidence. The fixed `NR-G2R7-03` is the terminal trial.

**Scientific interpretation:** R7 successfully preserved and executed the initial portal + rear-left corner Plan1 in genuine closed-loop. Its versioned structural prediction / Plan2 handoff remains unsolved, and whole-map distance is worse than Run04. No claim of generalization to Hospital or novelty.
