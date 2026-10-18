# G2-R7-R3.1 — Stable room hypotheses, New Room native 2D

**2026-10-18 | DELL | NR-G2R7-05 SOURCE_TERMINAL (not independent IR1/QA acceptance).**

## Main design change

Unlike R7-R3, which repartitions the entire predicted room at every G1/LaMa epoch, R7-R3.1 **freezes the initial accepted G1 two-chamber partition as a structural hypothesis**. When actual LiDAR reveals unexpected occupied cells previously predicted FREE, it records contradictions *inside the original chamber IDs*; the new observations may invalidate specific occupied/free cells and safe paths, but they cannot silently swap the chamber ID or move the original entrance/portal.

- Geometry frozen per native G1 LOCAL `room_id`; reset only for a genuinely new native room identity.
- Prediction is fallible. Actual occupancy and native inflated known-FREE A* remain authoritative for movement. No GT or future LiDAR enters G2.
- The robot still selects initial Plan1: overview from chamber portal, then viewpoint **deep behind the left-room object**, not a near-divider shortcut.
- **Plan2 is committed only after native ACK of Goal1**, NOT after the first few obstacle pixels seen while approaching it (an incorrect early commitment was found and isolated as aborted trial NR-G2R7-04).
- On verified Goal1 and current LiDAR, cluster new actual occupied cells that contradict the *initial* predicted empty chamber. Assign them spatially using the frozen chamber masks.
- Select geometrically deep far-wall corners on opposing transverse sides of these objects; order using still-UNKNOWN observed cells in corresponding chamber halves.
- Keep the Plan2 order through Goal2; native A* rechecks every promoted goal, and the original G1 LOCAL WFD exit gate remains authoritative for leftover frontiers.
- No D02 coordinates, forced 4-goal count, or New Room-specific direction labels are hardcoded into this algorithm.

## Exact demonstrated D02 behavior

| Goal | P1 row,col | Actual native arrival tick |
|---|---|---:|
| 1: portal between chambers | (169,132) | 27 |
| 2: rear lower-left corner | (196,96) | 55 |
| 3: deep lower-right | (196,206) | 110 |
| 4: deep upper-right | (148,206) | 130 |

At D02 tick12, the planner had Plan1 [G1,G2]. The first causal structural commitment to Plan2 occurred at **tick27 after GOAL1 ACK**, from 43 actual newly observed occupied cells inconsistent with the initial predicted FREE right chamber. It selected **G3 lower-right before G4 upper-right** because 562 UNKNOWN cells remained in lower vs 276 in upper half on the current initial-zone partition. The exact native `arrived_tick_*.json` receipts at ticks 27,55,110,130 certify the robot actually arrived at all four places. All goals were subject to native inflated observed-FREE A*.

## Measured terminal numbers

| Metric | NR-RN-04 original MapEx+G1 | NR-G2R7-03 prior R7-R3 | NR-G2R7-05 R7-R3.1 |
|---|---:|---:|---:|
| Native 2D terminal | SOURCE_TERMINAL | SOURCE_TERMINAL | SOURCE_TERMINAL |
| Real odometry distance, metres | 199.108 | 264.968 | **287.771** |
| Odom positions (native steps) | 709 | 954 | **1021** |
| Actually observed P1 cells | 51,304/53,389 | 51,308/53,389 | **51,308/53,389** |
| Percent observed | 96.095% | 96.102% | **96.102%** |

The stable-structure R7-R3.1 is **88.663 m (+44.53%) longer than RN04** for effectively identical observed coverage, and **22.803 m longer than prior R7-R3**. At a matched threshold of 51,304 observed P1 cells, RN04 first reached it after **197.908 m** while R7-R3.1 first reached it after **287.324 m**.

Frozen D02 first-visit mask (exact frozen R7 current D02 room ROI, threshold <=270 remaining UNKNOWN): RN04 after **24.637 m from native tick12**, R7-R3.1 after **25.091 m** (+0.454 m). This is NOT evidence of less travel to same local coverage. Note the threshold is crossed before the four-goal strategy has finished. After all goals, native G1 declares no remaining LOCAL WFD and returns GLOBAL.

Approximate traveled distance by scope classification (using postscan latest scope, therefore not exact partition at transition ticks):
- RN04: LOCAL ~104.48m / GLOBAL ~94.71m.
- R7-R3.1: LOCAL ~129.66m / GLOBAL ~158.19m.

This suggests both additional local loops and large later GLOBAL overhead. It does not prove all extra global travel was caused by D02. Need matched spatial decision/room analysis before causal attribution.

The initial D02 snapshot is **bitwise equal** to RN04 for observed map, LaMa mean, LaMa variance and G1 semantic (all mismatch counts = 0), so the first divergence is caused by the different G2 choices and not by a different initial LaMa map. This is one run per policy and does not establish generalization.

## Execution and assets

- Canonical: `/home/dell/DELL_NATIVE2D/runs/NR-G2R7-05`
- `RUN_MANIFEST.json`: immutable source hashes and configuration lineage; 3 LaMa small crop, original RoomNode V4-R4.1 and native MapEx grid.
- `RUN_RECEIPT.json`: SOURCE_TERMINAL, returncode0, ~453.4s guard wall, peak worker PSS 2169.91 MiB; mission elapsed ~437.39s.
- `g2_r7/decisions/decision_*.json` and `arrived_tick_*.json`: planned and *actually reached* goals.
- `R7_R31_NATIVE2D_TERMINAL_AUDIT.json`: terminal odom, original frozen D02 ROI, parity, matched coverage, events.
- `R7_NATIVE2D_REAL_PATH_SCROLL.html`: iPad-friendly embedded-image scroll gallery of the **executed** robot trajectory from T12 through terminal; G1–G4 filled green only after actual native arrival.
- `r7_visuals/`: PNG snapshots if the HTML is offline.
- Unit tests **10/10 PASS**, including horizontal reflection (no D02 hardcoding), time-causal first-goal ACK commitment, both-corner order, stable chamber ID despite changing semantic, mandatory native validation, no forced four-goal room.
- `NR-G2R7-04`: ABORTED after discovering premature Plan2 commitment on 4 anomalous occupied cells at tick14. Evidence kept, not overwritten.

## Interpretation and next steps

**Success:** architecture now follows intended room-structure reasoning through Plan1 then Plan2 and actually executes all four viewpoint goals in desired order. No hindsight or coordinate injection.

**Not yet solved:** total path efficiency. R7-R3.1 is worse than original MapEx+G1 in New Room despite near equal final observed P1 cells. It needs a mechanism for evaluating value of planned viewpoints **relative to observations along the route and the subsequent MapEx GLOBAL decision**, plus testing on Hospital/other spatially distinct rooms, without tuning magic thresholds solely for D02.

No Gazebo, no physical robot, no altered old runs. Do not call this a scientific gain, and do not attribute extra travel solely to local goals without additional causal work.
