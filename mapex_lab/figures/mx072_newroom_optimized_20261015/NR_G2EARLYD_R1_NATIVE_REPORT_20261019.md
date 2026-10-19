# G2-EARLYDOOR-R1 — Dynamic next GLOBAL through observed exits
Native New Room 2D · 19/10/2026 · NR-G2EARLYD-02

## Experiment performed
Original MapEx + G1 V4-R4.1 and G2-SWEEP-R1 LOCAL preserved in full. During each LOCAL epoch the robot also scores all original GLOBAL MapEx WFD candidates using the same-epoch observed/LaMa model, recovers original raw IG, detects observed-FREE passable openings on G1 ROOM boundaries, ranks candidate GLOBAL WFD from those exit points using IG/Euclidean(exit,global-frontier), and stores a provisional (GLOBAL frontier, exit) pair. Every LiDAR/LOCAL epoch refreshes the pair, including when a new opening becomes observed. On the first LOCAL->GLOBAL transition, the last saved goal is used only if still a current official WFD within 1m and native-A*-reachable. Otherwise original MapEx GLOBAL scorer chooses. Other GLOBAL decisions are not modified. This is a **new portal-anchored MapEx scoring variant**, not proof that original MapEx was wrong. No GT, handcrafted D02 goal or future map in robot inputs.

Four synthetic tests PASS, including simulated newly observed door causing next GLOBAL goal update and disappearing WFD causing safe fallback. 3-model LaMa smoke PASS. D01/D02 native numeric and mode parity PASS. Frozen staged code has SHA manifest. Receipt SOURCE_TERMINAL / returncode 0.

## Real run outcomes

| Metric | Original Run04 | G2-SWEEP-R1 | G2-EARLYDOOR-R1 |
|---|---:|---:|---:|
| GLOBAL distance | 94.928869 m | 125.348219 m | **121.895432 m** |
| LOCAL distance | 104.178714 m | **88.159364 m** | 91.306578 m |
| Total | **199.107583 m** | 213.507583 m | **213.202010 m** |
| Native ticks | 709 | 757 | 757 |
| Decision count | 25 | 27 | 28 |
| Final observed P1 | 51,304 | 51,303 | 51,303 |

EarlyDoor vs SWEEP: GLOBAL **−3.452786 m**, LOCAL **+3.147214 m**, TOTAL **−0.305573 m**. Final observed core known masks match *exactly* SWEEP-R1 (same 51,303 pixels, zero missing/extra); both are missing exactly one core P1 cell compared with Run04. Therefore improves distance only marginally compared with SWEEP, remains **+14.094427 m** worse than original MapEx+G1 Run04.

## Actual dynamic update evidence

22 LOCAL decision epochs held/rescored next GLOBAL information. Across actual New Room, no new **observed passable G1 boundary-opening cluster** appeared mid-room (portals stayed fixed within each room). This matters: only the synthetic test demonstrates direct reaction to a truly newly discovered door; this run itself tests dynamic frontier/IG updates and stored goal consumption.

At room1 entry D02, candidate target core [88,80] via passable portal [143,111]. After LiDAR, changes to core [124,199], later [128,192]. Upon GLOBAL at D07 t84, final reserved frontier no longer matched current official WFD, so safe fallback kept original MapEx. Room2 entry D08 forecast global [155,54], later at D12 moved to [100,88], at D13 moved back to [155,54], and at exit D14 t245 it matched original MapEx choice. Room3 exits D18 and room5 D27 also matched current MapEx.

**Only one GLOBAL goal was changed:** D23 t551 after room4, original MapEx scored frontier at [18,64] highest (score magnitude 0.3883), while provisional exit-anchored policy chose [48,59] (original MapEx score magnitude 0.3663, rank slightly lower). The current WFD remained valid, native A* path to selected first goal was 16.2m. This intervention led to changed subsequent trajectory/LOCAL while final observed mask remained equal.

Of 5 LOCAL→GLOBAL transitions, 1 saved reservation vanished -> fallback; 4 were valid; just **1** changed the chosen GLOBAL WFD. The dynamic early-GLOBAL strategy produced only a small net benefit in this particular run, not evidence of a large or general improvement.

## What was and was not demonstrated

Confirmed: choosing next GLOBAL early and revising it every LOCAL epoch is implementable without leaking GT; stale frontier safety gate works; one valid alternate GLOBAL order was actually executed; final observed P1 parity with SWEEP is exact; modest −0.31m total savings. A separate scan-by-scan replay found 756/757 postscan maps bit-identical; tick 601 differs transiently, although terminal observed masks agree. The first odometry divergence is at tick 567 (six tick positions differ).

Not confirmed: any physically new door discovered mid-room on this particular New Room exploration; improvement over original MapEx+G1; robust effect on Hospital, randomized start, dynamic LiDAR; whether pre-selecting GLOBAL should also steer LOCAL sweep to end at the most convenient exit. LOCAL remains unchanged until future information changes global path, by deliberate one-factor experimental isolation.

Next logical separate hypothesis: use the dynamic early (GLOBAL goal, exit portal) to choose LOCAL sweep direction / exit-side, while preserving native G1 WFD and no GT. Register under new version and evaluate on New Room + Hospital; do not retroactively modify this frozen result.

## Files

- Frozen run /home/dell/DELL_NATIVE2D/runs/NR-G2EARLYD-02/
- Algorithm /home/dell/DELL_NATIVE2D/build/g2_early_global_door_r1.py
- Runner /home/dell/DELL_NATIVE2D/build/g2_early_global_door_r1_deploy.py
- Audit /home/dell/DELL_NATIVE2D/analysis/g2_autonomous_research_20261018/NR_G2EARLYD_NATIVE_COMPARE.json
- Full observed decision gallery /home/dell/DELL_NATIVE2D/analysis/g2_autonomous_research_20261018/NR_G2EARLYD_R1_FULL_DECISIONS_SCROLL.html
