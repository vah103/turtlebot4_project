# G2-R6.2 R2 — Native2D DELL experiment, 2026-10-18

**Verdict: Native run completed under MapEx planner terminal condition; overall distance objective FAILED. No method ACCEPT or environment-generalization claim.**

## 1. Inputs and execution controls

- Experimental run: `NR-G2R62-03`, source `/home/dell/DELL_NATIVE2D/build/g2_r62_general_structural_r2_20261018`.
- Baseline: `NR-RN-04`, full DELL/COM1 parity verified in `/home/dell/DELL_NATIVE2D/runs/NR-RN-04/DELL_VS_COM1_FULL_PARITY.json`.
- New policy pre-choice D01/D02 numerical parity: `D01_D02_NUMERIC_PARITY_PASS` for observed LiDAR, 3-model LaMa mean/variance, Room Node mode and pose.
- Native 2D New Room P1 (203 × 263, 0.10 m/cell), 3 LaMa models, Room Node V4-R4.1, known-FREE/inflated pyastar2d A* only. No GT/future map in policy, no Gazebo.
- Runtime: DELL Python 3.6 native simulator with Python 3.x asynchronous G2 sidecar; 2 CPU model threads. Model smoke PASS; Python 3.6 actual native A* to D02 and D11 proposals PASS.
- Source and tests available in [PR #47](https://github.com/vah103/turtlebot4_project/pull/47). 13/13 G2-R6.2 targeted regression tests PASS including zero-real-observation arrival, native WFD residual fallback, no unnecessary heavy call for single room-border FREE cell, and immediate unsafe path hold.

## 2. Why R2 was needed

- Original R6.1 `NR-G2R61-03` stopped at native step 44 with 3 reachable LOCAL WFD frontiers and a rigid positive utility/metre acceptance floor (`12`) that rejected every safe viewpoint.
- R6.2 R1 `NR-G2R62-02` removed that acceptance floor and used geometry-driven structural triggers. It then oscillated between adjacent goals (195,197) ↔ (195,198). **This first R6.2 native run was manually stopped and preserved**, not silently overwritten.
- R6.2 R2 additionally discounts predicted sight from the current pose, detects **zero actual newly observed ROOM cells upon reaching a viewpoint**, and switches to the accepted **native LOCAL WFD frontier** to avoid speculative neighboring viewpoints. This is based on actual LiDAR observation, not an absolute New Room-specific gain/m threshold. Original known-FREE A* and local frontier-exit condition remain intact.
- R2 local WFD fallback triggered once in the full run. No persistent adjacent-cell oscillation was observed thereafter in the checked logs. Still, the method retains design limits (20 viewpoint candidates, 5 in initial route, ray approximation and sensor/topological evidence settings). It is not literally threshold-free.

## 3. Full-run observed-map comparison

All distance figures are reconstructed from **entire** `odom.npy` positions with native P1 resolution 0.1 m/cell. Observed coverage is computed solely from `global_obs/<native_tick>.png`, no GT.

| Metric | Baseline NR-RN-04 | G2-R6.2 NR-G2R62-03 |
| --- | ---: | ---: |
| Native terminal outcome | SOURCE_TERMINAL | SOURCE_TERMINAL |
| Planner condition | No selectable frontier | No selectable frontier |
| Total travelled distance | **199.108 m** | **236.903 m** |
| Total native steps | 709 | 846 |
| DELL guarded elapsed wall-time | 370.850 s | 351.463 s |
| Peak measured PSS | 2119.7 MiB | 2246.1 MiB |
| End observed P1 cells / 53,389 | 51,304 | 51,308 |
| End observed P1 fraction | 96.095% | 96.102% |
| Total LaMa + Room Node heavy decisions | 25 | 19 |

**Distance penalty: +37.796 m (+18.98%)** at nearly the same final observed map coverage. A single paired run with a ~19.4 s wall-time benefit does not prove stable speed improvement, particularly since it travelled farther and had different decisions.

Fixed-comparison observed-coverage milestones (total travelled metres from start):
- 90% P1 observed: baseline **152.870 m**, G2-R6.2 **193.853 m**.
- 95% P1 observed: baseline **192.290 m**, G2-R6.2 **230.762 m**.
- 96% P1 observed: baseline **196.408 m**, G2-R6.2 **234.580 m**.
- Baseline's final observed count 51,304 cells: baseline **198.208 m**, G2-R6.2 **236.603 m**.

End-core occupancy mismatch is only 33 cells between runs, with 4 additional observed cells for R6.2, so the distance disadvantage is not plausibly explained by a much higher achieved map observation area.

## 4. First room D02 (frozen causal room ROI)

Initial D02 accepted Room Node mask frozen at tick 12: **6,564 cells, 3,889 UNKNOWN**. Actual LiDAR maps were compared in this SAME mask.

- First at-or-below 270 UNKNOWN native tick: baseline **tick 102**, **24.861 m after D02**, remaining 269.
- G2-R6.2 R2: **tick 98**, **22.832 m after D02**, remaining 268.
- First-room local improvement: **2.029 m (~8.16%)** at matched observed-coverage cutoff; this gain does **not** carry through the full map.

Snapshot-only comparison at decision times gives different distances (23.432m vs 25.685m) because the actual threshold is crossed between decision boundaries; the above is the more precise per-native-step comparison.

## 5. Planning behavior and interpretation

Observed sidecar event counts R2: S1 KEEP **102**, S2 LIGHT REPLAN **48**, S3 STRUCTURAL REBUILD **3**; no logged S0 events. The sidecar triggered zero-observation checkpoint twice and one LOCAL WFD fallback in the worker log. Heavy LaMa/RoomNode decision count (19) is below baseline (25), but local route quality remains weaker across later rooms.

**The native mission-complete flag only means MapEx stopped finding selectable frontiers. It does NOT certify ground-truth complete exploration.**

Design failures still requiring independent review:
1. Greedy route/goal selection degrades in later room geometries; lower planning overhead alone cannot save travel.
2. Pathwise predicted visibility can over-credit unobserved cells; fallback limits loops but does not fix prediction quality.
3. Residual WFD fallback may be conservative in rooms where Watchman still has useful alternatives; instrument its causal impact.
4. Room topology and current-room ROI recognition need Hospital and varied multi-door/corridor tests under unchanged policy config.
5. Run one or more additional seeds / repeat native trials before drawing robust time or efficiency conclusions.

## 6. Reproduction and immutable files

- DELL run root: `/home/dell/DELL_NATIVE2D/runs/NR-G2R62-03`.
- Guard outcome: `RUN_RECEIPT.json`; parity gate: `DELL_PARITY_AUDIT.json`; source hash: `RUN_MANIFEST.json`; worker trace: `worker.stdout.log`.
- Exact full P1 per-tick coverage and D02 milestones: `R62_FULL_P1_OBSERVED_AUDIT.json`.
- Original G2-R6.1 failure: `/home/dell/DELL_NATIVE2D/runs/NR-G2R61-03`; first R6.2 oscillation STOP: `/home/dell/DELL_NATIVE2D/runs/NR-G2R62-02`.
- None of these prior run directories or `NR-RN-04` was overwritten.

**Scientific verdict: G2-R6.2 R2 is operationally feasible on DELL Native2D, but its main goal (shorter distance to the same whole-map observed coverage) FAILED on New Room. Scientific effectiveness remains NOT_USEFUL / unproven; hold any paper-quality gain claim until a better planner is tested on diverse maps.**