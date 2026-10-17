# Run 11 native 2D — R4 structural-first predicted-viewpoint arbitration

**Execution:** COM1, 2026-10-17. **Status:** Source terminal PASS, one deterministic New Room run. **No independent IR1 QA or multi-seed scientific conclusion.** Run10-R2 and all old baselines were not modified. No Gazebo.

## What changed

R11 protects the original G2-R4 known-FREE goal with a pathwise structural and **executed-endpoint-only** viewpoint-quality comparison. LaMa predicted FREE inside UNKNOWN may propose up to eight candidates per decision. The actual safe subgoal must maintain R4 region obligations, G2-R3 structural bins, path objective, and arrival camera visibility; additional hypothetical target value is discounted. R4 wins ties/failures. A structural-region witness is tried before the Run10-R2 G2 geometry-only reposition on R4 DEFER. RoomNode original WFD is an **exit-only** condition, not a LOCAL goal source.

R11 GitHub source branch: `mx072-g2-r4-structural-predicted-r11-20261017`. COM1 source root `/work/com1/mx072_g2_r4_structural_predicted_run11_20261017`, DELL synchronized root `/home/dell/mx072_g2_r4_structural_predicted_run11_20261017`.

## Preflight and real native run

- Source pinned and staged separately from frozen Run10-R2.
- Deterministic structural contract **9/9 PASS** on COM1 Python 3.13.
- Same-snapshot off-policy replay on Run10-R2: **24 LOCAL decisions**, **zero** speculative replacements accepted, zero WFD-as-movement-goal violations, 3 geometry-reposition-only decisions. Replay does not establish causal closed-loop performance.
- Guarded native MapEx 2D: **SOURCE_TERMINAL**, return code 0, **1,017 recorded steps**, **223.99s worker elapsed**.
- **26 primary decisions:** **16 LOCAL G2 goals**, 10 GLOBAL MapEx goals. **16/16 LOCAL goals** were same-snapshot observed FREE and in live native walkable RoomNode mask, and **16/16** exact goal coordinates eventually appeared in the saved odometry. **6/6 LOCAL→GLOBAL** transitions had `LOCAL_WFD_EXHAUSTED_RETURN_GLOBAL` with exactly **zero** reachable LOCAL frontier candidates. G2 produced **2** backstop LOCAL viewpoints.
- R11 actually compared **35 predicted candidates** across the new closed-loop trace, but **0 predicted targets passed arbitration**. No unsafe UNKNOWN goal was executed. This is an important scientific **negative outcome** for the experiment: structural safeguards are apparently so conservative that they disable using hypothetical UNKNOWN viewpoints on this run. Do not conflate this behavior with validated predictive planning.

## Full-run comparison, native New Room

| Metric | Run04 RoomNode/MapEx | Run08 R4 knownfree exact | Run09 R4 predicted | Run10-R2 frontier-exit G2 predicted | **Run11 R4 structural arbitration** |
|---|---:|---:|---:|---:|---:|
| Full path distance (m) | **199.108** | 310.183 | 282.224 | 277.293 | **285.471** |
| Native recorded steps | 709 | 1099 | 1006 | 988 | **1017** |
| Native worker wall (s) | **95.905** | 222.820 | 265.030 | 289.170 | **223.990** |
| Final map core UNKNOWN pixels | 2085 | 2086 | 2083 | 2083 | **2076** |
| D02 original room <=270 unknown milestone, distance from t12 (m) | **24.861** | 25.079 | 208.173 | 30.508 | **25.079** |

The D02 metric uses identical frozen original room mask, 6,564 cells, initial 3,889 unknown cells at native tick12. Run11 reaches the same <=270 threshold at native tick105, after 25.079 m from D02. Whole-run UNKNOWN difference of 7 to 10 cells relative to Run08/10-R2 should **not** be overinterpreted as a gain at equal final coverage or safety.

Relative to Run10-R2: **8.178m longer full-run path (2.95%)**, but **65.18s shorter worker wall (22.54%)**. Native wall is computational/simulator wall, **not physical travel time**. Relative to Run04, R11 still drives 86.363m farther on this map. At D02, R11 matches Run08 because G2 keeps the observed-FREE R4 viewpoint rather than the poor subgoal.

## Interpretation

The user's original quality criticism was well-founded: Run09/10 could prioritize a predicted endpoint while overlooking the actual inferior executable waypoint. R11 successfully prevents replacing good FREE R4 viewpoints by such weak subgoals on the evaluated trajectory. But it now **accepts zero UNKNOWN proposals**, so R11 is not a successful demonstration of combining prediction with structurally better G2 goal choice. A separate ablation is needed to relax structural gates thoughtfully, without reverting to endpoint-unaware heuristics or using GT for online tuning.

Do not overwrite this run to perform that ablation. No claims of generic algorithm superiority, online latency improvement or physical safety.

## Artifacts

- `evidence/guard_run11_structural_predicted/terminal_receipt.json` — signed worker terminal receipt.
- `evidence/R11_OFF_POLICY_SNAPSHOT_REPLAY.json` — preflight frozen Run10-R2 selector replay, off-policy only.
- `evidence/RUN11_STRUCTURAL_PREDICTED_COMPARISON.json` — native-source computed D02/full-run measures, frozen 04/05/07/08/09 comparison.
- `reports/RUN11_NATIVE_POLICY_AUDIT.json` — detailed 26-decision same-tick safety/exits and 35 rejected predicted candidate count.
- `reports/RUN11_G2_R4_STRUCTURAL_PREDICTED_FULL_SCROLL.html` — 26-decision self-contained observed map + RoomNode + goal + trajectory gallery.
- `source/g2_structural_arbitration.py`, `source/g2_r4_policy_sidecar.py`, `source/g2_predicted_goals.py`, `source/test_r11_structural_arbitration.py` — actual pinned COM1 run source.
