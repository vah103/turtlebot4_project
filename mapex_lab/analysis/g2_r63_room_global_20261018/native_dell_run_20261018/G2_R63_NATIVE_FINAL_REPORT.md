# G2-R6.3 R1 — Room-to-Global Watchman, native DELL 2D result
**Date:** 2026-10-18. **Verdict: implementation and native execution completed; distance objective FAILED.**
**Code / QA:** branch `mapex-g2-r63-room-to-global-r1-20261018`, [draft PR #48](https://github.com/vah103/turtlebot4_project/pull/48).

## 1. What was tested
G2-R6.2 retained its original causal robot LiDAR map, Room Node V4-R4.1, LaMa 3-model mean/variance, observed-FREE-only viewpoint candidates, native A* validation, original LOCAL-WFD exhaustion condition and R6.2 anti-loop residual frontier fallback. G2-R6.3 **does not add or command new GLOBAL goals**. It only reorders the same up to 5 native-safe room viewpoints, taking into account (a) known-FREE path length inside the room, (b) LaMa model-predicted pathwise observation gain, and (c) a lower-bound distance from a potential final viewpoint to an **observed external frontier** computed by 4-connected known-FREE BFS. External frontier distances are **not** validated by native inflated A* and are never motion commands. No GT or later scans were used in its policy.

## 2. Critical R6.2 distance decomposition before implementation
Real `odom.npy` and Room Node mode timeline:

| Mode | NR-RN-04 | G2-R6.2 R2 | Difference |
|---|---:|---:|---:|
| LOCAL | 104.179 m | 103.526 m | -0.653 m |
| GLOBAL | 94.929 m | 133.377 m | +38.449 m |
| TOTAL | 199.108 m | 236.903 m | +37.796 m |

This supports investigating room-to-global coupling but does **not establish a causal attribution** of GLOBAL excess to the room's last viewpoint. Global frontier order, entry locations and Room Node semantics all change between runs.

## 3. Causal offline frozen decision smoke
Replayed exactly D02, D08, D11, D14 and D16 from the stored **R6.2** causal observations, mean/variance, ROOM semantics and native reachable-FREE masks (before seeing later outcome). Same original G2-R6.2 goal **set**, only order may change. Nine focused tests PASS, including no GT/UNKNOWN movement and no external frontier fallback.

| Decision | Baseline model room+exit estimate | R6.3 model room+exit estimate | Note |
|---|---:|---:|---|
| D02 | 24.9 m | 24.3 m | Same five goal set; reorder |
| D08 | 38.2 m | 38.2 m | Effect negligible |
| D11 | 36.1 m | 33.9 m | Predicted 2.2 m better |
| D14 | 36.2 m | 36.2 m | Keep baseline |
| D16 | 38.2 m | 38.2 m | Effect negligible |

These are **model estimates only**. The 4-connected outside-frontier distance is a possibly optimistic estimate, not known to be a useful MapEx GLOBAL frontier, nor a validated safety path or end-to-end benefit.

## 4. Full DELL native 2D run NR-G2R63-02
- Launch via the same `DELL_NATIVE2D` pipeline and resource guard, separate run id; no old directory overwritten.
- LaMa model-smoke PASS; Python3.6 syntax/source preflight PASS; D01/D02 source-epoch obs, mean/variance and Room Node mode/robot numerical parity against `NR-RN-04` PASS.
- Final receipt: `SOURCE_TERMINAL`, return code 0, **913 native ticks**, **385.578 s**, `MAPEX_NO_SELECTABLE_FRONTIER_NOT_GT`. Mission-complete is a planner termination, NOT GT coverage certification.
- Final P1 observed = **51,304 / 53,389 cells (96.095%)**, matching baseline exact observed count (final occupancy differs in individual cells; see machine audit). No GT to compute coverage.
- Full native travelled distances from odometry: baseline `NR-RN-04` **199.108 m**; R6.2 **236.903 m**; R6.3 **255.317 m**. R6.3 worse by **+56.209 m (+28.2%)** relative to baseline, and **+18.414 m (+7.8%)** relative to R6.2.
- Phase totals: baseline LOCAL 104.179, GLOBAL 94.929 m; R6.2 LOCAL 103.526, GLOBAL 133.377 m; R6.3 LOCAL **110.633**, GLOBAL **144.684 m**.
- P1 same-coverage thresholds: 90% baseline 152.870 m, R6.3 **215.427 m**; 95% baseline 192.290 m, R6.3 **221.545 m**; 96% baseline 196.408 m, R6.3 **252.170 m**.
- Frozen initial D02 ROOM at <=270 observed UNKNOWN: baseline 24.861 m from D02, R6.2 22.832 m, R6.3 **37.968 m**. R6.3 only crossed this fixed ROI threshold at native tick 151, after exit from initial LOCAL room mode at tick 138. This highlights delayed observation of the original room.
- New R6.3 logs: 22 Room Node/LaMa heavy decisions, structural events S1_KEEP 98, S2_LIGHT_REPLAN 34, S3_STRUCTURAL_REBUILD 4.
- R6.3 detects seven LOCAL room sessions, but numerical room ids/detection times are not spatially equivalent to the six apparent R6.2 sessions. Do not equate room ID across runs.

## 5. Conclusion and research consequences
This experiment **refutes the simple R6.3 hypothesis** that assigning room-viewpoint order by optimistic distance to the nearest *currently observed external frontier* is sufficient to reduce global travel. It also degraded first-room fixed-coverage distance despite looking slightly better in the offline projected objective. Do **not** increase a generic exit cost weight or hand-tune thresholds to New Room to rescue this run.

Likely unresolved problems (hypotheses, not established causes): the nearest exterior WFD point may not be the frontier MapEx will choose next; room endpoint estimates ignore the actual global priority/utility and may redirect robot along long routes; pathwise LaMa visibility may be wrong and planning is continually revised with new LiDAR; altered local trajectories change subsequent global frontier discovery and room recognition.

A stronger next stage must **measure** whether choosing the exiting viewpoint actually reduces cost to a feasible **GLOBAL MapEx-scored/selected** candidate *using the same current observed map*, rather than a nearest geometrical exterior frontier. That may require explicit, read-only GLOBAL frontier ranking supplied alongside current ROOM state, with motion still on native safe FREE paths and LOCAL termination still by room WFD. First audit this causally in multiple rooms and Hospital, not only New Room.

## 6. Cross-environment limitation
Hospital archived hpx_003 on DELL contains 250x310 causal observed/LaMa/Room semantics, but current `DELL_NATIVE2D` contains **no validated Hospital native replay profile or same-epoch robot pose + native inflated A* scope comparable to NR-RN-04**. Therefore **no honest Hospital native R6.3 run** can be claimed from these arrays alone. Not transferring a New Room threshold is necessary but does not prove transfer performance.

## 7. Artifacts and reproducibility
- `/home/dell/DELL_NATIVE2D/runs/NR-G2R63-02`: `RUN_RECEIPT.json`, `DELL_PARITY_AUDIT.json`, `RUN_MANIFEST.json`, `worker.stdout.log`, `R63_FULL_P1_OBSERVED_AUDIT.json`, and all `global_obs/*.png` + `odom.npy`.
- `/home/dell/DELL_NATIVE2D/analysis/g2_r63_room_global_20261018`: `audit_distance.py`, `R63_SEGMENTED_DISTANCE_AUDIT.json`, `eval_frozen_decisions.py`, `R63_FROZEN_FIVE_DECISIONS.json`, `test_r63_exit.py`, `r63_exit_planner.py`.
- `/home/dell/DELL_NATIVE2D/build/g2_r63_room_global_r1_20261018`: exact R6.3 native worker/sidecar source; no original source replaced.

**Outcome: NO_GAIN / NOT_USEFUL at this stage.** Local-to-global coupling remains a plausible research question, but the proposed exit-distance surrogate is empirically insufficient.