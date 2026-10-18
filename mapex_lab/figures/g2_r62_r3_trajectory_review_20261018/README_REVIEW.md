# G2-R6.2 R2 vs R3: New Room Native2D comparison

R2 source: DELL NR-G2R62-03; R3 source: DELL NR-G2R62R3-02; date 2026-10-18.

Both reach exactly 51308/53389 P1 LiDAR-observed cells (96.1022%).

| Metric | R2 | R3 |
| --- | ---: | ---: |
| Path | 236.9033 m | 279.2113 m |
| LOCAL path | 104.2495 m | 120.3395 m |
| GLOBAL path | 132.6538 m | 158.8718 m |
| Native ticks | 846 | 992 |
| RoomNode decisions | 19 | 20 |

R3 is not a better replacement for R2, with +42.308m (+17.9%) total travel.

## Evidence-based observations

- Only two R3 native CHOOSE events select FRONTIER_RESIDUAL: D03 and D05 in the first room. First-room LOCAL distance R2=23.432m; R3=24.961m, equal frozen-room final UNKNOWN of about 266-267 cells.
- R2 leaves room2 at D10/t331 with 36 UNKNOWN inside the same frozen 8088-cell R2 room2 ROI; R3 leaves at D10/t264 with 628 UNKNOWN inside that ROI. R3 had 56 geometric frontier cells but zero officially eligible reachable Room WFD representatives. This does not by itself prove a bug in G1/WFD; must check reachability and room mask semantics.
- R3 D20/t919 predicts room_id=7, but its mask has IoU 0.953 with R2's room2 mask: this is the same physical room. After D19/t781, R3 goes 39.57m GLOBAL to D20 and then 20.53m LOCAL until terminal. This block is linked to the late return; not every meter is automatically avoidable.
- R3 room5 LOCAL 19.479m versus R2 3.494m. However R2 then resolves the room5 ROI from 623 UNKNOWN to 54 UNKNOWN during 5.690m GLOBAL travel: the extra R3 LOCAL scan is not necessarily the most efficient when considering the onward route.
- At first reaching 90% global P1 LiDAR coverage, R3 takes 166.058m versus R2 193.853m, a 27.795m early advantage. At 95%, R3 takes 254.186m versus R2 230.762m, a 23.424m disadvantage.

## Files

- G2_R62_R2_VS_R3_SIDE_BY_SIDE.html: 16 side-by-side landmark pairs, each with observed map/path, LaMa mean and RoomNode.
- NR_G2R62R3_02_REAL_PATH_SCROLL.html: complete causal actual-path gallery for R3.
- R3_VS_R2_LOCAL_ROI_SEGMENTS.json: frozen room-mask comparisons and per-room distances.
- R3_R2_MATCHED_GLOBAL_COVERAGE.json: precise first-tick coverage milestones.
- R3_NATIVE_DISTANCE_AND_OBS_AUDIT.json: native run receipts and causal planner summaries.
- R3_OFFLINE_TEST_RESULT.json: unit/offline prototype tests.

Policy inputs were causal. No ground truth was used for robot actions. Frozen masks are only for retrospective evidence.
