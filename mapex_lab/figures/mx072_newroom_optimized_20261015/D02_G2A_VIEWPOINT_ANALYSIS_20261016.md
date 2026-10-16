# G2-A Run 04 / Decision 02: causal viewpoint proposal (2026-10-16)

## Inputs and cutoff
- Read-only Run 04 original data from `/home/dell/mx072_native2d_pair_20261015/OPTIMIZED_FULL_PAIR_20261015/COM1_VERIFIED/native_opt_r41_lama3small_run04/R41_OPT_LAMA3_SMALLROI_RUN04_visvarprob` and section D02, panel **3 · Room Node: cấu trúc dự đoán** of the original comparison HTML on branch `mx072-newroom-real-25-decisions-ipad-20261015`, pinned source commit `e8b1c8a2b2672e5e39d4fd56b931c9f97c58de98`.
- Current sources: `room_r41/decisions/decision_000002.json`, `room_r41/inference/decision_000002_input.npz` (observed, LaMa mean, variance), `decision_000002_semantic.npz`, D02 Room Node report, D01 decision context, and native code for Room ROI segmentation. **No GT, D03+ map, scan, mission outcome or later room structure was used** to choose viewpoints.
- D02: tick 12, robot padded (647,611) -> (x=11.1m,y=14.7m), Room ID 1, local mode, 5 local vs 20 global frontier candidates, distance before decision 3.3708m. Core native image 203x263 at 0.1m/pixel, offset (row=500,col=500).
- Current Room 1 mask is the 4-connected component of `semantic==1 & observed!=2` containing robot, as in `room_native_integration.py`, **not** all pink regions and **not** the one extra R4.1 tentative hypothesis of 2,376 unknown cells located elsewhere. Connected mask bbox core rows 144–200 cols 92–210, 6,564 cells: 2,675 already observed free, 3,889 unknown (3,876 unknown with LaMa mean <0.5).

## Selected minimum sequence
- Robot: native (647,611), core (147,111), (11.1,14.7)m.
- **VP1 primary proposal:** native (668,635), core (168,135), (13.5,16.8)m. The observed-free, connected-grid route from robot is **4.5m** (4-connected shortest route, room-bounded, with 2-cell square exclusion around non-free cells). Clearance proxy nearest non-free ~0.36m (Euclidean).
- **VP2 optional only after a real VP1 LiDAR scan:** native (671,632), core (171,132), (13.2,17.1)m, ~0.6m of additional knownfree 4-connected path from VP1 with same two-cell inflation. This is **not a pre-committed fixed route**; recompute and cancel/replace based on actual scan.
- Uncertainty: no guarantee VP1 by itself actually completes this room, and VP2 is not chosen using an outcome of an offline GT scan.

## Causal offline G2-A heuristic
1. Candidate sampling every 3 core-grid cells, subject to current observed free, membership in robot-connected Room1, nearest non-free clearance >=2 core cells, reachable from robot on known-free 4-connected grid: **254 candidates**.
2. Nominal visibility: **720 rays**, **20m maximum**, **0.5 grid-cell step** per candidate; block at currently observed occupied, LaMa mean occupancy >=0.5 on unknown, or Room1 boundary. Only credit current unknown cells inside Room1 whose mean occupancy <0.5. Ranking: maximize count, break ties by shortest known-free route (not a fully time-optimal planner).
3. VP1 ranked top at 3,843 cells in the 720-ray search; same chosen VP1 refined using **2,500 rays with 0.25-cell sampling** yields **3,840/3,876** predicted-free unknown cells (~99.07%). Current-epoch VP2 contributes **27 additional cells**, nominal union **3,867/3,876**, only **9 predicted-free unknown** remain under the heuristic model. The other 13 room unknown cells have LaMa mean >=0.5 and are not included as predicted-free information; physical wall returns would need a different accounting.
4. All calculations use D02 LaMa mean for binary nominal occlusion and candidate ranking; variance was read but **not weighted**. No actual native probabilistic visibility score or true sensor model used. This is not an optimality proof.
5. Green path on the visualization is our provisional grid route over D02 known free; orange path VP1→VP2 is **conditional**. Neither is the native MapEx A* logged route, robot motion, or continuous-footprint safety certificate.

## Next actual test (separate approval)
Keep both viewpoints frozen before any evaluation; replay GT LiDAR offline if requested, then quantify real newly revealed unknown inside Room1 and lost/new gain on the route. Do not tune a viewpoint after inspecting D03+ or GT and misrepresent the tuned proposal as prior causal selection.

Artifact: `D02_G2A_VIEWPOINTS_ON_ORIGINAL_PANEL3_20261016.png`.
