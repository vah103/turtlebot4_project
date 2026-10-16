# Run04 D02 -> VP1: causally reconstructed observed map (2026-10-16)

This file is a **counterfactual G2 offline snapshot**, not a native Run04 decision D03, not a future-available LaMa forecast, not a new run or mission outcome.

Frozen at D02:
- Robot native (row,col)=(647,611), core (147,111).
- G2-A primary viewpoint already selected/committed in previous artifact: native (668,635), core (168,135).
- Room 1 connected component in D02 R4.1 semantic covers 6,564 cells, initially observed 2,675 free and 3,889 unknown.
- Snapshot taken after simulated **9 LiDAR observations**, one every 0.5m along 4.5m 4-connected knownfree route from robot to VP1, including endpoint. Range 20m, 2,500 rays, 0.25-cell ray step, 360-degree; **discrete approximation**, not exact native MapEx range_libc/flood-fill.
- Simulator uses GT cropped to current map dimensions ONLY to determine ray occlusion and classify returned free/occupied cells; no source cells outside ray coverage are revealed, GT does not determine viewpoint or path selection.
- Newly revealed in Room1: 2,868 cells = 2,822 free + 46 occupied. Unknown remains: 1,021 Room1 cells.
- Global newly observed cells: 3,841 = 3,676 free + 165 occupied, across all map areas.
- Among the **46 new wall pixels inside Room1**, **0** had LaMa mean >=0.5 at D02. Thus prior nominal binary wall prediction missed them; updated model **has NOT yet been rerun**, no statement about post-VP1 LaMa mean or Room Node output can be made.
- Frozen Room1 outline in this visualization is the **D02 semantic** and not the post-VP1 detector output.
- Reconstructed observed map saved on DELL, separate from original run at `/tmp/MX072_RUN04_D02_G2A_20261016/D02_VP1_RECONSTRUCTED_POSTSCAN_OBSERVED.npz`; source native Run04 files unchanged.

## To choose G2 viewpoints AFTER VP1 causally

1. Use `updated_observed` raster from reconstructed snapshot and robot pose (168,135) as only the new scan-derived map observation.
2. Perform new LaMa inference to recompute mean/variance from **this** updated observed map (do not reuse D02 mean).
3. Rerun G1 V4-R4.1 on fresh observed and fresh mean/variance; determine whether region is still Room1 and identify residual unknown and changed boundaries.
4. Recompute knownfree feasibility and candidate sensing masks; rank a new **minimal conditional goal sequence** with route/pathwise visibility and cost.
5. Only after freezing goals, compare against GT offline. Original D03 onward results never become policy input.

Current state: step 1 COMPLETE; steps 2–4 NOT RUN. New real MapEx run or native control NOT RUN.
