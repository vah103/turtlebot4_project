# G2-A: five offline one-scan counterfactuals on New Room Run04

Fixed points at D02, D08, D14, D18, D21 were chosen by human intuition, independently and prior to this offline scoring. No points were modified after viewing simulation outcomes.

GT source: exact native Run04 gt_map.png (1203x1263, WHITE=occupied, BLACK=free). Crop rows 500:703 and cols 500:763. This is not the separate SDF structural GT.

Standalone LiDAR approximation: 2500 beams, 20m, 360 deg, 0.1m/pixel; Bresenham to first wall plus visibility polygon. D01 calibration: 7404 known, matching native snapshot known total, with 7272 cells overlap (98.2%). This is NOT the exact MapEx native range_libc implementation.

Each scenario runs an isolated scan from exactly the current D snapshot at (a) original robot position, (b) fixed G2-A, (c) first native selected LOCAL goal. New FREE / WALL / semantic-room cells are counted against the observed map at that decision. GT was used ONLY to evaluate simulated scans, after viewpoint picks were fixed.

Travel lengths: G2-A is grid A* on observed FREE with 8 directions and no robot collision footprint; native goal is native A* length under a potentially different costmap. Do not treat distances as strictly comparable without harmonizing buffers.

No en-route sensor observations, navigation execution, scan integration or subsequent replanning. These are five independent ONE-SCAN counterfactuals, NOT a new full exploration run. Room is the 8-connected semantic ROOM component containing current robot; walls-near-room refer to a one-cell dilation.

Result summary: D02 G2-A 1047 vs native goal 808 new cells; D08 190 vs 2233; D14 462 vs 386; D18 322 vs 716; D21 528 vs 664. See HTML, JSON and CSV for all details.
