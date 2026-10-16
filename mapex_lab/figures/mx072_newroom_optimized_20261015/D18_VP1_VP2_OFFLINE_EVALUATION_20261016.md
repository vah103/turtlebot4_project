# D18 G2-A — Counterfactual static LiDAR offline result

## Provenance

Run 04 on DELL in `/home/dell/mx072_native2d_pair_20261015/OPTIMIZED_FULL_PAIR_20261015/COM1_VERIFIED/native_opt_r41_lama3small_run04/R41_OPT_LAMA3_SMALLROI_RUN04_visvarprob/`. Read only from run data. Decision 18, tick 437, Room 4, current robot native (553,540). Inputs: `room_r41/inference/decision_000018_input.npz` (observed, mean, variance), `decision_000018_semantic.npz` and root `gt_map.png` **only as retrospective raycasting truth**. Original MapEx data untouched.

**Viewpoint selection was already frozen before this GT-based offline evaluation**, and was based on observed + predicted map at D18: VP1 native (518,515) = (x=1.5m, y=1.8m); VP2 native (551,506) = (x=0.6m, y=5.1m). No D19 onward scan, map, or route is used to select or revise them. The actual GT wall occupancy mask is white pixels in GT png, cropped [500:703, 500:763] to 203x263 core. Observed map labels 0 free / 1 unknown / 2 occupied.

## Simulator approximation (NOT source-exact)

Emulate **2500** 360-degree rays of max **20m**, same nominal sensor settings as MapEx `configs/base.yaml`, on 0.1 m source grid. Along each ray sample at 0.25 grid-cell intervals (fine-sampled discrete approximation) and take all grid cells before first GT obstacle as observed free, first GT hit cell as observed occupied. Union rasterized returns, intersect with D18 originally unknown cells. No extra scans along the route and no MapEx/LaMa invocation. This is a **counterfactual static scan**, not a native `range_libc + Polygon.buffer + flood_fill` reproduction, actual LiDAR or an autonomous rollout.

D18 room (G1 connected semantic==1 excluding observed occupied): 4039 cells, 3206 observed-free, 833 unknown. GT-only retrospective diagnostic among these 833: 760 true free, 73 true occupied. 19 observed-free cells mismatch with GT occupancy in full core (source alignment/raster rounding limitation).

| Stage | New unknown cells inside Room 4 | True free | True occupied | Residual unknown in Room 4 | Additional unknown outside Room 4 |
|---|---:|---:|---:|---:|---:|
| VP1 only | **766** | **757** | **9** | **67** | 93 |
| VP2 after VP1 | **7** | **3** | **4** | **60** | 3 |
| Both | **773** | **760** | **13** | **60** | 96 |

A coarser 1.0-cell ray sample instead produces VP1=765 and VP2 additional=8; 0.5-cell and 0.25-cell agree at VP1=766 and VP2 additional=7 (union always 773). The 60 residual unknown cells happen to be GT-wall cells in this evaluator; the online planner has **no right to know that** or mark Room 4 complete from GT.

Path length estimated via 4-connected shortest route in D18 observed-free Room 4 with 3x3 unknown/occupied dilation (~one-cell clearance): start→VP1 6.0 m and VP1→VP2 5.6 m; 5x5 dilation VP1→VP2 6.0 m. Grid feasibility is **not continuous footprint/dynamic safety certification**. Also, these estimates ignore information that actual LiDAR would gather while traveling; no wall-clock time or coverage-vs-distance claim.

**Recommendation for a conditional G2/G3 policy:** Execute/verify VP1 only, observe LiDAR, then recompute room boundary, unknown, and safe route. VP2 is expensive for only seven additional cells in this static experiment, so it should normally be cancelled or replaced, not blindly dispatched. This is an offline one-room diagnostic, not proof of global distance/time savings.
