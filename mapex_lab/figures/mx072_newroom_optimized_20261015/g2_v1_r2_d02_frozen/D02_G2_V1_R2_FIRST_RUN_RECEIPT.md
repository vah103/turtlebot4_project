# G2-V1 R2: frozen first D02 test, no prior goal tuning (2026-10-16)

## Source and QA
- Pin **`2a29c4f99f8b4d52a4fbc97967a0d874ac113002`**, new branch `mx072-g2-v1-r2-room-sequence-20261016`; original G2-V1 and native MX072 Run04 untouched.
- DELL workspace: `/home/dell/mx072_g2_v1_r2_standalone_20261016`.
- 15/15 synthetic/unit tests PASS independently on DELL after fetching this commit; `sha256sum mapex_g2_v1/planner.py = 110cbcb2d5bae1017ca7e4f12919d71ba61eb3ffc23ca8ffa6db73ef1ac139f6`.
- First run on original Run04 D02 `decision_000002_input.npz` + `decision_000002_semantic.npz`, robot core (147,111), no GT, D03+, post-VP1 scan or later mean/R4.1.
- Exactly default G2Config, **no per-D02 tuning**.

## First frozen planner output
- **1 goal**: native padded (row,col)=(680,650), core (180,150), **x=15.0m, y=18.0m**. Observed-free grid shortest route **7.2 m**.
- Run time **5.88 s** for G2 only (not LaMa/R4.1/native controller); 199 sampled viewpoints.
- 2,500 beams during dense shortlist; coarse 250; 2 geometrical scenario proxies from mean/variance because individual LaMa members were not provided.
- Objective **99.4755 s-equivalent**; expected weighted observed 3,755.36 of total 4,046.93; remaining 291.57 weighted units. This is **prediction, not verified observed-room coverage**.
- Coarse search: best 1-goal objective 102.277, 2-goal 102.254; dense rerank preferred 1 goal.
- GOAL 1 isn't the manually selected old VP1. Independent dense cost check using exactly the same current D02 predictions/coefficients:
  - Original VP1 (13.5,16.8)m, route 4.5m: objective **114.31**, expected observed weighted 3,635.98.
  - R2 goal (15.0,18.0)m, route 7.2m: objective **99.48**, expected observed weighted 3,755.36.
  - Original V1 G1 (13.8,17.1)m, route 5.1m: objective **104.01**.
  This difference is driven by *uncalibrated* two-map scenario and uncertainty/travel weights, not actual LiDAR evidence. It should not be construed as proof that the longer route improves physical exploration.

## Limits
This is an offline **plan-only** result; no G2-driven pathwise sensor rollout or runtime/quality comparison to the original MapEx has been run. The route is constrained to knownfree core cells and inflated 2 cells around unknown/occupied; not native collision certification. The two fallback map scenarios are thresholded mean/variance proxies, not actual per-model ensemble outputs or statistically calibrated prediction. Replanning after arrival/change requires external LaMa+G1 recomputation.

## Next required scientific QA
Freeze the R2 output before GT-based LiDAR evaluation, then simulate observed updates along its route, run LaMa+R4.1 fresh, assess event-trigger and recompute whole remaining goal sequence; compare coverage/distance/time against frozen V1, manual old VP1, and native MapEx. The current output is neither accepted as improved nor rejected based solely on goal count.
