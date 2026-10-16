# MapEx Run 04 — Decision 18 / Room 4 — proposed G2-A viewpoints (2026-10-16)

Source visual: `mapex_lab/figures/mx072_newroom_optimized_20261015/MX072_NEWROOM_RUN04_vs_RUN05_REAL_25_DECISIONS.html`, section **D18**, panel **3 · Room Node: cấu trúc dự đoán** on branch `mx072-newroom-real-25-decisions-ipad-20261015` pinned to `e8b1c8a2b2672e5e39d4fd56b931c9f97c58de98`. The annotation is over an exact extracted source JPEG panel with a zoom of the same panel; MapEx map pixels are preserved except annotated markers.

## Scope
Only Run 04 D18 current-and-prior information was used for viewpoint selection. No D19–D25 result, final map, ground-truth map, hindsight future LiDAR, or any original selected goal was treated as a target answer. Run 04 full inventory verified: 25 decision JSON, 25 input NPZ, 25 semantic NPZ, 25 reports, 709 plan JSON, 709 summary JSON, 709 postscan NPZ, 709 observed PNG. Those later records were used for inventory checking only. Room 4 at D18: robot native padded row/col (553,540), core map shape (203,263), local frontiers 5, global frontiers 14, observed-free in room 3206, observed unknown in room 833.

## Viewpoint proposal at D18
- Start R: padded (row=553,col=540), coordinate x=4.0 m,y=5.3 m.
- VP1 initial proposed goal: padded (row=518,col=515), core (18,15), coordinate x=1.5 m,y=1.8 m.
- VP2 is NOT an initially committed goal: padded (row=551,col=506), core (51,6), coordinate x=0.6 m,y=5.1 m. Only reconsider based on later actual scans; it may be skipped or replaced.

The choice uses observed knownfree (value 0), R4.1 room mask (semantic value 1 connected to robot), LaMa mean and variance from D18, and conservative nominal predicted-wall raycasts. It never assumes predicted free is traversable. Nominal line-of-sight approximation: 823 unknown room cells are predicted-free under threshold mean<0.5; VP1 nominally reaches 733; VP2 adds 39 not seen from VP1 under the static approximation. Grid 4-connected observed-free distance start→VP1 is ~6.0 m. These are heuristics (not the actual MapEx probabilistic IG, not a certified native path, not actual sensor observations, not proven globally optimal). No actual new MapEx or LiDAR simulation was run.

Proposed causal sequence: R → VP1 → take new LiDAR scan → recompute unknown and room structure → retain/change/cancel VP2 accordingly → return to global MapEx when room is sufficiently assessed or defer residual unknown. Do not confuse schematic viewpoint ordering with the native A* path or a safety certificate.

Original Run 04 data on DELL, read-only:
`/home/dell/mx072_native2d_pair_20261015/OPTIMIZED_FULL_PAIR_20261015/COM1_VERIFIED/native_opt_r41_lama3small_run04/R41_OPT_LAMA3_SMALLROI_RUN04_visvarprob/`
