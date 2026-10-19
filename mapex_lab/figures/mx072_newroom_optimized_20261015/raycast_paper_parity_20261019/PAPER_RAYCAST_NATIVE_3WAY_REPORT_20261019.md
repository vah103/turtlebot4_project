# MapEx paper-cumulative raycast — New Room native benchmark
Date: 19/10/2026 | DELL | 3 new independent, sequential source-terminal runs

## Experimental question

The original MapEx Numba probabilistic raycast incorrectly initializes `accum_hit_prob=0` inside the per-cell loop of each ray. The MapEx paper describes a cumulative occupancy value per ray. This controlled experiment moves the same initialization from inside the per-cell loop to the start of each ray, leaving everything else unchanged. No file in `/home/dell/MapEx/` is modified. The runtime patch verifies SHA256 of upstream Numba source. The three policies share same 250 ray, 20 m predicted-visibility, same three LaMa ensembles, native WFD/A*, starting pose, New Room P1 and termination. The G2 SWEEP LOCAL policy is a byte-for-byte copy of frozen G2-SWEEP-R1.

## Offline faithful replay

Dataset: frozen COM1 25-decision original MapEx-only New Room native run. Original MapEx `get_vis_mask` including exact Shapely polygon buffer and Numba `flood_fill_simple`, with same saved mean, variance, WFD and pose. Replay obtains same selected goal in **25/25 decisions**. Numeric costs at first 11 decisions have small border-reconstruction residuals because evidence stores only cropped core 203×263 map; from D12 onward score arrays reproduce up to 1e-6. Thus equality refers to goal index, not full value-bitwise parity.

Changing only the raycast accumulator on same-epoch frozen inputs changes winner in **11/25** recorded states, at D02 D05 D06 D09 D10 D11 D14 D17 D18 D19 D21. This is a fixed-input counterfactual, **not** a prediction of the number of decisions changed in a real new run. D06 is sensitive to remaining border uncertainty (new top-two score gap 0.0191 versus original-code score reconstruction max delta 0.0268).

## Original-code versus cumulative-paper 2D native New Room

| Policy | Raycast mode | Total m | GLOBAL m | LOCAL m | Decisions | Native ticks | Terminal observed P1 |
|---|---|---:|---:|---:|---:|---:|---:|
| MapEx-only | code reset each cell | 369.696127 | 369.696127 | — | 25 | 1298 | 51,308 |
| MapEx-only | **paper cumulative** | **322.323991** | 322.323991 | — | 22 | 1135 | 51,304 |
| MapEx+G1 R4.1 | code reset each cell | **199.107583** | 94.928869 | 104.178714 | 25 | 709 | 51,304 |
| MapEx+G1 R4.1 | **paper cumulative** | **246.233821** | 167.849534 | 78.384287 | 23 | 875 | 51,304 |
| MapEx+G1+G2-SWEEP-R1 | code reset each cell | **213.507583** | 125.348219 | 88.159364 | 27 | 757 | 51,303 |
| MapEx+G1+G2-SWEEP-R1 | **paper cumulative** | **233.132505** | 145.278714 | 87.853791 | 30 | 827 | 51,303 |

All new run receipts report `SOURCE_TERMINAL`, `returncode=0`, condition `MAPEX_NO_SELECTABLE_FRONTIER_NOT_GT`, smoke three LaMa PASS and D01/D02 new-run variant-integrity PASS. They were executed sequentially on DELL:

- `NR-RAYMX-02`: 410.662 s wall, peak PSS 1,989 MiB.
- `NR-RAYRN-02`: 424.055 s wall, peak PSS 2,124 MiB.
- `NR-RAYSW-02`: 441.473 s wall, peak PSS 2,126 MiB.

Wall time includes full computation of native run; not simulation of Gazebo/SLAM Nav2 physics.

## Key findings on cumulative raycast

- G1 versus MapEx-only: **−76.090170 m (−23.61%)** travel, 246.234 versus 322.324 m.
- G2 SWEEP versus G1: **−13.101316 m (−5.321%)** travel, 233.133 versus 246.234 m.
- G2 versus MapEx-only: **−89.191486 m (−27.671%)** travel, 233.133 versus 322.324 m.
- G2 versus G1 splits into GLOBAL **−22.570820 m** (145.279 versus 167.850), LOCAL **+9.469504 m** (87.854 versus 78.384). Thus the paper-cumulative SWEEP benefit in this single New Room case is from GLOBAL, **not LOCAL**, unlike the original reset-cell history.
- Raycast change alone improves MapEx-only by **−47.372136 m** but lengthens G1 by **+47.126238 m** and G2-SWEEP by **+19.624922 m**, underscoring the strong interaction between policy and scorer.
- Terminal known-mask comparison (paper G1 versus paper SWEEP): 51,299 cells known in common, 5 G1-only, 4 G2-only; **9-cell symmetric difference**. This is close but not strictly identical terminal coverage.
- Paper SWEEP versus original-code SWEEP final core known masks are **bit-identical** (51,303 cells each). Paper G1 versus original-code G1: symmetric difference 8 cells (4 each).
- Paper MapEx-only terminal P1 known count 51,304; G1 51,304; SWEEP 51,303; exact cell sets differ slightly even when counts equal. End condition means no selectable WFD, not GT-validated 100% coverage.

## Interpretation and cautions

This is **one fixed-start native New Room run per variant**, not a statistical result across initial conditions or maps; it does not establish reliable dominance in Hospital or Gazebo. The raycast change causes altered early choices, after which observed/LaMa, frontier set and GLOBAL/LOCAL transitions change endogenously. Do not transfer the old "G1 wins by 46%" or "G2 loses on GLOBAL" estimates to the paper-cumulative variant. Similarly do not claim paper-cumulative raycast will categorically improve every policy: in this test MapEx alone improved, but G1/G2 absolute distance increased relative to their own old variants.

Future robust comparison: Hospital, a range of starting positions/maze geometries, motion and LiDAR noise using Gazebo/ROS 2, paired experiments across methods, time-to-coverage curves and completion at pre-agreed observed-map or GT evaluation threshold. GT is allowed for offline final evaluation, **never for online decisions**.

## Artifacts

- Frozen 25-decision replay `REPLAY_25.json`, exact-source replay `offline_raycast_parity.py`.
- Runtime source-safe accumulator correction `raycast_paper_patch.py`.
- Native sequential source-staging and guarded runner `deploy_paper_raycast_native.py`.
- Six terminal observed full maps with native odometry traces `PAPER_RAYCAST_3WAY_FULL_TRAJECTORIES_SCROLL.html`.
- Detailed per-run/decision metrics and terminal known-mask comparisons `PAPER_RAYCAST_NATIVE_3WAY_COMPARISON.json`.
- Sources frozen in `/home/dell/DELL_NATIVE2D/runs/NR-RAYMX-02/`, `NR-RAYRN-02/`, `NR-RAYSW-02/`. Original run folders unchanged.
