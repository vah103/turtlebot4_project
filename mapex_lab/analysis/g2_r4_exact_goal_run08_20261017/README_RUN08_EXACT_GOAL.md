# MX072 COM1 — G2-R4 exact-goal arrival: Run 08 vs Run 07 / Run 04 / Run 05

**2026-10-17** · Maker-run result; no independent IR1 acceptance or multi-seed replication. All original Run 04, 05, 07 logs preserved. New Run 08 copy is isolated, 2D only.

## Change under test (exactly one mechanism)

Run 07's source had conflicting arrival rules:

- MapEx native `cur_pose_dist_threshold_m: 1` unlocked any goal at distance below 1.0 m, including G2 viewpoint goals.
- G2's `g2_native_integration.G2Run.needs_replan()` also reselected its goal below 1.0 m.
- G2's evidence sidecar required distance <=0.5 m to confirm completed viewpoint; this caused false `INVALIDATED_EARLY` events and incomplete evidence/commitment memory.

Run 08 changes **only two branches**:

1. Native explorer: skip the generic **1-m early-unlock** only while a G2 LOCAL goal is locked; GLOBAL MapEx/UPEN behavior remains unchanged.
2. Native G2 adapter: trigger next G2 decision only when robot's grid pose equals G2 goal's grid pose exactly (`np.array_equal(g,p)`) or if A* route is no longer valid, goal becomes blocked/outside map, etc. The native pseudotrajectory controller moves 3 P1 cells at a time and automatically clamps the last movement to the A* goal endpoint. It is not necessary to reduce step size or add an extra scan. If path blocked, safety fallback remains intact.

**Unchanged SHA-256 bytes:** shared Run 04 LaMa 3-model, Room Node R4.1, G2-R4 sidecar/beam/chain valuation, R3.1 commitment, native A* and LiDAR. Original 1 m tolerance still applies to MapEx global. Crucially, no change to premature DEFER policy, no post-hoc D02 threshold tuning, no GT provided to goal selector, no Gazebo.

COM1 preflight: 8/8 new tests passed — includes 0.1–0.9 m partial approach without replan, exact-pixel arrival, blocked goal/route, path endpoint clamping from source function, shared-source hashes unchanged and only two source branches differ. Frozen source Run 07 left intact.

## Matched D02 time-to-same-coverage result

Same original D02 observed map at tick12 (zero different cells in all arms), P1 0.1m/cell, same snapshot-derived original room mask (6,564 cells; 3,889 initially UNKNOWN), same target <=270 initially unknown cells remaining; this is **93.057% of initially UNKNOWN observed**, not 93.1% of all room cells.

| Variant | Native tick reaching target | Additional native ticks since D02 | Travel since D02 (m) | Virtual travel at 0.3 m/s (s) |
| --- | ---: | ---: | ---: | ---: |
| **Run 04 — MapEx + Room Node LOCAL frontier** | 102 | 90 | **24.861** | 82.870 |
| **Run 08 — MapEx + Room Node + G2-R4 exact-goal** | 105 | 93 | **25.079** | 83.597 |
| Run 07 — same G2-R4 stops within 1 m | 625 | 613 | 173.205 | 577.350 |
| Run 05 — plain MapEx | 767 | 755 | 213.513 | 711.711 |

**Result:** Run 08 reaches D02 target with 148.126m *less travel* than Run 07, now within 0.218m of Run 04. This is one paired deterministic environment evaluation, not a multi-run scientific generalization.

At this D02 checkpoint, logged prediction+Room Node/native scoring+G2 values plus hypothetical 0.3 m/s driving (NOT physical clock time) are approximately: **Run04 89.669s**, **Run08 107.862s**, **Run07 723.544s**, **Run05 743.905s**. Run08 still spends about 18.193s more than Run04 on this proxy, chiefly because G2 goal/chain search is expensive. This proxy excludes extra native IO/simulation timings, and must not be presented as an exact robotic elapsed time. Same source-epoch optimized LaMa pipeline used for Run08 and Run07; only the G2 goal-completion logic changed.

## All-room / whole New Room outcome

| Metric | Run 04 | Run 05 | Run 07 | Run 08 |
| --- | ---: | ---: | ---: | ---: |
| Native steps (not scan-limited 0.5m; preserved source cadence) | 709 | 1298 | 1193 | **1099** |
| Total path travel (m) | **199.108** | 369.696 | 340.182 | **310.183** |
| Native worker COM1 elapsed (s) | **95.905** | 129.359 | 284.199 | **222.820** |
| Original 203x263 observed map UNKNOWN at terminal | 2085 | 2081 | 2084 | **2086** |
| Termination | SOURCE_TERMINAL | SOURCE_TERMINAL | SOURCE_TERMINAL | SOURCE_TERMINAL |

Native worker runtime is an algorithm/simulator wall measurement, **not physical driving time**. Run08 saves about **30.0 m** and **61.4s worker time** vs Run07, but **Run04 remains far better** on whole-map distance and compute time. Coverage unknown differences in final three pixels are negligible descriptively; same-Q threshold metric should be preferred. No claim of a completion or accuracy improvement from Run08.

## Viewpoint arrival and G2 feedback mechanism

- Run07: **17 G2 goals / 0 exact native odom visits** before next G2 call; **2 completed-goal ledger receipts**, **14 `INVALIDATED_EARLY`** receipts.
- Run08: **13 G2 goals / 13 exact native odom visits**, **10 completed-goal ledger receipts**, **0 `INVALIDATED_EARLY`** receipts.

The 10/13 completed receipts are from existing G2 sidecar `prior_observation_event` logging. Three other G2 goals were visited exactly but were not followed by a same-room G2 bookkeeping call; do **not** interpret 10 as exact-goal arrival rate. Exact spatial visits are verified from recorded odom trajectory. Future work could log goal-arrival events separately from next structural replanning, but Run08 deliberately did not modify that mechanism.

This confirms the user's observation that Viewpoint target selection itself was often sensible and Run07's early goal unlocking hurt execution and G2 memory. It does **not** prove goal quality is optimal, nor that remaining all-room unknown becomes fully observable from the viewpoint.

## Remaining limitations and next hypothesis

Run08 still uses G2 `DEFER_ROOM` to force MapEx GLOBAL even when Room Node may have reachable LOCAL WFD. Do not conflate "R4 chain value below threshold" with "room is fully explored." This is deliberately unchanged in Run08 to isolate the effect of exact-goal arrival. The next **separate** ablation would use LOCAL WFD fallback before GLOBAL, preserving this new goal-arrival behavior, and test on the same route/coverage metrics.

One environment, no IR1 science ACCEPT, no physical safety certificate, no hidden GT in online planner, source terminal is not GT completion, and the user asked for 2D only.

## Artifacts

- Raw COM1 run: `/work/com1/mx072_g2_r4_goal_exact_run08_20261017/native_g2_r4_goal_exact_run08/G2_R4_EXACT_GOAL_RUN08_visvarprob/`
- Synchronized raw DELL run: `/home/dell/mx072_g2_r4_exact_run08_com1_20261017/`
- Source guard: `guard_g2_r4_run08.py`, `native_g2_r4_3model_worker_run08.py`, exact-goal patched `explore_g2_r4_lama3_smallroi_run06.py`, `g2_native_integration.py`
- Raw measure: `evidence/RUN08_GOAL_ONLY_COMPARE_RUN04_05_07.json`; checked release `reports/RUN08_GOAL_ONLY_COMPARISON_VERIFIED.json`. Run05 model timing in verified output corrected to use `prediction_time_s` (32.194 s), not incorrectly default to zero.
- Self-contained HTML scroll `reports/RUN08_G2_R4_EXACT_GOAL_FULL_SCROLL.html`, **24 native decision cards** observed+Room Node+goal+mode; manifest+source generator.
- Exact terminal receipt `evidence/guard_run08_g2r4_exact/terminal_receipt.json`: SOURCE_TERMINAL, returncode 0, 1099 step records, 222.82s.
- Validation tests `test_exact_goal_run08.py`: 8/8 maker PASS.

Every primary gallery image/source is same-decision verified, no future/GT pictures and no manual goal injection.
