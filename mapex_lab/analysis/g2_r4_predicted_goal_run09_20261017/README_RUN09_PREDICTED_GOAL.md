# Run 09 — G2-R4 Predicted Goal (MapEx + Room Node V4-R4.1)

**Native 2D COM1, 2026-10-17.** This is an experimental maker run; no independent IR1 QA and no multi-seed/statistical claim. Original Run 04/05/07/08 are not modified.

## Requested modification

The user requested permission for G2-R4 to choose viewpoints in **UNKNOWN areas predicted FREE by LaMa**, rather than restricting every selected goal to LiDAR-observed FREE. The implementation strictly separates **hypothetical predicted viewpoint selection** from **physically executable known-FREE waypointing**.

Run08 base code preserved. Main additions:

1. `g2_predicted_goals.py`: propose candidate UNKNOWN viewpoints within accepted Room Node room component, with occupancy-mean and ensemble-variance heuristics, morphological clearance, hypothetical 4-neighbor route weighted against UNKNOWN, a capped 24-cell unknown-depth horizon, and predicted MapEx/LaMa raycast visibility improvement over current R4 observed-free viewpoint. Heuristic gates: mean occupancy <=0.17, variance <=0.045, raycast gain at least 25 cells and 15% more than baseline. These gates are **NOT calibrated safety probabilities**.
2. `g2_r4_policy_sidecar.py`: augment the **old unchanged R4 structural-chain scorer** with predicted candidate selection only if it has a usable known-FREE baseline goal. Record predicted long-term viewpoint, the exact current executable FREE subgoal, original R4 baseline and two-stage continuation. Do not override R4 DEFER with fabricated coverage.
3. `g2_native_integration.py`: pass only the **live native RoomNode-scoped traversable mask** (observed known-FREE, native inflation, room membership) to the predictor; validate safe subgoal through the native A* grid. The output of a speculative plan is **never directly commanded** as a robot goal. Re-evaluate after each scan/replan. If an UNKNOWN target becomes confirmed FREE, it can be executed directly as a normal known-FREE goal. If unsafe/unreachable, abandon/reselect without UNKNOWN traversal.
4. `explore_g2_r4_lama3_smallroi_run06.py` is byte-for-byte the Run08 exact-viewpoint-arrival variant: arrival must be same goal cell for G2, source MapEx global 1m tolerance untouched. Room Node source, LaMa ensemble 3, scan, A*, input P1 map, terminal and default DEFER→GLOBAL behavior unchanged.

This run changes goal candidate generation, goal value comparison and safe execution subgoal semantics together, because all three are necessary to allow selecting an UNKNOWN target while enforcing no speculative motion. **It does not change RoomNode or fix early DEFER.**

## Preflight and source evidence

- 5/5 synthetic/frozen snapshot predicted-goal tests PASS (known-FREE executable, predicted target stays UNKNOWN, input native scope required, reject occupancy-mean bad predictions/known walls, original D02 predicted candidate).
- D02 source snapshot test: original G2-R4 baseline goal **core (171,135)**; new predicted target **core (171,167) UNKNOWN**, native-certifiable FREE subgoal **core (165,131)** in eroded source-snapshot replay. Full sidecar test also passed and returned the UNKNOWN target with only a FREE subgoal.
- During native execution, Run09 chose 9 predicted/continue goals corresponding to 5 unique UNKNOWN viewpoint targets. Four of five were eventually visited **after being confirmed FREE**; one was dropped.
- Independent postrun scan of same-tick observed input and all G2 native goals: **15/15 executable G2 goals were observed FREE**, and **15/15 appeared in recorded odometry exactly** (as of later native poses). No unexpected native goal in UNKNOWN; source policy GT-input prohibited. This is simulated-cell safety evidence, NOT a physical robot safety certificate.

## Full native run result

| Metric | Run04 MapEx+RoomNode | Run05 plain MapEx | Run07 G2-R4 early 1m stop | Run08 G2-R4 exact knownfree | **Run09 predicted goal** |
|---|---:|---:|---:|---:|---:|
| Whole-map distance (m) | **199.108** | 369.696 | 340.182 | 310.183 | **282.224** |
| Native step records | 709 | 1298 | 1193 | 1099 | **1006** |
| Native worker wall time (s) | **95.905** | 129.359 | 284.199 | 222.820 | **265.030** |
| Final core UNKNOWN cells | 2085 | 2081 | 2084 | 2086 | **2083** |

Run09 has **27.959m less travel (9.0%)** than Run08, but native worker wall time **42.210s longer (18.9%)**. Run04 remains better than Run09 on full-map travel, and Run05 retains its own globally scoped planner. Native worker wall time is computational/simulator runtime, **not physical robot driving time**, which must additionally be evaluated under speed/dynamics assumptions.

## Same D02 original room coverage endpoint — critical negative finding

Room mask is frozen from the **original D02 same-snapshot Room Node memory**, 6,564 pixels and initially 3,889 UNKNOWN. The evaluation stops at **<=270 UNKNOWN remaining**, or 93.057% of initially-unknown cells resolved (NOT 93.1% of room area).

| Arm | First tick reaching <=270 | Distance travelled since D02 (m) |
|---|---:|---:|
| Run04 | 102 | **24.861** |
| Run08 | 105 | **25.079** |
| Run07 | 625 | **173.205** |
| **Run09** | **756** | **208.173** |
| Run05 | 767 | **213.513** |

Run09 leaves original D02 room when its G2-R4 `DEFER` fires at **tick 73** with **295 initially unknown D02 room cells remaining**, missing the target by 25 cells. MapEx global later visits/observes it, reaching <=270 only at tick756. This dramatically worsens *D02 time-to-same-room-coverage* vs Run08. The early-DEFER LOCAL→GLOBAL bug/semantics were **deliberately left unchanged** to avoid combining interventions.

Optional 0.3m/s virtual drive-time plus recorded prediction, RoomNode+MapEx score and G2 planning CPU/wall components through this milestone (not physical measured time):
- Run04: **89.669s**.
- Run08: **107.862s**.
- Run09: **845.154s**.
- Run05: **743.905s** (the `prediction_time_s` source key was used, not the zero-default key).
These are crude comparable proxies and exclude movement overhead and scan/IO; actual native worker wall times are listed in the whole-run table.

## What the new capability proves — and does NOT prove

**Proven in this 2D run:** G2 can choose UNKNOWN predicted viewpoint coordinates, preserve some across new LiDAR scans, and only physically command known-free A*-connected subgoals; the proposed target may become confirmed free and then be directly reached. Four of five unique predicted goals were eventually reached, safely under source-snapshot FREE/A* checks.

**Not proven:** Prediction is reliable outside this simulated map; lower total travel necessarily reduces actual robot total elapsed time; the new planner is better at completing each room; prediction variance is a calibrated collision probability; the best deep-room viewpoint was found; all GT is explored; route safety transfers to physical hardware.

A next *separate* ablation should preserve this predictor while changing **DEFER→LOCAL WFD fallback** to avoid long late return trips; also improve the value of predicted long-term commitment to avoid prematurely switching its goal before it becomes observable. Do NOT tune these gates by held-out ground-truth outcomes in Run09.

## Preserved artifacts

**COM1 run root:** `/work/com1/mx072_g2_r4_predicted_goal_run09_20261017/`.

**DELL synced root:** `/home/dell/mx072_g2_r4_predicted_goal_run09_20261017/`.

- Native output: `native_g2_r4_predicted_run09/G2_R4_PREDICTED_GOAL_RUN09_visvarprob/`, observed maps, odometry, per-tick paths, same-epoch RoomNode input/semantic, G2 decision JSON and native certified `native_scope_*.npz`.
- Guard: `evidence/guard_run09_predicted/terminal_receipt.json` = `SOURCE_TERMINAL`, return code 0, **1,006 records**, worker wall 265.03 s.
- Full HTML scroll: `reports/RUN09_G2_R4_PREDICTED_GOAL_FULL_SCROLL.html` has **28 actual decision cards**, with observed map, RoomNode overlay, orange-outline hypothetical predicted goal, green executable FREE waypoint, current robot, and GLOBAL/LOCAL mode. Six or more duplicate fallback audit records are not additional decisions.
- Numeric measures `reports/RUN09_PREDICTED_COMPARISON_VERIFIED.json`, decision trace `reports/RUN09_G2_DECISIONS_CAUSAL.json`, safety/target audit `reports/RUN09_PREDICTED_GOALS_AUDIT.json`; reproduction code `compare_run09.py`, `generate_run09_full_scroll.py`; synthetic tests `test_predicted_goal_run09.py`.
