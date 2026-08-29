# STATUS

## Current stage

**Stage 2 — MapEx Nearest baseline (official runner: `scripts/nf_basic.py`)**

## Done

- Research workspace is `mapex_lab/`; official MapEx source is pinned at `53636bd1c79153acc3c74a532837d78c926bae5e`.
- Hospital v2 target runtime SLAM/policy grid is `0.10 m/cell`; fixed evaluation canvas remains `hospital_canvas_v1` at `0.05 m/cell` with frozen ROI `hospital_connected_free_v1` denominator `215435`.
- Nearest frontier semantics are fixed: free `==0`, unknown `<0`, 8-neighbour frontier, 8-connected regions, keep region `>10`, representative nearest arithmetic mean, Euclidean ranking.
- Hospital keeps exact frontier `x/y` as the execution target and ignores terminal yaw.
- **`mapex_lab/scripts/nf_basic.py` is the canonical/official execution file for the Nearest Frontier baseline.** `mapex_lab/scripts/nf_run.py` is the integrated measurement wrapper and must not diverge from the canonical exploration policy.
- Official `nf_basic.py` uses path-guided recovery: a failed main goal may use a temporary subgoal on the latest valid main-goal `/plan`, then retry the same frontier.
- Official `nf_basic.py` has two conservative completion conditions:
  1. zero frontier regions `>10` across 5 distinct map sweeps, >=2 s apart;
  2. frontier regions remain, but the full eligible representative set is revalidated with Nav2 `ComputePathToPose` and 5 consecutive planner sweeps find no reachable frontier.
- Both completion conditions still require >=10 s navigation idle and >=20 s node age. Any planner-reachable frontier resets terminal verification and exploration resumes.
- Main-plan diagnostics require at least 2 poses and the final `/plan` pose to be within `0.10 m` of the exact frontier before that path may be used for subgoal recovery.
- Closed-curtain runtime test showed a repeatable tolerance-snapped path: frontier `(19.48,-11.66)`, `/plan` endpoint `(19.88,-11.66)`, 7 poses, endpoint error `0.400 m`; this path is correctly rejected for subgoal recovery.
- Nav2 controller `xy_goal_tolerance` is currently `0.4 m`; this is controller success tolerance and is distinct from planner `GridBased.tolerance`.
- `config/nav2.yaml` explicitly overrides `planner_server.ros__parameters.GridBased.tolerance: 0.0`.
- With planner tolerance `0.0`, reachable goals produced exact-frontier paths (`endpoint_to_frontier=0.000 m`). For main frontier `(6.58,0.95)`, Nav2 first produced valid exact paths, then controller failure `105`, path-guided recovery, and after the robot pose changed the same main frontier began returning planner error `208` (`NO_VALID_PATH`) with no usable `/plan`.
- A main-goal `208` is abandoned immediately so exploration can move to another candidate, but its position is not treated as permanent terminal reachability evidence. If all ordinary candidates become suppressed, terminal planner revalidation explicitly checks those representatives again.
- Run `nearest_004` exposed the same infinite-retry pathology for planner error `206` (`GOAL_OCCUPIED`): frontier `(11.38,7.82)` was repeatedly accepted, failed with `206`, had no usable `/plan`, then was retried roughly once per second until manual Ctrl+C.
- `scripts/nf_basic.py` now treats main-goal `206 GOAL_OCCUPIED` the same way as `208 NO_VALID_PATH` for ordinary selection: suppress candidates within `0.10 m`, move to another frontier, and allow terminal `ComputePathToPose` revalidation to revive the frontier later. Other controller/execution failures remain eligible for path-guided recovery.
- `scripts/nf_run.py` records coverage/known fraction vs time/distance, odometry trajectory, decision computation time, full eligible candidate sets, main/subgoal attempts/results/error codes, `/plan` endpoint diagnostics, exact decision maps, periodic maps, final map, and `summary.json` under `experiments/nearest/<run_id>/`.
- `scripts/nf_run.py` writes `metadata.json` and `runtime_nav2_merged.yaml` at run start. The merged Nav2 file is reconstructed from the installed TurtleBot4 `config/nav2.yaml` plus `mapex_lab/config/nav2.yaml` using the same deep-merge semantics as the current launchers. `metadata.json` records git commit/dirty state, protocol/canvas/ROI identity, benchmark start, config SHA-256 hashes, Nav2 source paths, map resolution once observed, and final termination/result fields.
- `scripts/nf_run.py` preserves the exact `nf_basic.py` completion reason in `summary.json`/`metadata.json` (`no_frontier_region_gt_10` or `no_planner_reachable_frontier`) instead of reducing all automatic completion to the generic string `complete`.
- `scripts/nf_run.py` leaves `occupied_iou` and `tu` as online `NaN`/`None`; retained raw/fixed-canvas maps are the source for offline computation of those metrics.
- First runtime smoke-test exposed a Python import-name collision: `pathlib.Path` was overwritten by `nav_msgs.msg.Path`, causing `Path(__file__)` to fail before recording started. The recorder aliases these as `FilePath` and `NavPath`, respectively.
- Pilot `pilot_002` confirmed numeric ROI coverage (`0.452920835` at manual stop), odometry distance logging (`11.18 m`), exact-frontier `/plan` diagnostics, periodic/decision map snapshots, and a genuine `105 -> path-guided subgoal -> same-main retry -> success` sequence.
- Pilot `pilot_002` also exposed startup contamination from a transient NavigateToPose lifecycle state and an unclosed goal row when Ctrl+C occurred during an active goal.
- The recorder gates benchmark start until NavigateToPose has remained ready for at least `3.0 s` with map and TF available. The benchmark clock still starts at the first actual frontier decision after that gate opens.
- The recorder writes an active main/subgoal as `result=interrupted` during finalize/Ctrl+C instead of silently dropping it, and tracks `main_interrupted` / `subgoal_interrupted` separately from navigation failures.
- Pilot `pilot_003` passed the recorder smoke-test: readiness occurred before the first frontier decision, coverage was numeric (`0.446064938` at manual stop), and Ctrl+C during main goal 4 produced `result=interrupted` with `main_interrupted=1` in `summary.json`.
- To reduce disk use without losing the fixed evaluation representation, periodic `maps/snapshot_*` files save **fixed-canvas NPZ only**. Exact decision maps still save both raw + fixed-canvas, and the final map still saves both raw and fixed-canvas.
- `nearest_001` is retained as run 1 by user decision: exploration data are usable, final coverage `0.996184464`, distance `578.24 m`, but the run ended by manual Ctrl+C because the old completion logic deadlocked after the remaining large frontier representatives returned `208`. Any comparison/report must disclose this manual legacy termination rather than treating it as automatic terminal confirmation.
- The run-1 completion deadlock directly motivated the case-2 planner-reachability completion verification.
- `nearest_002` completed automatically under the current local-window debug workflow with coverage `0.993965697`, distance `525.85 m`, benchmark time `5897.07 s`, 59 selected frontiers, 50/62 successful main attempts (`80.65%`), 3 recovery subgoal attempts (2 success, 1 failure), and terminal reason `no_planner_reachable_frontier` after 5 stable planner sweeps.
- `nearest_003` completed automatically with coverage `0.992034720`, distance `479.20 m`, benchmark time `5663.42 s`, 56 selected frontiers, 42/58 successful main attempts (`72.41%`), 2 recovery subgoals (0 success, 2 failure), error counts `105:2`, `206:1`, `208:15`, and terminal reason `no_planner_reachable_frontier`.
- `nearest_004` is **not a valid completed run**: it was manually stopped at coverage `0.854499037`, distance `159.13 m` after the `206 GOAL_OCCUPIED` infinite-retry pathology. It should be deleted/replaced by a fresh `nearest_004` after the fix.
- `config/slam.yaml` and `config/slam_local.yaml` currently use `minimum_time_interval=0.5 s` in the active local-window debug workflow.
- `local.launch.py` remains a debug-only mapping frontend relative to the formal `hospital_v2` protocol. Results collected with it must keep that runtime-profile caveat until/if the protocol is deliberately changed and all compared methods are rerun under the same stack.
- Cartographer and local-window SLAM alternatives remain debug-only and are not part of the official Hospital v2 benchmark unless protocol identity is changed and all methods are rerun under the same stack.
- Historical controller/launch variants remain available only for separate regression diagnostics; they are not the official Nearest runner.

## In progress

- Runtime-validate the new `206 GOAL_OCCUPIED` suppression behavior: confirm one `206` abandons the current main frontier instead of retrying it forever, and confirm terminal planner revalidation can still revive it if it later becomes reachable.
- Continue runtime validation of the `no_planner_reachable_frontier` completion path and distinguish genuine terminal exhaustion from planner/global-costmap failure.
- Continue runtime validation of path-guided recovery for controller error `105`.
- Continue local-window SLAM diagnostics separately; the current local-window workflow remains a debug runtime profile relative to formal `hospital_v2`.

## Next actions

1. Pull the latest repo before the next run.
2. Delete the interrupted `experiments/nearest/nearest_004` folder and rerun `nearest_004` from the beginning.
3. During the rerun, if a main goal returns `206`, verify the next log is `Main frontier abandoned after GOAL_OCCUPIED (206)` followed by selection of another frontier, not repeated retries of the same `(x,y)`.
4. Keep `nearest_001` with its manual-termination caveat; keep `nearest_002` and `nearest_003` as completed local-window runs.
5. Continue the Nearest batch to at least 5 completed runs, target 10, after the `206` runtime check passes.
6. Before treating the batch as formal `hospital_v2` benchmark data, resolve the current `local.launch.py` debug-profile vs official-protocol mismatch and ensure Nearest/MapEx/proposed method all use the same effective SLAM/Nav2 stack.

## Important decisions

- **Official Nearest Frontier policy/runner source: `mapex_lab/scripts/nf_basic.py`.** This is the canonical algorithm file to maintain going forward.
- `scripts/nf_run.py` is an instrumentation wrapper around that canonical class for Nearest data collection; it must not become a divergent copy of the exploration policy.
- Hospital implementation is MapEx frontier/ranking semantics plus a ROS/Nav2 execution adapter; it is not claimed to be the original simulator implementation unchanged.
- Exact frontier `x/y` is the exploration target; planner endpoint is diagnostic/reachability evidence only.
- A path that stops materially short of the frontier must not be used to generate a recovery subgoal.
- Genuine controller/execution failure after a path reaches the exact frontier remains eligible for temporary path-guided subgoal recovery.
- A selected main frontier returning planner-blocking `GOAL_OCCUPIED (206)` or `NO_VALID_PATH (208)` is abandoned for ordinary selection within a `0.10 m` suppression radius so the node can move on; neither code is permanent proof that the frontier can never become reachable.
- When every eligible representative has been suppressed after `206/208`, `nf_basic.py` must re-run actual Nav2 `ComputePathToPose` checks over the full set. Five consecutive fully exhausted sweeps are required for `no_planner_reachable_frontier` completion.
- A failed temporary subgoal, including subgoal planner errors, does not alone prove the main frontier unreachable and therefore does not blacklist the main frontier.
- Official Nearest, MapEx, and proposed method comparisons must use the same Hospital adaptations and effective Nav2/SLAM configuration.
- Pilot/debug launch alternatives do not count as official benchmark runs unless the protocol is deliberately changed for every compared method.
- Storage policy: periodic snapshots keep the fixed canvas only; decision-time maps and final maps keep both raw and fixed-canvas forms.
- Per-run provenance is mandatory: every new `scripts/nf_run.py` run should contain `metadata.json` and `runtime_nav2_merged.yaml` from startup onward.

## Latest result

2026-08-29: `nearest_003` completed automatically at coverage `0.992034720` (99.2035%), distance `479.20 m`, benchmark time `5663.42 s` (~1 h 34 min 23 s). It selected 56 frontiers; 42 of 58 main navigation attempts succeeded (`72.41%`). Error counts were `208:15`, `105:2`, `206:1`; 14 frontiers were abandoned after `208`. Completion used the second terminal rule after repeated `ComputePathToPose` sweeps found no planner-reachable eligible frontier. A subsequent `nearest_004` attempt was interrupted at coverage `0.854499037`, distance `159.13 m` after repeated `206 GOAL_OCCUPIED` retries; commit `6c81982` changes main-goal `206` handling to the same suppress-and-terminal-revalidate policy as `208`, so `nearest_004` must be rerun from scratch.
