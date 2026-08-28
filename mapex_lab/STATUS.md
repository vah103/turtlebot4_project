# STATUS

## Current stage

**Stage 2 — MapEx Nearest baseline (official runner: `nf_basic.py`)**

## Done

- Research workspace is `mapex_lab/`; official MapEx source is pinned at `53636bd1c79153acc3c74a532837d78c926bae5e`.
- Hospital v2 target runtime SLAM/policy grid is `0.10 m/cell`; fixed evaluation canvas remains `hospital_canvas_v1` at `0.05 m/cell` with frozen ROI `hospital_connected_free_v1` denominator `215435`.
- Nearest frontier semantics are fixed: free `==0`, unknown `<0`, 8-neighbour frontier, 8-connected regions, keep region `>10`, representative nearest arithmetic mean, Euclidean ranking.
- Hospital keeps exact frontier `x/y` as the execution target and ignores terminal yaw.
- **`mapex_lab/nf_basic.py` is now the canonical/official execution file for the Nearest Frontier baseline.** Older Nearest/debug controller files are not the main runner unless explicitly selected for a separate diagnostic.
- Official `nf_basic.py` uses path-guided recovery: a failed main goal may use a temporary subgoal on the latest valid main-goal `/plan`, then retry the same frontier.
- Official `nf_basic.py` now has two conservative completion conditions:
  1. zero frontier regions `>10` across 5 distinct map sweeps, >=2 s apart;
  2. frontier regions remain, but the full eligible representative set is revalidated with Nav2 `ComputePathToPose` and 5 consecutive planner sweeps find no reachable frontier.
- Both completion conditions still require >=10 s navigation idle and >=20 s node age. Any planner-reachable frontier resets terminal verification and exploration resumes.
- Main-plan diagnostics require at least 2 poses and the final `/plan` pose to be within `0.10 m` of the exact frontier before that path may be used for subgoal recovery.
- Closed-curtain runtime test showed a repeatable tolerance-snapped path: frontier `(19.48,-11.66)`, `/plan` endpoint `(19.88,-11.66)`, 7 poses, endpoint error `0.400 m`; this path is correctly rejected for subgoal recovery.
- Nav2 controller `xy_goal_tolerance` is currently `0.4 m`; this is controller success tolerance and is distinct from planner `GridBased.tolerance`.
- `config/nav2_stock_xy_only.yaml` explicitly overrides `planner_server.ros__parameters.GridBased.tolerance: 0.0`.
- With planner tolerance `0.0`, reachable goals produced exact-frontier paths (`endpoint_to_frontier=0.000 m`). For main frontier `(6.58,0.95)`, Nav2 first produced valid exact paths, then controller failure `105`, path-guided recovery, and after the robot pose changed the same main frontier began returning planner error `208` (`NO_VALID_PATH`) with no usable `/plan`.
- A main-goal `208` is still abandoned immediately so exploration can move to another candidate, but its position is no longer treated as permanent terminal reachability evidence. If all ordinary candidates become suppressed, terminal planner revalidation explicitly checks those representatives again.
- Added `mapex_lab/stage2_run.py` as a single integrated Stage-2 measurement wrapper. It inherits the canonical `NearestEuclideanFrontier` from `nf_basic.py` rather than copying policy logic, and records coverage/known fraction vs time/distance, odometry trajectory, decision computation time, full eligible candidate sets, main/subgoal attempts/results/error codes, `/plan` endpoint diagnostics, exact decision maps, periodic maps, final map, and `summary.json` under `experiments/nearest/<run_id>/`.
- `stage2_run.py` leaves `occupied_iou` and `tu` as online `NaN`/`None`; retained raw/fixed-canvas maps are the source for offline computation of those metrics.
- First runtime smoke-test exposed a Python import-name collision: `pathlib.Path` was overwritten by `nav_msgs.msg.Path`, causing `Path(__file__)` to fail before recording started. `stage2_run.py` now aliases these as `FilePath` and `NavPath`, respectively.
- Pilot `pilot_002` confirmed numeric ROI coverage (`0.452920835` at manual stop), odometry distance logging (`11.18 m`), exact-frontier `/plan` diagnostics, periodic/decision map snapshots, and a genuine `105 -> path-guided subgoal -> same-main retry -> success` sequence.
- Pilot `pilot_002` also exposed startup contamination from a transient NavigateToPose lifecycle state and an unclosed goal row when Ctrl+C occurred during an active goal.
- `stage2_run.py` now gates the benchmark start until NavigateToPose has remained ready for at least `3.0 s` with map and TF available. The benchmark clock still starts at the first actual frontier decision after that gate opens.
- `stage2_run.py` now writes an active main/subgoal as `result=interrupted` during finalize/Ctrl+C instead of silently dropping it, and tracks `main_interrupted` / `subgoal_interrupted` separately from navigation failures.
- Pilot `pilot_003` passed the recorder smoke-test: `STAGE2 READY` occurred before the first frontier decision, coverage was numeric (`0.446064938` at manual stop), and Ctrl+C during main goal 4 produced `result=interrupted` with `main_interrupted=1` in `summary.json`.
- To reduce disk use without losing the fixed evaluation representation, periodic `maps/snapshot_*` files now save **fixed-canvas NPZ only**. Exact decision maps still save both raw + fixed-canvas, and the final map still saves both raw + fixed-canvas.
- A long `local.launch.py` diagnostic run exposed the old terminal deadlock: coverage reached `0.996184464`, the remaining large frontier representatives all returned `208`, then the process repeated `no eligible candidate ... NOT declaring exploration complete` indefinitely until manual Ctrl+C.
- That diagnostic directly motivated the new case-2 planner-reachability completion verification. `local.launch.py` remains debug-only and the manually stopped run is not an official Hospital-v2 benchmark result.
- Cartographer and local-window SLAM alternatives remain debug-only and are not part of the official Hospital v2 benchmark unless protocol identity is changed and all methods are rerun under the same stack.
- `control_tb4.py` and historical `hospital_slam.yaml=0.05` remain available only for separate regression diagnostics; they are not the official Nearest runner.

## In progress

- Runtime-validate the new `no_planner_reachable_frontier` completion path in `nf_basic.py`.
- Confirm the expected end-of-run sequence: all normal candidates suppressed after `208` -> planner revalidation every >=2 s -> 5 exhausted sweeps -> one `COMPLETE`.
- Confirm that if any revalidation sweep finds a non-empty successful `ComputePathToPose`, the corresponding suppression is removed, completion streak resets, and normal nearest-frontier exploration resumes.
- Continue runtime validation of path-guided recovery for controller error `105`.
- Continue local-window SLAM and Cartographer diagnostics separately; neither is yet an official benchmark change.

## Next actions

1. Pull the latest repo and run a fresh terminal-validation test before accepting a new official `nearest_001`.
2. At the end of exploration, verify logs show `Planner-reachability completion verification: 1/5 ... 5/5` followed by `EXPLORATION COMPLETE` with reason `no_planner_reachable_frontier` when only unreachable frontiers remain.
3. Also verify a deliberately/recurrently reachable candidate discovered during revalidation resets the streak and resumes navigation instead of false-completing.
4. After the completion path passes, run official Nearest runs as `nearest_001`, `nearest_002`, ... using `stage2_run.py` with the intended Hospital-v2 stack.
5. After `nearest_001`, verify `metrics.csv`, `trajectory.csv`, `decisions.csv`, `candidates.csv`, `goals.csv`, `plans.csv`, `decision_maps/`, `maps/`, and `summary.json` for consistency before continuing the batch.
6. Run the Nearest baseline closed-loop at least 5 times, target 10, and report per-run plus mean ± std.
7. Before collecting/finalizing official benchmark data, ensure the active SLAM/Nav2 runtime matches the intended Hospital v2 configuration and archive effective runtime provenance.

## Important decisions

- **Official Nearest Frontier policy/runner source: `mapex_lab/nf_basic.py`.** This is the canonical algorithm file to maintain going forward.
- `stage2_run.py` is an instrumentation wrapper around that canonical class for Stage-2 data collection; it must not become a divergent copy of the exploration policy.
- Hospital implementation is MapEx frontier/ranking semantics plus a ROS/Nav2 execution adapter; it is not claimed to be the original simulator implementation unchanged.
- Exact frontier `x/y` is the exploration target; planner endpoint is diagnostic/reachability evidence only.
- A path that stops materially short of the frontier must not be used to generate a recovery subgoal.
- Genuine controller/execution failure after a path reaches the exact frontier remains eligible for temporary path-guided subgoal recovery.
- A selected main frontier returning `NO_VALID_PATH (208)` is abandoned for ordinary selection so the node can move on, but `208` is not permanent proof that the frontier can never become reachable.
- When every eligible representative has been suppressed after `208`, `nf_basic.py` must re-run actual Nav2 `ComputePathToPose` checks over the full set. Five consecutive fully exhausted sweeps are required for `no_planner_reachable_frontier` completion.
- A failed temporary subgoal, including subgoal error `208`, does not alone prove the main frontier unreachable and therefore does not blacklist the main frontier.
- Official Nearest, MapEx, and proposed method comparisons must use the same Hospital adaptations and effective Nav2/SLAM configuration.
- Pilot/debug launch alternatives do not count as official benchmark runs.
- Storage policy: periodic Stage-2 snapshots keep the fixed canvas only; decision-time maps and final maps keep both raw and fixed-canvas forms.

## Latest result

2026-08-29: long local-window diagnostic reached coverage `0.996184464` but exposed a completion deadlock after the final reachable frontier: remaining frontier representatives repeatedly returned `NO_VALID_PATH (208)`, became suppressed, and the old code refused to complete because large frontier regions still existed. `nf_basic.py` now implements a second terminal condition consistent with the protocol: when all eligible representatives are suppressed, it performs repeated full-set `ComputePathToPose` revalidation; 5 consecutive exhausted sweeps, >=2 s apart and satisfying idle/startup guards, produce `COMPLETE` with reason `no_planner_reachable_frontier`. If any candidate becomes planner-reachable again, its suppression is cleared and exploration resumes.
