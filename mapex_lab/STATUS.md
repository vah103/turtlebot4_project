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
- Official `nf_basic.py` uses the event-driven BT and conservative completion verification (5 distinct no-frontier map sweeps, >=2 s apart, >=10 s idle, >=20 s node age).
- Main-plan diagnostics require at least 2 poses and the final `/plan` pose to be within `0.10 m` of the exact frontier before that path may be used for subgoal recovery.
- Closed-curtain runtime test showed a repeatable tolerance-snapped path: frontier `(19.48,-11.66)`, `/plan` endpoint `(19.88,-11.66)`, 7 poses, endpoint error `0.400 m`; this path is correctly rejected for subgoal recovery.
- Nav2 controller `xy_goal_tolerance` was reduced from `0.4 m` to `0.3 m`; the curtain `/plan` endpoint still remained exactly `0.400 m` from the frontier, so controller tolerance was not the cause of the planner endpoint offset.
- `config/nav2_stock_xy_only.yaml` explicitly overrides `planner_server.ros__parameters.GridBased.tolerance: 0.0`.
- With planner tolerance `0.0`, reachable goals produced exact-frontier paths (`endpoint_to_frontier=0.000 m`). For main frontier `(6.58,0.95)`, Nav2 first produced valid exact paths, then controller failure `105`, path-guided recovery, and after the robot pose changed the same main frontier began returning planner error `208` (`NO_VALID_PATH`) with no usable `/plan`.
- Official `nf_basic.py` treats a **main-goal** error `208` as terminal for that selected frontier: it clears the pending main goal, records the rejected position, and suppresses future frontier representatives within `0.10 m` for the rest of that `nf_basic.py` process so exploration can continue to another candidate. A subgoal error `208` does **not** blacklist the main frontier by itself.
- Runtime test confirmed the `208 -> abandon selected main frontier` behavior resolves the observed infinite retry loop, and this behavior is retained in the official `nf_basic.py` runner.
- Added `mapex_lab/stage2_run.py` as a single integrated Stage-2 measurement wrapper. It inherits the canonical `NearestEuclideanFrontier` from `nf_basic.py` rather than copying policy logic, and records coverage/known fraction vs time/distance, odometry trajectory, decision computation time, full eligible candidate sets, main/subgoal attempts/results/error codes, `/plan` endpoint diagnostics, exact decision maps, periodic maps, final map, and `summary.json` under `experiments/nearest/<run_id>/`.
- `stage2_run.py` leaves `occupied_iou` and `tu` as online `NaN`/`None`; retained raw/fixed-canvas maps are the source for offline computation of those metrics.
- First runtime smoke-test exposed a Python import-name collision: `pathlib.Path` was overwritten by `nav_msgs.msg.Path`, causing `Path(__file__)` to fail before recording started. `stage2_run.py` now aliases these as `FilePath` and `NavPath`, respectively.
- Pilot `pilot_002` confirmed numeric ROI coverage (`0.452920835` at manual stop), odometry distance logging (`11.18 m`), exact-frontier `/plan` diagnostics, periodic/decision map snapshots, and a genuine `105 -> path-guided subgoal -> same-main retry -> success` sequence.
- Pilot `pilot_002` also exposed startup contamination from a transient NavigateToPose lifecycle state and an unclosed goal row when Ctrl+C occurred during an active goal.
- `stage2_run.py` now gates the benchmark start until NavigateToPose has remained ready for at least `3.0 s` with map and TF available. The benchmark clock still starts at the first actual frontier decision after that gate opens.
- `stage2_run.py` now writes an active main/subgoal as `result=interrupted` during finalize/Ctrl+C instead of silently dropping it, and tracks `main_interrupted` / `subgoal_interrupted` separately from navigation failures.
- Cartographer and local-window SLAM alternatives remain debug-only and are not part of the official Hospital v2 benchmark unless protocol identity is changed and all methods are rerun under the same stack.
- `control_tb4.py` and historical `hospital_slam.yaml=0.05` remain available only for separate regression diagnostics; they are not the official Nearest runner.

## In progress

- Re-run one short `stage2_run.py` pilot after startup-gating/interruption fixes and verify the first frontier is issued only after `STAGE2 READY` and Ctrl+C produces an `interrupted` row for any active goal.
- Continue runtime validation of the official `nf_basic.py` over longer Hospital exploration runs.
- Keep path-guided subgoal recovery for genuine execution failures such as controller error `105` when a valid path reaches the exact frontier.
- Observe whether a `0.10 m` session suppression radius is sufficient to prevent an unreachable region from reappearing via a slightly shifted representative.
- Continue local-window SLAM and Cartographer diagnostics separately; neither is yet an official benchmark change.

## Next actions

1. Pull the latest repo and run a short `stage2_run.py --run-id pilot_003` with the intended Hospital stack active.
2. Confirm log order contains `STAGE2 READY` before the first `Selected nearest frontier` and no pre-benchmark NavigateToPose retry loop.
3. Ctrl+C during an active goal once and verify `goals.csv` contains exactly one `result=interrupted` row for that goal and summary increments `main_interrupted` or `subgoal_interrupted`.
4. Check `metrics.csv`, `trajectory.csv`, `decisions.csv`, `candidates.csv`, `goals.csv`, `plans.csv`, `decision_maps/`, `maps/`, and `summary.json` for consistency.
5. After the recorder passes, run the Nearest baseline closed-loop at least 5 times, target 10, and report per-run plus mean ± std.
6. Continue a full Hospital exploration run and verify `105 -> path-guided recovery` and `208 -> abandon main frontier` remain stable without infinite retries.
7. If the same unreachable region reappears shifted by more than `0.10 m`, decide whether region-aware suppression is needed.
8. Before collecting official benchmark data, ensure the active SLAM/Nav2 runtime matches the intended Hospital v2 configuration and archive effective runtime provenance.

## Important decisions

- **Official Nearest Frontier policy/runner source: `mapex_lab/nf_basic.py`.** This is the canonical algorithm file to maintain going forward.
- `stage2_run.py` is an instrumentation wrapper around that canonical class for Stage-2 data collection; it must not become a divergent copy of the exploration policy.
- Hospital implementation is MapEx frontier/ranking semantics plus a ROS/Nav2 execution adapter; it is not claimed to be the original simulator implementation unchanged.
- Exact frontier `x/y` is the exploration target; planner endpoint is diagnostic/reachability evidence only.
- A path that stops materially short of the frontier must not be used to generate a recovery subgoal.
- Genuine controller/execution failure after a path reaches the exact frontier remains eligible for temporary path-guided subgoal recovery.
- A selected **main frontier** returning `NO_VALID_PATH (208)` is abandoned and session-suppressed within `0.10 m` to prevent infinite retries.
- A failed temporary subgoal, including subgoal error `208`, does not alone prove the main frontier unreachable and therefore does not blacklist the main frontier.
- Official Nearest, MapEx, and proposed method comparisons must use the same Hospital adaptations and effective Nav2/SLAM configuration.
- Pilot/debug launch alternatives do not count as official benchmark runs.

## Latest result

2026-08-28: `nf_basic.py` is accepted as the official/canonical Nearest Frontier runner. Exact-planner testing with `GridBased.tolerance=0.0` produced exact frontier endpoints for reachable goals. Controller failure `105` remains handled by path-guided subgoal recovery, while main-goal `208` now abandons/suppresses that selected frontier and allows exploration to continue instead of looping indefinitely.

2026-08-28: `pilot_002` verified numeric coverage, distance/trajectory logging, decision/plan/map recording, and `105` recovery behavior. Recorder startup is now gated on 3 s of stable NavigateToPose readiness, and finalize now records any active goal explicitly as `interrupted` instead of dropping it.
