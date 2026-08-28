# STATUS

## Current stage

**Stage 2 — MapEx Nearest baseline (Hospital v2 validation/debug before official runs)**

## Done

- Research workspace is `mapex_lab/`; official MapEx source is pinned at `53636bd1c79153acc3c74a532837d78c926bae5e`.
- Hospital v2 target runtime SLAM/policy grid is `0.10 m/cell`; fixed evaluation canvas remains `hospital_canvas_v1` at `0.05 m/cell` with frozen ROI `hospital_connected_free_v1` denominator `215435`.
- Nearest frontier semantics are fixed: free `==0`, unknown `<0`, 8-neighbour frontier, 8-connected regions, keep region `>10`, representative nearest arithmetic mean, Euclidean ranking.
- Hospital keeps exact frontier `x/y` as the execution target and ignores terminal yaw.
- Debug `nf_basic.py` currently uses the restored path-guided recovery: a failed main goal may use a temporary subgoal on the latest main-goal `/plan`, then retry the same frontier.
- Debug `nf_basic.py` also uses the event-driven debug BT and conservative completion verification (5 distinct no-frontier map sweeps, >=2 s apart, >=10 s idle, >=20 s node age).
- Added main-plan diagnostics to `nf_basic.py`: a main `/plan` is usable for subgoal recovery only if it has at least 2 poses and its final pose is within `0.10 m` of the exact frontier.
- Closed-curtain runtime test showed a repeatable tolerance-snapped path: frontier `(19.48,-11.66)`, `/plan` endpoint `(19.88,-11.66)`, 7 poses, endpoint error `0.400 m`; this path is now correctly rejected for subgoal recovery, but Nav2 still reports main goal success and the same frontier is selected again.
- Debug Nav2 controller `xy_goal_tolerance` was reduced from `0.4 m` to `0.3 m`; the curtain `/plan` endpoint still remained exactly `0.400 m` from the frontier, so controller tolerance was not the cause of the planner endpoint offset.
- `config/nav2_stock_xy_only.yaml` now explicitly overrides `planner_server.ros__parameters.GridBased.tolerance: 0.0` so the next debug run can test whether the global planner returns a true exact-frontier path or `NO_VALID_PATH` instead of tolerance-snapping to a nearby endpoint.
- Cartographer and local-window SLAM alternatives remain debug-only and are not part of the official Hospital v2 benchmark unless protocol identity is changed and all methods are rerun under the same stack.
- `control_tb4.py` and historical `hospital_slam.yaml=0.05` remain available for the separate regression diagnostic; official Hospital v2 runs must restore the intended `0.10 m/cell` runtime profile.

## In progress

- Re-run the closed-curtain case with `GridBased.tolerance=0.0` active in the launched Nav2 process.
- Determine whether the curtain frontier becomes a true planner `NO_VALID_PATH` case or whether another Nav2 tolerance/goal-checking mechanism is still producing the 0.4 m endpoint.
- Keep path-guided subgoal recovery for genuine execution failures where a valid path reaches the exact frontier.
- Continue local-window SLAM and Cartographer diagnostics separately; neither is yet an official benchmark change.

## Next actions

1. Pull the latest repo and fully restart `local.launch.py` so the new planner parameter is loaded.
2. Confirm runtime value with `ros2 param get /planner_server GridBased.tolerance`; expected value is `0.0`.
3. Restart `nf_basic.py` and reproduce the curtain frontier.
4. Inspect whether `/plan` reaches the exact frontier (`endpoint_to_frontier <= 0.10 m`) or Nav2 returns planner failure/no path.
5. If the curtain becomes `NO_VALID_PATH`, implement separate handling: planner no-path -> skip/defer frontier; planner-valid execution failure -> path-guided subgoal recovery.
6. If the planner still ends 0.4 m short despite runtime tolerance `0.0`, inspect the effective planner plugin/runtime parameters and BT path generation before changing frontier logic again.

## Important decisions

- Hospital implementation is MapEx frontier/ranking semantics plus a ROS/Nav2 execution adapter; it is not claimed to be the original simulator implementation unchanged.
- Exact frontier `x/y` is the exploration target; planner endpoint is diagnostic/reachability evidence only.
- A path that stops materially short of the frontier must not be used to generate a recovery subgoal.
- Genuine controller/execution failure after a path reaches the exact frontier remains eligible for temporary path-guided subgoal recovery.
- Planner no-path evidence should not automatically become a permanent blacklist in an online changing map; final suppression/revalidation policy is still being designed.
- Official Nearest, MapEx, and proposed method comparisons must use the same Hospital adaptations and effective Nav2/SLAM configuration.
- Debug/pilot runs do not count as official benchmark runs.

## Latest result

2026-08-28 curtain diagnostic: `nf_basic.py` repeatedly selected frontier `(19.48,-11.66)` at about `0.75 m`; Nav2 published a 7-pose path ending at `(19.88,-11.66)`, exactly `0.400 m` short of the frontier, and still reported `Goal reached`. The new endpoint check correctly labels that path unusable for subgoal recovery. Reducing controller `xy_goal_tolerance` to `0.3 m` did not change the 0.4 m path endpoint. The next test explicitly forces `GridBased.tolerance=0.0` in the debug Nav2 override.
