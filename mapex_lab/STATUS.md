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
- Closed-curtain runtime test showed a repeatable tolerance-snapped path: frontier `(19.48,-11.66)`, `/plan` endpoint `(19.88,-11.66)`, 7 poses, endpoint error `0.400 m`; this path is correctly rejected for subgoal recovery.
- Debug Nav2 controller `xy_goal_tolerance` was reduced from `0.4 m` to `0.3 m`; the curtain `/plan` endpoint still remained exactly `0.400 m` from the frontier, so controller tolerance was not the cause of the planner endpoint offset.
- `config/nav2_stock_xy_only.yaml` now explicitly overrides `planner_server.ros__parameters.GridBased.tolerance: 0.0`.
- With planner tolerance `0.0`, later runtime logging showed exact-frontier paths (`endpoint_to_frontier=0.000 m`) for reachable goals. For main frontier `(6.58,0.95)`, Nav2 first produced valid exact paths, then controller failure `105`, path-guided recovery, and after the robot pose changed the same main frontier began returning planner error `208` (`NO_VALID_PATH`) with no usable `/plan`.
- Debug `nf_basic.py` now treats a **main-goal** error `208` as terminal for that selected frontier: it clears the pending main goal, records the rejected position, and suppresses future frontier representatives within `0.10 m` for the rest of that `nf_basic.py` process so the node can continue to another candidate instead of retrying forever. A subgoal error `208` does **not** blacklist the main frontier by itself.
- Cartographer and local-window SLAM alternatives remain debug-only and are not part of the official Hospital v2 benchmark unless protocol identity is changed and all methods are rerun under the same stack.
- `control_tb4.py` and historical `hospital_slam.yaml=0.05` remain available for the separate regression diagnostic; official Hospital v2 runs must restore the intended `0.10 m/cell` runtime profile.

## In progress

- Runtime-test the new `208 -> abandon selected main frontier` rule on the curtain/unreachable-goal case and confirm the node immediately chooses another eligible frontier instead of looping.
- Keep path-guided subgoal recovery for genuine execution failures such as controller error `105` when a valid path reaches the exact frontier.
- Observe whether a `0.10 m` session suppression radius is sufficient to prevent the same unreachable curtain region from reappearing via a slightly shifted representative.
- Continue local-window SLAM and Cartographer diagnostics separately; neither is yet an official benchmark change.

## Next actions

1. Pull the latest repo and restart only `nf_basic.py` if Nav2 with `GridBased.tolerance=0.0` is already running; otherwise fully restart `local.launch.py` first.
2. Reproduce a main frontier that returns `error_code=208`.
3. Verify the log shows `Main frontier abandoned after NO_VALID_PATH (208)` and that the next selected frontier is different.
4. Verify controller/execution failure `105` still uses path-guided subgoal recovery rather than being abandoned immediately.
5. If the same curtain frontier reappears shifted by more than `0.10 m`, decide whether to enlarge the debug suppression radius or replace position suppression with region-aware/revalidation handling.
6. Before official Hospital v2 runs, reconcile this debug rule with the protocol's planner-revalidation policy rather than silently treating the debug blacklist as official benchmark behavior.

## Important decisions

- Hospital implementation is MapEx frontier/ranking semantics plus a ROS/Nav2 execution adapter; it is not claimed to be the original simulator implementation unchanged.
- Exact frontier `x/y` is the exploration target; planner endpoint is diagnostic/reachability evidence only.
- A path that stops materially short of the frontier must not be used to generate a recovery subgoal.
- Genuine controller/execution failure after a path reaches the exact frontier remains eligible for temporary path-guided subgoal recovery.
- Active debug `nf_basic.py` now session-suppresses a selected **main frontier** after `NO_VALID_PATH (208)` to avoid an infinite retry loop. This is a debug execution rule requested for the current diagnosis, not yet a change to the official `hospital_v2` planner-revalidation protocol.
- A failed temporary subgoal, including subgoal error `208`, does not alone prove the main frontier unreachable and therefore does not blacklist the main frontier.
- Official Nearest, MapEx, and proposed method comparisons must use the same Hospital adaptations and effective Nav2/SLAM configuration.
- Debug/pilot runs do not count as official benchmark runs.

## Latest result

2026-08-28 exact-planner diagnostic: with `GridBased.tolerance=0.0`, reachable main goals produced `/plan` endpoints exactly at the frontier (`endpoint_to_frontier=0.000 m`). Main frontier `(6.58,0.95)` initially had valid exact paths but failed execution with controller error `105`; path-guided recovery was attempted and one temporary subgoal succeeded. After that pose change, retries of the same main frontier returned `error_code=208` with no usable `/plan`, causing the old code to loop forever. Active debug `nf_basic.py` is now changed so a main-goal `208` abandons that selected frontier and suppresses positions within `0.10 m` for the current process, allowing exploration to move on to another candidate.
