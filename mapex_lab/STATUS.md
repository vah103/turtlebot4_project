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
- Cartographer and local-window SLAM alternatives remain debug-only and are not part of the official Hospital v2 benchmark unless protocol identity is changed and all methods are rerun under the same stack.
- `control_tb4.py` and historical `hospital_slam.yaml=0.05` remain available only for separate regression diagnostics; they are not the official Nearest runner.

## In progress

- Continue runtime validation of the official `nf_basic.py` over longer Hospital exploration runs.
- Keep path-guided subgoal recovery for genuine execution failures such as controller error `105` when a valid path reaches the exact frontier.
- Observe whether a `0.10 m` session suppression radius is sufficient to prevent an unreachable region from reappearing via a slightly shifted representative.
- Continue local-window SLAM and Cartographer diagnostics separately; neither is yet an official benchmark change.

## Next actions

1. Use `mapex_lab/nf_basic.py` as the Nearest Frontier execution entry point for subsequent runs.
2. Continue a full Hospital exploration run and verify `105 -> path-guided recovery` and `208 -> abandon main frontier` remain stable without infinite retries.
3. If the same unreachable region reappears shifted by more than `0.10 m`, decide whether region-aware suppression is needed.
4. Before collecting official benchmark data, ensure the active SLAM/Nav2 runtime matches the intended Hospital v2 configuration and archive effective runtime provenance.

## Important decisions

- **Official Nearest Frontier runner: `mapex_lab/nf_basic.py`.** This is the canonical file to run and maintain going forward.
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
