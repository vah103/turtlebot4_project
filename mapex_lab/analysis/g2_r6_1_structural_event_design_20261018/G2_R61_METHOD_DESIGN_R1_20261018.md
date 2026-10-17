# G2-R6.1 — Structural Event-Driven Watchman
**METHOD DESIGN R1 · 2026-10-18 · NOT IMPLEMENTED OR NATIVE VERIFIED**

Parent: G2-R6 R1, draft PR #44. Dedicated design branch. Objective: exploit robot travel time to keep a good room-coverage route fresh, and detect structural surprises before executing an obsolete chain of viewpoints.

## 1. Scientific and safety invariants

- Only causal LiDAR observed occupancy, same-epoch LaMa mean/variance, Room Node V4-R4.1 outputs and native planner state. Do not use GT as policy input or observations from a future tick.
- Movement is permitted only on currently observed FREE, robot-connected, Room-Node-approved and native-inflated A*-validated cells. Predicted FREE is never a physical path.
- Local exploration ends only according to the same eligible local WFD / accepted ROOM mode gate as NR-RN-01, not because predicted coverage is high.
- The robot may abandon any future waypoint, but all replans start at its actual current location.
- Protect native 2D LiDAR and motion timing. No Gazebo; no training or per-room handpicked D02 goals.
- All tentative results are explicitly marked model projections; the only scientific measurement of coverage is the observed map.

## 2. Audit of G2-R6 R1 and why R6.1 is needed

The actual R1 source has these gaps:

- The scan-time native adapter feeds observed occupancy and its cached reachability, but **does not feed newly inferred Room Node semantics**. This means ordinary UNKNOWN-to-FREE observations cannot by themselves cause Room Node reclassification.
- The current R1 scene/raycast cache is tied to the LaMa initialization, not to each new observed occlusion map. Cached projected visibility can be stale when a new wall or opening appears.
- The native cached-next-goal path ACKs the visited goal and may dispatch the next stale goal without a mandatory post-arrival structural checkpoint.
- Old logic largely prunes from the tail and protects the first goal; it is not prepared to discard multiple unnecessary intermediate destinations.
- Scalar area-change triggers are insufficient: a small observed doorway may matter more than a large expected FREE reveal.

Example: G2-R6 R1 predicted five D02 goals, first [189,120]. We **do not know** how actual LiDAR at that point will change the structure until a fresh closed-loop run. R6.1 must detect that change without borrowing future snapshots from another trajectory.

## 3. Epoch and evidence model

Immutable ObservationEpoch:
- observation_seq, native_tick, scan_id, robot_core_rc, observed_map, observed_hash, native_knownfree_scope and acquisition timestamp.

LaMaEpoch:
- parent_observation_seq, mean, variance, model identities, prediction_hash. Predicted mean is a heuristic occupancy proxy, not calibrated certainty.

RoomEpoch:
- parent_observation_seq, parent_lama_epoch, semantic mask, robot-connected accepted ROOM mask, room ID, frontier and geometry report.

PlanEpoch:
- parent observation/LaMa/Room ids, route generation, source robot pose, committed active-goal id, pending goal ids and known-FREE route certification.

Every scan enters the native mapper immediately. Background optimization may sample less frequently, but must aggregate *all* actual changes since its last accepted ObservationEpoch. Every committed command must be recertified on the **latest** native occupancy and pose. An asynchronous job based on an older observation may not overwrite a newer accepted decision.

## 4. StructuralEventMonitor: inexpensive checks using real LiDAR

### 4.1 New information and contradiction

At each scan compute:
- new_free = previous UNKNOWN now observed FREE.
- new_wall = previous UNKNOWN now observed WALL.
- blocked_known_free = previously observed FREE now observed WALL; this is an immediate navigation safety signal.
- Unexpected-free mass: sum of old predicted occupancy mean in new_free cells.
- Unexpected-wall mass: sum of 1 minus old predicted occupancy mean in new_wall cells.
- Spatial concentration and affected structural region. High variation reduces prediction confidence, but never suppresses real observed obstacles.

A thousand newly observed FREE cells consistent with LaMa do not automatically require re-running LaMa. A narrow opening connecting sectors, even with fewer changed cells, may be far more significant.

### 4.2 Geometry and topology

Use observed-only geometric analysis around the current room boundary, updated FREE patches and robot path, without re-running the LaMa ensemble:

- Match frontier clusters from old and new observed maps by overlap; detect a newly accessible frontier branch or the disappearance of a major frontier.
- Label newly observed FREE patches and check whether they expand the robot-reachable FREE region into previously closed-off parts of the accepted room.
- Detect **portal candidates**: observed FREE gaps between sufficiently supported observed wall boundaries or new safe connections between previously separated local sectors. UNKNOWN boundaries yield hypotheses, not verified doors.
- Track observed bottlenecks and occlusion changes near pending goals, including walls obstructing previous expected sightlines.
- Recheck all active path cells and the next native-knownfree path under the newest native inflation mask.

The lightweight monitor can mark a room-boundary violation as a structural candidate, but **must not pretend it has recomputed Room Node V4-R4.1**. That official result requires a new coherent LaMa/Room epoch.

### 4.3 Visibility and pending goals

For the *already traversed* route prefix, compare actual formerly UNKNOWN cells that became observed with the old prediction. Do not count pixels predicted visible from future goals as actually scanned.

For each future goal calculate:
- expected incremental visibility on UNKNOWN that remains unobserved in the current map,
- extra path distance using current native-knownfree routes,
- marginal value lost if the goal is removed, after all other proposed sightings and pathwise sensing are considered,
- whether new observed walls or openings invalidate the old raycache.

Invalidate the affected visibility cache when observed occlusion changes; recompute locally with the latest observed occupancy. A new observed branch inside the accepted room may justify a **light Watchman replan without LaMa**.

## 5. Event priorities and actions

| Code | Trigger | Mandatory action |
|---|---|---|
| S0 SAFETY_HOLD | Active segment blocked, native A* invalid, loss of knownfree safety | Stop using that segment immediately; no async wait; native safe stop or verified fallback |
| S1 KEEP | Routine observed progress, old route remains productive and safe | Keep current goal; cheap remaining-gain updates; drop proven redundant tail goals |
| S2 LIGHT_REPLAN | Important new FREE branch *inside accepted ROOM*, meaningful new reachable frontier, or multiple obsolete goals | Recompute unexecuted Watchman from current pose using current observed map and old LaMa only as a tentative prior; check native path before commit |
| S3 STRUCTURAL_REBUILD | Verified portal, invalid/changed room hypothesis, concentrated strong model contradiction or a branch outside old room boundary | Do not commit stale next goal; recompute LaMa from latest observation, then Room Node, room-scoped WFD and new Watchman route |
| S4 EXIT_LOCAL | Updated accepted room is absent or reachable local WFD is empty | Hand control to original GLOBAL MapEx; no invented completion based on predicted visibility |

Priority is S0 above everything. S4 is determined from a coherent accepted Room Node/WFD gate, not inferred from missing predicted UNKNOWN. Non-dangerous S2/S3 events may require a second confirming scan and cooldown to prevent false alarms; no cooldown can delay S0.

**Initial, unvalidated thresholds** to test, not tune to D02:
- S2 when a newly connected FREE patch has at least roughly 40 cells **and** changes the frontier/branch graph; or when the marginal value of the future route changes by roughly 20%; or when an important new reachable frontier appears.
- S3 when reliable geometric evidence demonstrates a portal / room boundary mismatch; or when a concentrated contradiction is seen, such as roughly 32 new wall cells in a region predicted strongly FREE; or a fresh structural scope decision reports a material change.
- The old 5% changed room-mask statistic is applicable **only after a new Room Node result**. Never use the identical cached semantic map to declare structure unchanged.

Uncertainty and per-region evidence must accompany every trigger, including the reason a candidate doorway was not confirmed.

## 6. State machine and execution contract

States: GLOBAL, LOCAL_COMMITTED, LIGHT_PENDING, STRUCT_PENDING, SAFE_HOLD, EXIT_LOCAL.

For every native scan:
    integrate LiDAR in native mapper
    snapshot newest observed map and native-knownfree mask
    update structural evidence and actual observation progress
    if unsafe native path: safety hold immediately
    else if S3: enqueue a coherent LaMa + Room Node refresh
    else if S2: enqueue light Watchman replan
    else: keep safe current goal and refresh goal utility
    never let background optimization directly command native motion

During travel:
    continue currently certified safe segment while pending work runs
    coalesce superseded requests but preserve accumulated scan differences
    do not wait for heavy LaMa just because some predictable FREE was observed
    if current goal loses value, redirect only after native validates the alternative

**Mandatory goal-arrival checkpoint**:
    consume latest LiDAR scan and structural evidence
    confirm the robot actually reached the current goal
    check current native path safety and eligible local WFD
    if safety event / unresolved material structural change: do not ACK old future goal
    if S2: commit the newly computed, validated remaining route
    if S1: re-evaluate goal value and native A*, then ACK and continue
    if S4: exit LOCAL and use MapEx GLOBAL

A plan proposal contains source observation, mean/variance, room and route epoch IDs, plus the robot pose used for calculation. On completion it must pass stale-result rejection, native-inflated A*, accepted ROOM scope and idempotent goal-ACK checks. Stale background plans may never overwrite newer safe plans.

## 7. Calculation budgets and refresh policy

ALWAYS CHEAP: occupancy deltas, actual observation progress, connected reachable FREE/frontier change, route/goal safety and expected remaining viewpoint value.

S2 LIGHT: rebuild affected visibility using current observed blockers, recalculate native-knownfree paths, run bounded greedy coverage and limited local goal swaps/deletions. No new LaMa.

S3 HEAVY: capture *one coherent newest observation epoch*, run LaMa on it, pass that same epoch's map/mean/variance to V4-R4.1, get current semantic, local WFD and safe scope, rebuild the route. Do not pass old LaMa as a fictitiously refreshed Room Node result.

S4 EXIT: unchanged MapEx global frontier selection.

**Performance goals, not results:** per-scan light monitor usually below 0.1 s on target CPU; light reroute below 1 s; heavy refresh only when causally justified. Record queue delay, LaMa time, Room Node time, G2 time and actual motion time separately. Prioritize native LiDAR/control threads and bound optimization work.

## 8. Code changes required (design only)

New structural module r61_structural_events.py:
- observe_delta(previous_obs, current_obs, old_prediction, history) returns new_free, new_wall, contradiction and updated evidence.
- build_observed_structural_sketch(...) returns frontier cluster matches, accessible patch graph, supported portals and occlusions.
- classify_event(...) returns code S0-S4, reason, supporting cells/regions and uncertainty.

New plan manager r61_plan_manager.py:
- on_scan(ObservationEpoch) tracks cumulative evidence and schedules optional jobs.
- checkpoint_goal_arrival(...) returns ACK_NEXT, WAIT_REPLAN, SAFE_HOLD or EXIT_LOCAL.
- propose_light_route(...) and propose_structural_route(...) start from actual current pose.
- try_commit(...) validates epoch lineage, latest FREE/A*, approved ROOM and goal ACK sequence.

Patch core G2-R6:
- visibility cache keyed by map/prediction epoch and affected occlusions;
- no claim of observed coverage for simulated rays;
- delete redundant intermediate goals (not only the last one) when valid;
- re-evaluate all unvisited goals against actual new LiDAR progress.

Patch R6 native adapter and sidecar:
- scan updates transmit causal observation/plan versions and evidence;
- background jobs cannot block motion or accumulate unbounded queue;
- cached_next_goal() cannot bypass the mandatory viewpoint-arrival checkpoint;
- native performs final FREE/A* validation on every new commanded segment;
- retain the original local WFD exit rule.

All implementation occurs on a later reviewed branch; R6 R1 is frozen.

## 9. QA acceptance matrix

T01 expected FREE revealed: KEEP or cheap refresh, not S3.
T02 a thousand predictably FREE cells: no automatic expensive re-inference by cell count alone.
T03 a small, credibly verified observed portal connecting sectors: S3 even for small change.
T04 significant new FREE branch inside accepted ROOM: S2, route starts at current robot pose.
T05 fresh WALL blocks next native step: immediate S0 regardless of cooldown.
T06 observed wall changes occlusion without blocking motion: invalidate and recalculate affected visibility caches.
T07 a future goal becomes redundant before robot reaches it: safe skip at next valid commitment point.
T08 mandatory viewpoint arrival check: no blind automatic continuation to second cached goal after change.
T09 two async plans finish out of order: stale candidate discarded.
T10 no GT or future map files provided: all tests still pass; input hash and sequence strictly validated.
T11 updated Room Node finds corridor/no supported room: hand back to GLOBAL under original mode gate.
T12 replayed scan or duplicated ACK: idempotent, no repeat-goal or state regression.
T13 LOCAL WFD remains, no positive safe Watchman: safe fail-closed or explicitly reviewed fallback; not false exit/completion.
T14 noisy portal hypothesis: temporal evidence prevents thrashing; S0 cannot be delayed.
T15 old LaMa used as tentative light planner prior, not falsely named a new official Room Node epoch.
T16 slow optimizer: native scan/control remains responsive, cumulative deltas preserved.

### Native causal D02 trial
- Freeze the exact observed + LaMa + Room Node inputs at D02.
- Run G2-R6.1 native **closed loop**. Only actual scans along its new route may update understanding at viewpoint 1; do not splice in D03/D04 data from a different run.
- Record each newly observed patch, portal candidate, event, last/current plan overlay, canceled future goal and native safe route evidence.
- When a local room update occurs, show previous Room Node hypothesis and new Room Node result explicitly.
- Compare distance and elapsed time to the **same** remaining UNKNOWN threshold in a frozen D02 ROI versus NR-RN-01 (24.861 m to <=270 unknown), only if that condition is truly achieved. Compare later rooms and Hospital without tuning per room.

### Visualization
Four-pane per significant event: current observed LiDAR + robot/frontiers; LaMa/Room semantics; former route in grey vs revised route in orange (canceled goals crossed out); **actual newly observed cells** distinct from **projected future visible cells**. Every frame includes scan seq, pose, event reason and map/plan epoch ids.

## 10. Deliverable status
This file is an exact method/interface/QA design. No R6.1 code, synthetic results, new LaMa run, native run or science advantage is claimed. Next phase is implementation and 16 tests on a separate branch, followed by independent method/integration QA before COM1 native execution.
