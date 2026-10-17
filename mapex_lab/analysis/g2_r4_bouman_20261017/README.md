# G2-R4 — Adaptive Structural Coverage Planning (offline prototype)

**Date:** 2026-10-17. **Status:** maker-developed, synthetic-only tests. No IR1, native MapEx/ROS/robot integration, physical safety certificate, or demonstrated distance/time savings.

This is a **design-preserving R4 overlay**, not a replacement for original G2-R3/R3.1 or Bouman MCTS. Source method: Bouman et al., *Adaptive Coverage Path Planning for Efficient Exploration of Unknown Environments*, arXiv:2302.03164, equations 2, 6, 8; Algorithm 2. G2 context: frozen D02/D18 G2-R3/R3.1 source. Never uses future observations or ground truth as planner input. A simulated hidden GT LiDAR must remain outside this package.

## Preserve the four design commitments

1. **Pathwise structural value, not endpoint-only IG:** `PathValue` operates on R3 nominal and stress visibility bitsets for *all observed-free route scan poses*. Each new leg is credited only for previously unseen unknown cells. Room structural regions are weighted using existing R3 metadata and a failure-aware multiplier; weak predictions **never clear UNKNOWN**. The full chain, including the observed-free exit/return leg, is scored against time. A low-gain transit leg can remain if a later leg has positive whole-chain value. Alternative viewpoint prefixes have no fixed goal count.
2. **Observation-based reliability:** `EvidenceLedger.observe_completed_leg` accepts before/after observed occupancy and prior-time region IDs, and only updates from `UNKNOWN -> FREE/OCCUPIED` changes after a truly executed leg. The reliability factor is a smoothed, non-calibrated measured/predicted ratio, floored so uncertainty does not imply room completion. Failed views require spatially independent positions to count separately.
3. **Event-triggered receding horizon:** `on_scan_fast` is a cheap observed-map/route validity check after each scan. It can HOLD immediately for invalid paths; ordinary scans KEEP a bounded <=0.5 m movement prefix; goal arrival, important map/room changes or sufficient prediction mismatch request a heavy R3-R4 episode. `plan_episode` only evaluates on the heavy trigger, compares old full/prefix chains with fresh R3 proposals, and may KEEP/REPLAN/DEFER without any robot command.
4. **Preserve known-free and MapEx boundaries:** R3's known-free path and clearance grid rules are reused; candidate route requires a reachable known-free rejoin anchor, including its route and cost. `robot_command=None`, `policy_action=None` throughout. Do not use LaMa predicted free to permit travel. DEFER is not ROOM_COMPLETE or global STOP. For physical use, real footprint/continuous trajectories/scan/safety and exit/rejoin must be certified independently.

## Deliberately not copied from Bouman

- No MCTS or full 3D Information Roadmap (2D R3 beam search already searches structural sequences).
- No replacement of visibility rays by adaptive-radius circles at final ranking (occlusion near doors and furniture is crucial). Could benchmark a cheap coarse screening alternative separately.
- No direct reproduction of the Bouman motion/risk controller, PLGRIM, 3D pointcloud, Husky/Spot dynamics. These are not our source assumptions.
- No claim that `conservative` LaMa scenario is calibrated probability, that `PathValue` heuristic parameters are scientifically tuned, or that risk/time saved is measured. Initial coefficients are only synthetic control defaults and need a separate development set.

## Files

- `r4_adaptive.py` — frozen inputs, observation evidence, pathwise scoring, event detector, source-shaped G2-R3 visibility bridge.
- `r4_episode.py` — supervisory entry points, R3/R3.1 proposal reuse, prefix selection and bounded execution.
- `test_r4.py`, `test_episode.py` — maker unit tests incl. no duplicated cells, low-value transit with future value, unexpected walls, stale/replayed inputs, no hardcoded coordinates, no movement commands.
- `shadow_readonly.py` — an **existing-snapshot** diagnostic CLI for frozen G2-R3.1 stage files. No new LiDAR, model forward or counterfactual rollout. Not an end-to-end proof.

## Running the unit suite

```bash
python3 -m unittest discover -s . -p 'test_*.py' -v
```

Dependencies: Python 3.11+, numpy and opencv-python. `r4_adaptive.py` imports R3 only when `from_r3_snapshot` is called; its `mapex_g2_r3` package must be on `PYTHONPATH` for real snapshot evaluation. The tests do not depend on GT or a robot.

## Inputs and outputs

Fast: observed before/after, robot pose, unexecuted verified route, goal-arrival and structural change signals, scan IDs -> SAFE_HOLD / FAST_KEEP / FAST_RECHECK / REVALUE_NOW (advisory).

Heavy: MapEx observed map + hash, current pose, current Room Node semantic, cached LaMa mean/variance (same snapshot), MissionMemory/region report and current/fresh R3 chains + an explicit **known-free** exit/rejoin anchor -> evaluated scenario-specific structural gain, cost/whole-chain choice, selected next movement prefix <=0.5 m, reason codes. If exit is unavailable, DEFER. Full MapEx baseline numerical comparison is NOT verified unless a matched native reference path and horizon are separately supplied.

## Scientific boundary and next work

- No claim of D18 17.2 m savings: D18 is a known failed R3.1 retrospective case. A new policy must decide without GT and be validated on held-out rooms without tuning on D18.
- `safe_event` uses a raster dilation clearance proxy, not a continuous robot-footprint collision certificate.
- `EvidenceLedger` tracks region IDs with current R3 lineage; validate split/merge mapping before deployment.
- Different candidates may lead to different future maps. The R4 choice only scores **current-snapshot hypothetical pathwise sensor masks**, not a causal closed-loop equivalent to native scan outcomes.
- Native MapEx score epochs, path/pose frame, sensor model, cache freshness, G3 handoff, whole-map goal return, and compute profile remain integration work. D02/D18 examples are **diagnostic only**, not held-out science.
