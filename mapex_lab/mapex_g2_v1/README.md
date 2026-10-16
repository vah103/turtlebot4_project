# G2-V1 — Adaptive Room Viewpoint Sequence Planner

Standalone **shadow planner**, not yet connected to MapEx navigation or G3. Source code is designed for the exact Run04-style arrays, with no hidden D02 coordinate assumptions.

## Inputs (current snapshot only)

- `observed`: 2D occupancy, codes `0=observed free`, `1=unknown`, `2=observed occupied`.
- `mean`, `variance`: current LaMa ensemble occupancy mean and variance; optional `ensemble` (`K,H,W`) if individual predictions available. Mean/variance **must have been recalculated** after new scans, not copied from the previous decision.
- `semantic`: current G1 V4-R4.1 room/corridor semantic. Active room is its `semantic==1` connected component containing the robot, excluding observed walls.
- `robot_rc`: current robot position in the same array's **row,col** coordinates. For MX072 native padded coords subtract `(500,500)` to produce the 203×263 core indices.
- Optional current `frontier_rc`, and `G2Config` sensor, navigation, search and cost parameters.

## Exact pipeline

1. Find robot-connected Room Node. Count **all unknown** in the room, including unknown LaMa predicts may be walls.
2. Generate vantage candidates within observed-free, clearance-checked, robot-reachable cells; optional frontier-adjacent free candidates. **No predicted free is traversable until observed.**
3. Evaluate nominal and pessimistic uncertain ray visibility (optional three-member ensemble), combine observations along each safe shortest-grid route with a max-union to avoid double-counting across scans.
4. Beam-search conditional **whole viewpoint sequences**, with variable goal count and no fixed 1/2/3 target. Optimize `J = travel_time + stop_time + λ remaining_weighted_unknown + ρ predictive_gap`. It evaluates total cost at each chain length. `λ`, `ρ`, speed and stop-time are explicit configurable assumptions, **not scientifically calibrated**.
5. Return `goal_sequence`, `route_segments`, predicted marginal/cumulative gain, cost, `CONTINUE`/`DEFER`/`COMPLETE_OBSERVED`, and provenance SHA hashes.
6. At every meaningful scan change, call `assess_replan`; when it requests `RERUN_LAMA_AND_ROOM_NODE_THEN_G2`, the **orchestrator** must actually recompute both LaMa and G1 for the updated observed map and current pose. Then call G2 again for a **new full remaining sequence**. This planner does not invent new LaMa/Room Node outputs.

`COMPLETE_OBSERVED` is returned only when **zero unknown cells remain in the accepted room mask**. `DEFER` means no positive currently executable safe sequence; it is **not** a completion claim. This policy is deliberately conservative pending G3's explicit exit/stop/defer logic.

### Quick run / D02 adapter (when explicitly asked to run D02)

```bash
PYTHONPATH=/path/to/parent python -m mapex_g2_v1.cli \
  --snapshot /path/to/decision_000002_input.npz \
  --semantic /path/to/decision_000002_semantic.npz \
  --robot-rc 147,111 --decision-id RUN04_D02 --output /tmp/g2_d02_v1.json \
  --visualization-png /tmp/g2_d02_v1.png --coverage-npz /tmp/g2_d02_v1_coverage.npz
```

The CLI opens only these two current-decision NPZs; never reads GT, D03+, retrospectively generated VP1 maps, or answers. It does **not** itself rerun LaMa/G1 or move the robot.

### Synthetic technical tests

```bash
PYTHONPATH=/path/to/parent python -m unittest discover -s /path/to/mapex_g2_v1/tests -v
```

### Limits and QA boundary

- Probabilistic rays use an independent-cell attenuation approximation, max across scans, and a reduced number of planning beams (default 128 vs nominal MapEx LiDAR 2,500). It is **not** source-native RangeLibc and does not prove numerical parity.
- A free-grid inflated path is a conservative feasibility *proxy*, not a certified robot footprint, safe velocity, local controller, or collision test. Candidate cap and beam width are search budgets, not scientific guarantees of optimality.
- A planned distant goal **in predicted-only free space is not executable**. Future conditional candidates must wait for new observed-free evidence and be regenerated; the current version does not output a speculative through-unknown route.
- The model can be overconfident. Variance and optionally individual model predictions affect projected visibility, while room status never comes from predictions alone.
- No training, GT calibration, synthetic evidence-to-real claim, or verified time savings are included.
- D02 must be a **prospective test** of the frozen code/config, not a place to hand-tune thresholds until it recreates a desired `[VP1]` answer.
