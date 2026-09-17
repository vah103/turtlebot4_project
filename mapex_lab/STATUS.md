# mapex_lab status

_Last synchronized with `main`: 2026-09-17._

This file records the current code/research state on `main`.

## Current focus

The active thesis direction is **Direction 2: early stopping for MapEx**.

- **Way1: absolute global uncertainty threshold** is closed.
- **Way2: frontier-utility-based stopping** is open.
- Way2 is still in the **algorithm-design/offline-analysis** stage; online MapEx has not yet been changed to stop early.
- Offline replay, the gain-definition audit, and the historical travel-cost audit are complete.
- For historical Way2 replay, `G` is frozen as raw `information_gain` and `C` is frozen as Euclidean `distance_m`.
- The current next technical step is `lambda` analysis.

Research question:

> **When is the best remaining reachable frontier no longer worth its expected information gain relative to the physical cost of reaching it?**

## Experiment runner

Normal entry point:

```bash
./mapex_lab/run
```

Current interactive flow:

1. choose `NF` or `MapEx`;
2. choose `New Room` or `Hospital`;
3. choose whether to save a record;
4. enter a run ID when recording.

Current map/profile mapping:

- New Room -> `world:=new_room`, environment `new_room`, runtime profile `slam`;
- Hospital -> `world:=hospital`, environment `hospital`, runtime profile `hospital_slam`.

The runner launches `launch/slam.launch.py`, waits for `/map` and `/navigate_to_pose`, then dispatches the selected exploration method. Existing run directories are not overwritten.

## Core MapEx state

- `scripts/mapex.py` — MapEx exploration/frontier ranking.
- `scripts/mapex_run.py` — recording and evaluation wrapper.
- `scripts/mapex_lama_bridge.py` / `mapex_lama_worker.py` — LaMa inference path.
- Online MapEx uses the **3-model ensemble**.
- All-training prediction is an offline evaluation path, not a fourth online ensemble member.

Current MapEx frontier-scoring semantics:

```text
observed map
-> 3 LaMa predictions
-> ensemble mean + variance
-> frontier extraction
-> predicted visibility
-> information gain over predicted-visible + currently-unknown cells
-> Euclidean distance
-> score = information_gain / distance
-> choose max-score frontier
```

The recorded candidate/decision data exposes the quantities needed for Way2, including `information_gain`, `distance_m`, `score`, `visible_unknown_cells`, selected frontier state and candidate eligibility/suppression information.

---

# Way1 — CLOSED

## Definition

Way1 used:

```text
unknown_variance_p95
```

Stopping rule:

```text
P95 <= threshold for K consecutive distinct decision states
```

Primary frozen rule:

```text
P95 <= 0.23, K=1
```

## Development/tuning — New Room mpx_001...mpx_010

Fixed-rule results:

| Rule | Time saved | Distance saved | Mean IoU loss | Worst IoU loss |
|---|---:|---:|---:|---:|
| `P95 <= 0.23, K=1` | 23.64% | 24.03% | -0.00275 | 0.00938 |
| `P95 <= 0.25, K=2` | 22.23% | 22.67% | -0.00285 | 0.00977 |
| `P95 <= 0.20, K=1` | 17.28% | 16.90% | -0.00030 | 0.00540 |

LOOCV showed that repeatedly selecting the maximum-savings rule on only nine training runs could become too aggressive. Therefore `0.23 x 1` was frozen before prospective validation.

## Prospective New Room validation — mpx_011...mpx_015

| Run | Time saved | Distance saved | IoU loss vs analyzer final reference |
|---|---:|---:|---:|
| `mpx_011` | 18.69% | 20.72% | +0.000053 |
| `mpx_012` | 27.62% | 25.39% | -0.003336 |
| `mpx_013` | 21.67% | 23.28% | -0.001435 |
| `mpx_014` | 24.77% | 27.81% | -0.002251 |
| `mpx_015` | 29.10% | 29.58% | -0.003084 |

Aggregate:

- trigger rate: **5/5**;
- mean time saved: **24.37%**;
- mean distance saved: **25.36%**;
- mean IoU loss: **-0.00201**;
- worst IoU loss: **+0.000053**;
- IoU loss `> 0.01`: **0/5**;
- TU loss: **0 on all 5 runs**.

Conclusion: Way1 had strong prospective validation **within New Room**.

## Hospital cross-environment diagnostic — hpx_001...hpx_002

Frozen `P95 <= 0.23, K=1` was tested without retuning.

### hpx_001

```text
P95 min    = 0.318309
P95 median = 0.333203
P95 last   = 0.333257
trigger     = false
```

### hpx_002

```text
P95 min    = 0.331268
P95 median = 0.333242
P95 last   = 0.333297
trigger     = false
```

### Way1 conclusion

- Hospital trigger rate: **0/2**.
- Hospital P95 remains close to `1/3` through most of both runs.
- With three prediction models and sample variance (`ddof=1`), binary-like disagreement such as `[0,0,1]` or `[0,1,1]` gives variance `1/3`, so a high global quantile can remain saturated.
- Retuning the threshold specifically for Hospital would undermine the cross-environment objective.

**Way1 is closed as the primary research path.**

No `hpx_003` is required merely to reconfirm the same Way1 failure mode.

---

# Way2 — OPEN

## Design objective

Way2 asks:

> **Does any remaining reachable frontier still have enough expected information value to justify the cost of physically visiting it?**

The stopping condition is derived from a cost-benefit objective first. Existing runs are used as development/diagnostic data to inspect behavior and failure modes, not as independent validation.

## Core one-step utility formulation

For reachable/selectable frontier `f` at decision `t`:

```text
V_t(f) = G_t(f) - lambda * C_t(f)
V*_t   = max_f V_t(f)
```

Core stopping rule:

```text
if V*_t <= 0:
    stop exploration
else:
    continue exploration
```

Equivalent when `C_t(f) > 0`:

```text
max_f (G_t(f) / C_t(f)) <= lambda
```

This is a **one-step utility-based stopping rule**, not a full-horizon optimal-control solution.

## Offline replay — COMPLETED

Development/diagnostic runs:

- New Room: `mpx_001...mpx_015`;
- Hospital: `hpx_001...hpx_002`.

Replay baseline:

```text
G = information_gain
C = distance_m
score = G / C
candidate set = selectable frontiers only
```

The replay matches current MapEx behavior:

```text
score_mismatch     = 0
selection_mismatch = 0
```

Therefore the offline analyzer reproduces the recorded MapEx ranking semantics correctly.

Implemented analysis scripts include:

- `scripts/analyze_way2_utility.py`;
- `scripts/audit_way2_gain.py`;
- `scripts/compare_way2_gain_definitions.py`;
- `scripts/audit_way2_cost.py`.

## Information gain G — FROZEN FOR CURRENT WAY2

Two definitions were compared:

```text
G_raw  = information_gain
G_norm = information_gain / visible_unknown_cells
```

Main observations:

- `G_norm` reduces startup/spawn-scale effects, but changes the best frontier substantially: candidate agreement with raw MapEx ranking is only about **30%** of decisions.
- Therefore dividing by `visible_unknown_cells` is not merely a harmless scale normalization; it changes the frontier-value semantics from total expected information to mean uncertainty per visible cell.
- With raw gain, post-early `G/C` scale is already reasonably similar across environments:
  - New Room median post-early: **3.05**;
  - Hospital median post-early: **2.23**.
- Late-run scale is also close:
  - New Room median late: **0.472**;
  - Hospital median late: **0.367**.
- In both environments, raw `G/C` decreases strongly toward the end of exploration, which is the behavior needed for a stopping signal.

Hospital startup outlier:

```text
first-decision G/C ≈ 348
```

Audit shows this is mainly caused by a large initially visible unknown region plus a short frontier distance. Its per-cell uncertainty is not anomalously high.

### Gain conclusion

**Keep the original MapEx gain:**

```text
G_t(f) = information_gain(f)
```

Do **not** divide by `visible_unknown_cells` for Way2.

Do **not** use first-decision/startup maxima as the basis for choosing `lambda`, because they are strongly spawn/view dependent.

## Travel cost C — FROZEN FOR HISTORICAL REPLAY

Historical replay uses:

```text
C_t(f) = recorded Euclidean distance_m
```

Cost audit over `mpx_001...mpx_015` and `hpx_001...hpx_002` found:

| Environment | Euclidean coverage of selectable candidates | Planner coverage of selected decisions | Planner all-candidate coverage upper bound | Median run-level planner/Euclidean ratio |
|---|---:|---:|---:|---:|
| New Room | 100.0% | 98.1% | 5.5% | 1.413 |
| Hospital | 100.0% | 89.6% | 6.7% | 1.479 |

Interpretation:

- Euclidean `distance_m` is available for every selectable candidate at decision time.
- `plans.csv` stores Nav2 path length only after a frontier has been selected as an active navigation goal; it does not provide planner cost for every candidate in the same decision.
- A selected decision can contain multiple plan updates while the robot moves, so the audit compares Euclidean distance against the **first usable plan** for that decision.
- On selected goals, planner paths are typically longer than straight-line distance, as expected; however this selected-goal subset cannot be used as an unbiased all-candidate Way2 cost replacement.

### Cost conclusion

For the existing 17 development runs, freeze:

```text
C_t(f) = distance_m
```

Planner path length is **not** used as historical `C` because its all-candidate coverage is only about 5.5–6.7%, which would make the replay incomplete and selection-biased.

A future planner-cost Way2 variant would need to compute/store `ComputePathToPose` for every selectable frontier at each decision before ranking/stopping. That would be a separate online design change and should be evaluated separately from the current historical replay.

## Reachability

Utility is evaluated only on genuinely selectable/reachable candidates.

Planner/execution-suppressed candidates must not keep `V*_t` artificially positive and prevent stopping.

## Meaning of lambda — NOT YET FROZEN

```text
lambda = required information gain per unit travel cost
```

`lambda` must not be chosen from first-decision maxima or by cherry-picking the value with the prettiest savings.

The next offline analysis should inspect the empirical decision sequence:

```text
R_t = max_f (G_t(f) / C_t(f))
```

with frozen raw `G`, frozen Euclidean `C`, and a `K=2` distinct-state debounce, then study threshold crossings, rebounds and premature-stop risk across both environments before freezing any value/range.

## Noise guard

Current candidate guard:

```text
require V*_t <= 0 for K=2 distinct decision states before stopping
```

`K=2` is treated as a debounce guard, not a parameter to sweep for maximum savings.

## Way2 data roles

Because the current 17 runs have already been used to design/audit Way2:

- `mpx_001...mpx_015` = development/diagnostic only;
- `hpx_001...hpx_002` = development/diagnostic only;
- they must **not** later be reported as independent Way2 validation.

Independent validation requires new runs collected only after `G`, `C`, `lambda` and the noise guard are frozen.

## Way2 implementation roadmap

### A. Audit current code/data — COMPLETED

Verified `information_gain`, Euclidean distance, `score = IG/distance`, selectable/suppressed candidates and available recorder fields.

### B. Build offline Way2 replay — COMPLETED

Replay matches current MapEx score and selected frontier on all evaluable decisions.

### C. Freeze gain and cost definitions — COMPLETED FOR HISTORICAL REPLAY

- `G`: **frozen as raw `information_gain`**.
- `G / visible_unknown_cells`: rejected because it changes the best frontier in about 70% of decisions.
- `C`: **frozen as Euclidean `distance_m` for the current 17-run replay** because it has 100% selectable-candidate coverage.
- Planner path length remains a possible future variant only if per-candidate path cost is recorded online.

### D. Define lambda from the objective — NEXT

With `G` and `C` now fixed, analyze `R_t = max(G/C)` distributions/crossings and select a defensible declared `lambda` or sensitivity range.

### E. Backtest development data

Evaluate premature-stop timing, threshold rebound, time/distance savings and map-quality proxies without treating old data as independent validation.

### F. Freeze Way2

Freeze:

- `G_t(f)`;
- `C_t(f)`;
- `lambda`;
- persistence/noise guard.

### G. Integrate online

Only after the offline formulation is stable, add the utility stop condition to online MapEx.

### H. Independent validation

Collect new New Room and Hospital runs after freeze, using the same formulation and same `lambda` without environment-specific retuning.

## Way2 success criteria

Way2 is considered promising only if:

1. the same formulation and same `lambda` can be used across New Room and Hospital without per-map retuning;
2. it saves meaningful time/distance while staying within a quality-loss tolerance declared before independent validation;
3. it does not show systematic premature stopping while valuable reachable frontiers remain;
4. every stopping input is available online — no ground truth, final IoU/TU or future trajectory information is used.

## Hospital — canonical 1.0x state

Hospital remains configured at **scale 1.0x**.

Canonical runtime state:

- world: `map/hospital_aws_flat.sdf`;
- Hospital scale: `1.0`;
- TurtleBot4 is not scaled;
- spawn: `(0.0, 12.0, -1.57)`.

Hospital ground-truth tooling:

- `scripts/generate_hospital_ground_truth.py`
- `scripts/hospital_ground_truth_core.py`

Verified locally on `com1`:

```text
status: ok_frozen_v1_match
hospital_scale: 1.0
ROI cells: 215435
ROI SHA-256: 05d45b7aba66cb6dbb71e0406005f4a3e21875901af3ae72164b17f1d3add8d1
frozen_v1_match: true
wall segments: 1040
elevator blockers: 2
structural evaluation-mask cells: 245623
structural occupied cells: 30188
```

Generated heavy Hospital GT/ROI artifacts remain local and are not committed.

## Evaluation caveat

Current `iou_loss_vs_final` uses the reconstructed map at the **last analyzed policy decision** as its reference, not necessarily the actual final fully observed SLAM map.

Therefore a negative IoU loss means the earlier hypothetical-stop reconstruction outperformed the last-decision reconstruction under analyzer semantics; it does not automatically mean it outperformed the actual final observed map.

TU remains weakly discriminative in the current dataset and should not be the primary stopping signal.

## Current next actions

1. **Way1 remains closed; do not retune the P95 threshold.**
2. **Way2 replay is complete and raw `G = information_gain` is frozen.**
3. **Historical replay cost is frozen as `C = distance_m` (Euclidean).**
4. Analyze `R_t = max_f(G/C)` distributions and threshold crossings across all 17 development runs.
5. Test `K=2` guarded crossings for rebounds/premature stopping; do not use first-decision maxima to select `lambda`.
6. Define/freeze a defensible `lambda` or declared sensitivity range.
7. Backtest the frozen candidate rule on development data for failure modes and savings.
8. Freeze the complete Way2 rule before any prospective validation.
9. Integrate online stopping only after offline behavior is stable.
10. Collect new independent New Room + Hospital validation runs with no environment-specific retuning.
11. Improve the final IoU reference before making strong final map-quality claims.