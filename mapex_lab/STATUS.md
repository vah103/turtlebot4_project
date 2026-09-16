# mapex_lab status

_Last synchronized with `main`: 2026-09-16._

This file records the current code/research state on `main`.

## Current focus

The active thesis direction remains **Direction 2: early stopping for MapEx**, but the research path has now moved from **Way1: absolute global uncertainty threshold** to **Way2: frontier-utility-based stopping**.

Research question:

> **When is the best remaining reachable frontier no longer worth its expected information gain relative to the physical cost of reaching it?**

The online MapEx policy has **not** yet been changed to stop early. Way2 is currently in the algorithm-design/offline-analysis stage.

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

The recorded decision/candidate data already exposes the quantities needed for Way2 analysis, including `information_gain`, `distance`, `score`, selected frontier state and candidate eligibility/suppression information.

---

# Way1 — CLOSED

## Definition

Way1 used the global uncertainty statistic:

```text
unknown_variance_p95
```

computed over cells that are currently unknown in the observed map.

Stopping rule:

```text
P95 <= threshold for K consecutive distinct decision states
```

Primary frozen rule:

```text
P95 <= 0.23, K=1
```

Repeated identical terminal policy ticks were collapsed before persistence counting.

## Development/tuning — New Room mpx_001...mpx_010

`mpx_001...mpx_010` were the Way1 development/tuning dataset.

Fixed-rule results:

| Rule | Time saved | Distance saved | Mean IoU loss | Worst IoU loss |
|---|---:|---:|---:|---:|
| `P95 <= 0.23, K=1` | 23.64% | 24.03% | -0.00275 | 0.00938 |
| `P95 <= 0.25, K=2` | 22.23% | 22.67% | -0.00285 | 0.00977 |
| `P95 <= 0.20, K=1` | 17.28% | 16.90% | -0.00030 | 0.00540 |

LOOCV showed that repeatedly selecting the maximum-savings rule on only nine training runs could become too aggressive. Therefore `0.23 x 1` was frozen before prospective validation.

## Prospective New Room validation — mpx_011...mpx_015

Five new runs were collected after freezing `P95 <= 0.23, K=1`.

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

Way1 was then tested without retuning on canonical Hospital 1.0x.

### hpx_001

```text
P95 min    = 0.318309
P95 median = 0.333203
P95 last   = 0.333257
trigger     = false
```

Full-run reference:

```text
decisions = 56
time      = 2063.74 s
distance  = 769.94 m
IoU       = 0.429910
TU        = 0.55
```

### hpx_002

```text
P95 min    = 0.331268
P95 median = 0.333242
P95 last   = 0.333297
trigger     = false
```

Full-run reference:

```text
decisions = 50
distance  = 695.34 m
IoU       = 0.417957
TU        = 0.52
coverage  = 0.999331585
```

### Way1 conclusion

- Hospital trigger rate with frozen `P95 <= 0.23, K=1`: **0/2**.
- Hospital P95 remains close to `1/3` through most of both runs.
- With three prediction models and sample variance (`ddof=1`), a binary-like disagreement pattern such as `[0,0,1]` or `[0,1,1]` gives variance `1/3`; therefore a high global quantile can remain saturated if a persistent subset of unknown cells keeps strong ensemble disagreement.
- Retuning the absolute threshold specifically for Hospital would undermine the original goal of a cross-environment stopping criterion.

**Way1 is therefore closed as the primary research path.**

No `hpx_003` is required merely to reconfirm the same Way1 failure mode. `hpx_001` and `hpx_002` are sufficient diagnostic evidence to motivate redesign, but not enough to claim a statistically general failure across all Hospital runs.

---

# Way2 — OPEN

## Design objective

Way2 asks a more direct exploration decision question:

> **Does any remaining reachable frontier still have enough expected information value to justify the cost of physically visiting it?**

The goal is to derive the stopping condition from a cost-benefit objective first, then use recorded runs to inspect behavior and failure modes. The algorithm should not be created by repeatedly sweeping thresholds until the experiment output looks favorable.

## Core one-step utility formulation

At decision `t`, for reachable/selectable frontier `f`:

- `G_t(f)` = expected information gain from visiting `f`;
- `C_t(f)` = travel cost to `f`;
- `lambda` = declared exchange rate between information gain and travel cost.

Define:

```text
V_t(f) = G_t(f) - lambda * C_t(f)
```

and:

```text
V*_t = max_f V_t(f)
```

over the remaining reachable/selectable frontiers.

Core stopping rule:

```text
if V*_t <= 0:
    stop exploration
else:
    continue exploration
```

This is a **one-step utility-based stopping rule**, not a full-horizon globally optimal stopping solution. It is intentionally compatible with the current MapEx decision pipeline.

## Information gain G

MapEx already computes `information_gain` for every candidate frontier using ensemble uncertainty over cells that are both:

- currently unknown; and
- predicted visible from the frontier.

Way2 should reuse this quantity rather than invent an unrelated stopping signal.

For cross-environment use, gain normalization must be audited before it is frozen. A candidate is to normalize per-cell variance by a theoretically justified ensemble reference scale rather than by a run-specific maximum. With the current three-model pipeline, prediction range and variance semantics must be verified before treating `1/3` as the final normalization constant.

Do **not** normalize by the maximum score observed so far in a run: startup frontiers can be extremely close and generate abnormally large `IG/distance` values, which can make all later utilities look artificially small.

## Travel cost C

Initial offline baseline:

```text
C_t(f) = current Euclidean frontier distance
```

Preferred final definition if robustly available online:

```text
C_t(f) = planner path length to frontier
```

Planner path length is more physically meaningful in Hospital because walls, corridors and detours can make Euclidean distance underestimate actual navigation cost.

## Reachability

Stopping utility must only consider frontiers that are genuinely eligible for execution.

Planner-unreachable/suppressed candidates must not keep `V*_t` artificially positive and prevent stopping.

## Meaning of lambda

`lambda` is the only central trade-off parameter in the core formulation:

```text
lambda = required information gain per unit travel cost
```

It should **not** be chosen by running many experiments and selecting the value that gives the prettiest savings.

The preferred route is to define `lambda` from an explicit cost-quality objective before independent validation. If a defensible operational objective is not yet available, perform a declared sensitivity analysis and do not label one post-hoc value as a validated final parameter.

## Noise guard

Per-decision gain/utility can fluctuate because of SLAM, frontier extraction and prediction noise.

Candidate implementation guard:

```text
require V*_t <= 0 for K=2 distinct decision states before stopping
```

`K=2` is treated as a debounce guard, not a parameter to sweep for maximum savings. If future analysis supports a confidence-bound rule, that could replace the fixed persistence guard later.

## Way2 data roles

Because existing runs are already being inspected while Way2 is designed:

- `mpx_001...mpx_015` = Way2 development/diagnostic data if used in formulation checks;
- `hpx_001...hpx_002` = Way2 development/diagnostic data;
- these runs must **not** later be presented as independent Way2 validation.

Independent validation must use new runs collected only after the Way2 formula, normalization, `lambda`, cost definition and guard are frozen.

## Way2 implementation roadmap

### A. Audit current code/data

Verify exactly how `mapex.py` computes and records:

- `information_gain`;
- `distance`;
- `score = IG/distance`;
- selectable/suppressed candidates;
- planner reachability semantics.

Confirm whether existing `candidates.csv` / `decisions.csv` are sufficient for offline replay without changing the online controller.

### B. Build offline Way2 replay

For every recorded decision in existing development/diagnostic runs:

```text
compute G_t(f)
compute C_t(f)
compute V_t(f)
compute V*_t
```

The purpose is to test whether the formulation behaves sensibly and expose failure modes, not to fit a hidden threshold.

### C. Freeze gain and cost definitions

Compare only scientifically motivated definitions, especially:

- Euclidean distance vs planner path length;
- raw summed variance vs a justified normalized gain.

Choose the version that is physically meaningful, online-computable and consistent across New Room/Hospital.

### D. Define lambda from the objective

Specify the accepted cost-quality trade-off before validation.

If no defensible final `lambda` can yet be declared, report a sensitivity range and keep Way2 in development status.

### E. Backtest development data

Evaluate:

- premature-stop timing;
- time saved;
- distance saved;
- IoU loss;
- TU loss;
- behavior in New Room vs Hospital.

Backtesting is for debugging/design diagnosis, not proof of generalization.

### F. Freeze Way2

Freeze all of the following before prospective validation:

- `G_t(f)` definition;
- `C_t(f)` definition;
- normalization;
- `lambda`;
- persistence/noise guard.

### G. Integrate online

Insert the stopping decision after reachable frontier scoring is available:

```text
map -> predictions -> variance -> frontiers -> reachability
-> compute gain/cost/utility
-> V*_t <= 0 for guard period ?
   yes -> stop with explicit utility-exhausted reason
   no  -> continue normal MapEx frontier execution
```

### H. Independent validation

Collect entirely new runs after freeze on both:

- New Room;
- Hospital.

Use the **same formula and same lambda** without environment-specific retuning.

Compare Way2-stopped exploration against the corresponding full MapEx behavior using time, distance, IoU, TU and stop timing.

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

1. **Do not continue Way1 threshold retuning.**
2. **Do not run `hpx_003` merely to reconfirm Way1 P95 saturation.**
3. Audit `mapex.py`, `candidates.csv` and `decisions.csv` for the exact Way2 gain/cost/reachability semantics.
4. Build an offline Way2 replay using existing `mpx_001...015` and `hpx_001...002` as development/diagnostic data.
5. Decide whether Way2 final travel cost should use planner path length or Euclidean distance.
6. Verify a defensible cross-environment normalization for information gain.
7. Define `lambda` from an explicit cost-quality objective, or keep a declared sensitivity analysis if the objective is not yet fixed.
8. Backtest for failure modes; do not cherry-pick the best-looking parameter from old runs.
9. Freeze the full Way2 rule before collecting new independent validation runs.
10. Integrate online stopping into `mapex.py` only after the offline formulation is stable.
11. Improve the final IoU reference before making strong final map-quality claims.
