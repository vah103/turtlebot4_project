# mapex_lab status

_Last synchronized with `main`: 2026-09-14._

This file describes the code and research state that actually exist on the current `main` branch. Historical experiments or ideas that are no longer active are not treated as current state.

## Current focus

- Maintain a reproducible exploration benchmark comparing Nearest Frontier (NF) and MapEx in New Room.
- Keep the navigation/execution layer as shared as possible so the comparison focuses on frontier-selection policy rather than different Nav2 behavior.
- Record run provenance, trajectory/map metrics, prediction artifacts and offline evaluation outputs.
- The active research direction is **Direction 2: prediction/uncertainty-aware early stopping**.
- Current research question: **At what point has MapEx learned enough that further physical exploration is no longer worth its time/distance cost while reconstructed map quality remains acceptable?**
- The online MapEx policy has **not** been modified with a stopping rule yet. Early-stopping work remains offline until independent validation is completed.

## Primary way to run experiments

The normal simulation entry point is:

```bash
./mapex_lab/run
```

Current behavior of `run`:

- interactively asks for `NF` or `MapEx`;
- asks whether the run should be recorded;
- asks for a run ID when recording and refuses to overwrite an existing run directory;
- sources ROS 2 Jazzy and `ros2_ws/install/setup.bash`;
- launches `launch/slam.launch.py`;
- waits until `/map` and `/navigate_to_pose` are visible, then waits a short stabilization interval;
- dispatches:
  - NF, no record -> `scripts/nf_basic.py`
  - MapEx, no record -> `scripts/mapex.py`
  - NF, record -> `scripts/nf_run.py --runtime-profile slam`
  - MapEx, record -> `scripts/mapex_run.py --runtime-profile slam`
- defaults to `ROS_DOMAIN_ID=0` unless the environment already sets another value;
- handles Ctrl+C by stopping the launched simulation/SLAM/Nav2 process.

Important: the current runner does **not** implement the previously documented domain-42 isolation, `ROS_LOCALHOST_ONLY=1`, stale-ROS/Gazebo rejection, Nav2 lifecycle checks, or a 5 s monotonic `/clock` gate. Those features must not be described as active unless they are implemented in the code first.

## Current launch stack

Files currently present under `mapex_lab/launch/`:

- `slam.launch.py` — primary simulation launch used by `./mapex_lab/run`.
- `slam_robot.launch.py` — launch path for robot-side/real-robot use.
- `stock.launch.py` — stock SLAM diagnostic/baseline launch.
- `local.launch.py` — local scan frontend diagnostic launch.
- `submap.launch.py` — segmented submap/local-ICP diagnostic launch.
- `toolbox.launch.py` — auxiliary Toolbox diagnostic launch.
- `new_toolbox.launch.py` — auxiliary newer Toolbox diagnostic launch.

`oldmap_toolbox.launch.py` is no longer present and must not be treated as an active runtime profile.

## Configuration source of truth

`mapex_lab/config/` currently contains only:

- `mapex.yaml` — MapEx policy/model parameters.
- `nav2.yaml` — Nav2 overrides.
- `slam.yaml` — SLAM Toolbox configuration shared by the relevant launchers.

Do not document additional config files as active unless they exist on `main`.

## NF implementation

- `scripts/nf_basic.py` — canonical Nearest-Frontier exploration/execution layer.
- `scripts/nf_run.py` — NF benchmark wrapper and provenance registration.
- `scripts/_nf_run_core.py` — shared recorder/runtime core used by NF benchmarking.
- `scripts/evaluate_nf_profiled.py` — post-run/profile-aware NF evaluation support.

NF run data is stored under:

```text
mapex_lab/experiments/nearest/<run_id>/
```

## MapEx implementation

- `scripts/mapex.py` — MapEx exploration policy and frontier ranking.
- `scripts/mapex_run.py` — MapEx benchmark recording, map/prediction artifact saving and evaluation integration.
- `scripts/mapex_lama_bridge.py` — bridge from the ROS-side process to the LaMa inference environment.
- `scripts/mapex_lama_worker.py` — online LaMa ensemble inference worker.
- Online MapEx uses the three-model ensemble configuration; the separate all-training predictor is an offline evaluation path rather than a fourth online ensemble member.

MapEx run data is stored under:

```text
mapex_lab/experiments/mapex/<run_id>/
```

## Offline all-training evaluation

Current offline evaluation tooling includes:

- `scripts/predict_alltrain_offline.py` — generate all-training predictions from completed run snapshots without ROS.
- `scripts/reevaluate_alltrain.py` — batch/re-evaluate completed NF/MapEx runs using the all-training predictor.
- `scripts/evaluate_mapex_run.py` — core MapEx IoU/TU evaluator.
- `scripts/evaluate_mapex_profiled.py` — environment/profile-aware MapEx evaluation wrapper.
- `scripts/evaluate_nf_profiled.py` — NF evaluation with compatible prediction/evaluation handling.

Recent evaluation work on 2026-09-13:

- ground-truth-specific evaluation canvas metadata was adopted;
- reused all-training predictions are checked against the requested checkpoint;
- the all-training prediction source label was normalized;
- NF and MapEx all-training evaluation outputs were refreshed;
- the temporary IoU-curve export workflow was removed after the comparison curves were generated.

## Ground truth and map-quality evaluation

The benchmark supports environment-specific ground truth / ROI handling.

For New Room:

- `scripts/generate_new_room_ground_truth.py` generates the structural GT and connected-free ROI when needed;
- generated heavy artifacts remain local under `ground_truth/new_room/generated/`;
- Coverage uses the connected-free ROI for the active environment;
- MapEx evaluation uses occupied IoU and TU against the active environment's structural GT/ROI.

Hospital retains its own ground-truth/profile files for the workflows that explicitly select that environment.

## Current repository structure relevant to the benchmark

```text
mapex_lab/
├── run
├── config/
│   ├── mapex.yaml
│   ├── nav2.yaml
│   └── slam.yaml
├── launch/
│   ├── slam.launch.py
│   ├── slam_robot.launch.py
│   ├── stock.launch.py
│   ├── local.launch.py
│   ├── submap.launch.py
│   ├── toolbox.launch.py
│   └── new_toolbox.launch.py
├── scripts/
│   ├── nf_basic.py
│   ├── nf_run.py
│   ├── _nf_run_core.py
│   ├── mapex.py
│   ├── mapex_run.py
│   ├── mapex_lama_bridge.py
│   ├── mapex_lama_worker.py
│   ├── analyze_early_stopping.py
│   ├── sweep_early_stopping_thresholds.py
│   ├── validate_early_stopping_loocv.py
│   ├── compare_fixed_early_stopping_rules.py
│   ├── predict_alltrain_offline.py
│   ├── reevaluate_alltrain.py
│   ├── evaluate_nf_profiled.py
│   ├── evaluate_mapex_run.py
│   ├── evaluate_mapex_profiled.py
│   └── ...
├── experiments/
│   ├── nearest/
│   └── mapex/
├── ground_truth/
├── analysis/
├── results/
├── references/
├── docs/
└── src/
    └── README.md
```

`src/slam/` has been removed. `src/` currently contains only its README describing possible future module organization.

`scripts/apply_hard_chain.py` still exists as a legacy diagnostic helper, but it is not part of the primary NF/MapEx pipeline and must not be presented as the current research direction.

## Static MapEx audit state

A static comparison with the MapEx paper/reference implementation was previously recorded in `docs/experiment_notes/2026-09-09-mapex-audit.md`.

Current interpretation:

- the main MapEx IG/ranking idea follows the paper-level pipeline;
- there are implementation/evaluation adaptations that must remain documented when making formal claims;
- numerical/runtime equivalence to the reference implementation should not be assumed without dedicated validation;
- candidate eligibility differences between NF and MapEx should be considered when interpreting benchmark results.

## Active research direction — Direction 2: prediction/uncertainty-aware early stopping

The active thesis direction is to use MapEx prediction/uncertainty not only to rank frontiers but also to decide whether further physical exploration is still useful.

Core idea:

- current MapEx uses ensemble prediction + uncertainty to score candidate frontiers;
- the extension asks whether the same uncertainty signal can indicate that the remaining unknown area is already predictable enough to stop;
- if stopping at decision `t`, the reconstructed map is formed from observed SLAM evidence on currently known cells plus ensemble-mean prediction on the remaining unknown cells;
- evaluation then measures saved time/distance versus occupied IoU/TU degradation;
- structural GT/ROI is used only for offline evaluation/calibration and is never an online stopping input.

The online MapEx policy remains unchanged while this hypothesis is validated offline.

### Early-stopping tooling currently implemented

- `scripts/analyze_early_stopping.py`
  - replays each recorded MapEx policy decision;
  - computes online-available uncertainty statistics on currently unknown cells;
  - reconstructs the map that would result from stopping at that decision;
  - evaluates Coverage, occupied IoU, TU, time saved and distance saved;
  - writes `early_stopping_analysis.csv` in each run directory.
- `scripts/sweep_early_stopping_thresholds.py`
  - evaluates rules of the form `unknown_variance_p95 <= threshold for K consecutive decision states`;
  - collapses exact repeated online states by default so persistence is not satisfied by duplicate policy ticks.
- `scripts/validate_early_stopping_loocv.py`
  - performs leave-one-run-out rule selection/validation on the current 10-run development dataset.
- `scripts/compare_fixed_early_stopping_rules.py`
  - compares a small frozen set of fixed rules using the same state-collapse/stop semantics as the sweep;
  - current default candidates are `0.23×1`, `0.25×2`, and `0.20×1`.

### Current development dataset

Completed MapEx runs:

```text
mpx_001 ... mpx_010
```

All 10 have per-decision `early_stopping_analysis.csv` outputs.

These runs are now treated as the **development/tuning dataset** because they were used to inspect P95 behavior, sweep thresholds/persistence and choose the leading candidate rule. They must not be presented as an independent validation set for the selected rule.

### Main uncertainty signal

`unknown_variance_p95` (P95 ensemble variance over currently unknown observed-map cells) is the leading stopping signal.

Observed behavior across the development runs:

- P95 stays relatively high through much of exploration and drops late;
- transient low-P95 states exist, which motivated testing persistence `K > 1`;
- mean variance is less stable/universal than P95 across the current runs;
- repeated no-progress tail decisions exist in several runs, so persistence must be counted across distinct online states rather than duplicate policy ticks.

### Threshold sweep results

Sweep space:

- 21 P95 thresholds from `0.10` to `0.30`;
- persistence `K ∈ {1,2,3}`;
- 63 total rules;
- exact duplicate online states collapsed before persistence evaluation.

Representative safe rules on the 10-run development dataset, requiring all 10 runs to trigger, worst IoU loss `<= 0.01`, and worst TU loss `<= 0`:

- `P95 <= 0.23, K=1`
  - mean distance saved: **24.03%**
  - mean time saved: **23.64%**
  - mean IoU loss: **-0.00275**
  - worst IoU loss: **0.00938**
- `P95 <= 0.25, K=2`
  - mean distance saved: **22.67%**
  - mean time saved: **22.23%**
  - mean IoU loss: **-0.00285**
  - worst IoU loss: **0.00977**
- `P95 <= 0.20, K=1`
  - mean distance saved: **16.90%**
  - mean time saved: **17.28%**
  - mean IoU loss: **-0.00030**
  - worst IoU loss: **0.00540**

Aggressive examples such as `0.30×1` save substantially more distance/time but increase worst-case IoU loss above `0.02`, so maximizing savings alone is not acceptable.

### LOOCV result and interpretation

Leave-one-run-out validation was applied to the same 10-run development pool:

- `0.23×1` was selected on 9/10 folds;
- when `mpx_004` was held out, the remaining 9 runs allowed the more aggressive `0.30×3` rule to be selected;
- held-out mean distance saved: **24.54%**;
- held-out mean time saved: **24.07%**;
- mean held-out IoU loss: **-0.00110**;
- worst held-out IoU loss: **0.02066** on `mpx_004`;
- TU loss remained `0`.

Interpretation:

- the P95 early-stopping signal is promising;
- adaptive rule selection that maximizes savings on only 9 training runs is not robust enough;
- the `mpx_004` failure is specifically a warning about dynamic rule selection choosing `0.30×3`, not evidence that all fixed P95 rules fail.

### Fixed-rule comparison

The three frozen candidates were then applied consistently to all 10 development runs.

`P95 <= 0.23, K=1`:

- triggered: `10/10`;
- mean distance saved: **24.03%**;
- mean time saved: **23.64%**;
- mean IoU loss: **-0.00275**;
- worst IoU loss: **0.00938** (`mpx_009`);
- runs with IoU loss `> 0.005`: `mpx_003`, `mpx_009`;
- runs with IoU loss `> 0.01`: `0`;
- worst TU loss: `0`.

`P95 <= 0.25, K=2`:

- triggered: `10/10`;
- mean distance saved: **22.67%**;
- mean time saved: **22.23%**;
- mean IoU loss: **-0.00285**;
- worst IoU loss: **0.00977** (`mpx_009`);
- runs with IoU loss `> 0.005`: `mpx_009`;
- runs with IoU loss `> 0.01`: `0`;
- worst TU loss: `0`.

`P95 <= 0.20, K=1`:

- triggered: `10/10`;
- mean distance saved: **16.90%**;
- mean time saved: **17.28%**;
- mean IoU loss: **-0.00030**;
- worst IoU loss: **0.00540** (`mpx_003`);
- runs with IoU loss `> 0.005`: `mpx_001`, `mpx_003`;
- runs with IoU loss `> 0.01`: `0`;
- worst TU loss: `0`.

Current rule interpretation:

- **Primary candidate:** `P95 <= 0.23, K=1` — highest savings among the current fixed candidates while remaining below `0.01` worst IoU loss on all 10 development runs.
- **Persistence challenger:** `P95 <= 0.25, K=2` — conceptually useful against transient low-P95 states, but it does not currently show a quantitative advantage over `0.23×1` on the development dataset.
- **Conservative baseline:** `P95 <= 0.20, K=1` — lower savings but lower worst IoU degradation.

These labels are development-set conclusions only. `0.23×1` must not yet be described as the final validated online stopping rule.

### Evaluation caveat

Current `iou_loss_vs_final` uses the reconstructed map at the **last analyzed policy decision** as its reference, not necessarily the actual final fully observed SLAM map.

Therefore:

- negative IoU loss can occur when an earlier reconstructed map is better than the last-decision reconstruction;
- claims about final map-quality degradation must keep this reference definition explicit;
- a future improvement should also compare stopping reconstructions against an actual final observed-map benchmark where appropriate.

TU is currently weakly discriminative across these runs (mostly around the same low value) and should not be treated as the main stopping signal.

### Independent prospective validation — next formal step

The next formal experiment is to collect **new MapEx runs `mpx_011` onward** under the same New Room runtime/configuration as the development runs.

Planned protocol:

- do **not** change `mapex.py`, Nav2 parameters, LaMa ensemble configuration, map, launch stack, recording semantics, or candidate thresholds before collecting the new runs;
- run normal full MapEx exploration and record it exactly as before;
- keep the stopping rules offline during validation;
- freeze the three current rules before examining the new results:
  - primary: `P95 <= 0.23, K=1`;
  - challenger: `P95 <= 0.25, K=2`;
  - conservative baseline: `P95 <= 0.20, K=1`;
- treat `mpx_011+` as an independent validation set and **do not retune thresholds using those runs**;
- for each new run, compute whether the frozen rule triggers, time/distance saved, IoU loss and TU loss;
- only after independent validation succeeds should an early-stopping condition be integrated into the online MapEx policy.

Target batch currently planned for the next session:

```text
mpx_011 ... mpx_020
```

## Direction 1 — retained as a secondary alternative

Direction 1 remains available as a fallback/secondary research direction but is not the active implementation focus.

- Improve MapEx frontier selection while preserving full exploration.
- Investigate whether `IG / EuclideanDistance` causes unnecessary travel, difficult goals, poor navigation success or weak information gained per travelled meter.
- Candidate changes may use stronger travel penalties, path/navigation cost, reachability or navigation-success likelihood while preserving the MapEx prediction/uncertainty pipeline.

No Direction-1 scoring change should be implemented while Direction 2 validation is the active experiment unless explicitly decided otherwise.

## Removed / no longer active

The following items were previously described in this file but are no longer part of the current `main` state:

- `launch/oldmap_toolbox.launch.py`;
- `scripts/nf_oldmap_toolbox_run.py`;
- `scripts/mapex_oldmap_toolbox_run.py`;
- `src/slam/adaptive_anchor_v1.md`;
- `src/slam/oldmap_anchor_v1.md`;
- `src/slam/oldmap_anchor_v2.md`;
- `src/slam/temporal_anchor_ceres.patch`;
- the Old-Map-First V1/V2 implementation plan as an active next action.

Historical commits may still contain these files, but they must not be used to describe the current branch.

## Current known limitations / next actions

1. `mpx_001 ... mpx_010` are the development/tuning set for early stopping; do not reuse them as the independent validation set.
2. Before the next formal run batch on `com1`, `git pull` so the local checkout matches `main`.
3. Collect `mpx_011 ... mpx_020` with the same New Room MapEx protocol/configuration as the first 10 runs.
4. Do not modify the frozen rule thresholds after seeing the validation runs; otherwise those runs cease to be independent validation data.
5. Analyze each new run with `analyze_early_stopping.py`, then evaluate the frozen candidate rules without running a new threshold sweep on the validation set.
6. Verify the unusually large accumulated New Room trajectory distances (~285–331 m in the development runs) before using absolute distance claims in final thesis conclusions; check for odometry/map jumps, resets or simulation artifacts.
7. Improve/clarify the final IoU reference so stopping quality can also be compared against an actual final observed SLAM benchmark where appropriate.
8. Only after the frozen rule performs acceptably on independent validation data should early stopping be integrated into `mapex.py` for online experiments.
