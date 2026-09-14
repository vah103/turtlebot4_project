# mapex_lab status

_Last synchronized with `main`: 2026-09-14._

This file describes the code that actually exists on the current `main` branch. Historical experiments or ideas that are no longer present in the repository are not treated as active state.

## Current focus

- Maintain a reproducible exploration benchmark comparing Nearest Frontier (NF) and MapEx in New Room.
- Keep the navigation/execution layer as shared as possible so the comparison focuses on frontier-selection policy rather than different Nav2 behavior.
- Record run provenance, trajectory/map metrics, prediction artifacts and offline evaluation outputs.
- Use completed NF/MapEx runs to identify a defensible MapEx research direction instead of assuming the bottleneck in advance.

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

## Candidate research directions

Two directions are currently being considered. Neither is selected as the final thesis contribution yet.

### Direction 1 — Improve MapEx frontier selection while preserving full exploration

- Keep the original full-exploration completion objective.
- Test whether `IG / EuclideanDistance` causes unnecessary travel, difficult goals, poor navigation success, or weak information gained per travelled meter.
- Candidate changes may use stronger travel penalties, path/navigation cost, reachability or navigation-success likelihood while preserving the MapEx prediction/uncertainty pipeline.
- Primary target: reduce time/distance and improve robustness while maintaining or improving Coverage, occupied IoU and TU.
- Any scoring change must be justified by evidence from existing runs.

### Direction 2 — Prediction/uncertainty-aware early stopping

- Use MapEx prediction not only to rank frontiers but also to decide whether further physical exploration is still useful.
- Permit earlier termination only when remaining unknown regions are predicted with sufficiently justified confidence/low useful uncertainty.
- Reconstruct the final map from observed SLAM evidence plus prediction for the remaining unobserved area.
- Measure the trade-off between saved time/distance and any loss in occupied IoU/TU.
- Do not assume an arbitrary fixed stopping coverage; derive the criterion from prediction confidence/uncertainty and data.
- Prefer offline validation on intermediate snapshots before changing the online exploration policy.

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

1. Treat `./mapex_lab/run` as implemented in the current shell script, not as described by older status notes.
2. On `com1`, `git pull` before the next formal run so the local checkout matches `main`.
3. Run fresh NF and MapEx smoke tests through the current runner and confirm clean startup, exploration completion, recording and offline evaluation.
4. If ROS-domain isolation, stale-process checks or clock-monotonicity validation are still desired, implement and test them in `run` before marking them DONE here.
5. Keep new NF/MapEx formal batches on one fixed runtime/configuration and do not mix results collected under materially different code/config states.
6. Use the existing completed-run analysis to choose and justify the final MapEx improvement direction.
