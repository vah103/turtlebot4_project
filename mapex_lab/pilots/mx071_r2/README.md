# MX071 R2: DELL offline 2D pilot

This runner executes the accepted MX071 R2 Stage 1 design: four fixed KTH map IDs, two fixed starts per map, and eight PIPE_ALIGNED source trajectories. It preserves complete decision snapshots and compares five first-goal contracts followed by the same PIPE_ALIGNED continuation. Stage 2 is closed.

Design: [MX071 R2](https://github.com/vah103/chat-gpt/blob/e2987a65e23df7cd04fc60c19505492d771ce801/company/projects/mapex/MAPEX_MULTI_DIRECTION_2D_PILOT_DESIGN_R2_20261009.md), SHA-256 `e85156fa594fb164d02a7d521d13772eb24bfa9062ceac58469fcd9942479b8f`. Focused methodology review: [PR #51](https://github.com/vah103/chat-gpt/pull/51). This implementation has not received an independent result verdict.

Implementation review: [draft PR #40](https://github.com/vah103/turtlebot4_project/pull/40).

## Current execution checkpoint

On 2026-10-09 the first map block (`50052750`) completed both source trajectories and 20 logical first-goal requests. Both sources reached the 100 m budget. Six distinct snapshots represent seven fulfilled target slots; the remaining 80 m slot is retained as NA. One map is insufficient for the preregistered direction screening decision.

The original process then failed while writing the Markdown report: Python 3.6's default ASCII locale could not encode an em dash. The saved states and JSON/CSV outcomes survived. The same sealed Python implementation was resumed with `LC_ALL=C.UTF-8`, `LANG=C.UTF-8`, and `PYTHONIOENCODING=utf-8`. `evidence/locale_utf8_resume_20261009.json` records the failure phase and protected data hashes. No model, map, goal, score, controller, or budget changed.

The UTF-8 resume exposed redundant recomputation of completed offline diagnostics. A subsequent recorded three-line guard skips a snapshot only when all five selector contracts already have outcomes. Four focused checks passed: completed groups do not reconstruct or diagnose again; completed data hashes stay unchanged; a missing selector still enters execution; Python 3.6 compilation passes. No source or branch was interrupted for this fix. The resumed batch successfully wrote the UTF-8 report and entered source 3/8 (`50010535_PLAN2/S1`); `evidence/continuation_started.json` records the live-process checkpoint. The original execution manifest is retained; the current manifest records RESUME_FIX1 and its code hash. Original implementation commit: `b008d997a3332c758347a09cc12c5cbe594cd487`. See `evidence/completed_request_resume_fix1.json` and `evidence/resume_fix1_checks.json`.

## Source pins and machine

| Asset | Exact identity |
| --- | --- |
| Project base | `88fb673b7a19d2973cfb1c78b4d0a083fdf66b50` |
| PIPE | `e5bcb5ec4a9a13cbe14aa9f7850bc8b5989a2bb0` |
| MapEx | `53636bd1c79153acc3c74a532837d78c926bae5e` |
| MapEx LaMa | `b61dcb33e063fe9586b50e2dc7b70f97d5046c1d` |
| pyastar2d 1.0.2 | `0bfc4c1051cc768c5e543cdbc4b8f0a554834096` |

DELL has four logical CPU cores, approximately 8 GiB RAM, and no CUDA. The existing Python 3.6.13 / Torch 1.10.2 LaMa environment is `/home/dell/miniforge3/envs/lama`. The fixed three checkpoints remain in `/home/dell/MapEx/pretrained_models/weights/lama_ensemble/train_{1,2,3}/models/best.ckpt`; their full hashes are in the asset manifest.

The deployment root is `/home/dell/mx071_dell_20261009`. It contains pinned `pipe_source`, `pyastar_source`, isolated `deps`, `artifacts`, `logs`, and this isolated project worktree. Dependencies are Shapely 1.8.0, pyastar2d 1.0.2, and range_libc built from the pinned PIPE source. Building the extensions used `LDSHARED='g++ -shared'`; running them uses the process-scoped system libstdc++ preload shown below. The existing LaMa environment and system files were not changed.

## Run and resume

Use the prepared deployment assets. `preflight.py seal` is for a new deployment only; it must not overwrite an active asset seal. The current runner verifies the sealed hashes of every Python file before resuming. Run only one Stage 1 process at a time.

```bash
env LC_ALL=C.UTF-8 LANG=C.UTF-8 PYTHONIOENCODING=utf-8 \
  LD_PRELOAD=/usr/lib/x86_64-linux-gnu/libstdc++.so.6 \
  PYTHONPATH=/home/dell/mx071_dell_20261009/deps MPLBACKEND=Agg \
  /home/dell/miniforge3/envs/lama/bin/python \
  /home/dell/mx071_dell_20261009/project/mapex_lab/pilots/mx071_r2/run_stage1.py
```

The active resumed run logs to `/home/dell/mx071_dell_20261009/logs/stage1_resume_fix1.log`. The first UTF-8 resume log remains `logs/stage1_resume_utf8.log`. The original failure log remains `logs/stage1.log`. Completed sources and completed logical requests are skipped on resume. Source progress is checkpointed every 25 successful moves. A physically interrupted branch restarts from its original complete snapshot; current branch state is saved but is not loaded by this version. The UTF-8 recovery occurred after all branches in a map block had completed and did not restart any physical branch.

## Numerical and execution contracts

- Source helpers are audited under CODE_REFERENCE; Stage 1 trajectories use ALIGNED_SHARED, which explicitly blocks unknown cells. These are separate contracts.
- Physical sensor: 360 degrees, 20 m, 2500 rays. Predicted renderer: 250 rays, probability threshold 0.8, source Polygon.buffer(1), union, holes, rasterization, and flood fill.
- Native scoring uses the full immutable A* path's `path[2::3]`, without an appended endpoint, and divides by `max(1, sampled_count)`. Matched scoring appends the endpoint when absent, deduplicates samples, and uses full four-connected path metres.
- All first-goal requests share the exact snapshot, candidate pool, frozen uncertainty map, model identity, controller, RNG state, and remaining budget. Reuse requires the complete execution key; same goal alone is insufficient. Aliases retain valid structural zeros and do not increase independent sample size.
- Online P0 inputs contain observed data and predictions only. GT supplies the environment, fixed evaluation support, starts, and explicitly offline oracle diagnostics.
- The named aligned controller advances one successful four-connected cell (0.1 m) and scans after each move. It releases a reached, invalid, or unreachable goal and continues with PIPE_ALIGNED. The budget is 1000 successful moves / 100 m; GT IoU does not stop the run.
- The common online predictor runs frozen members serially on CPU, crops actual centered LaMa padding back to the original frame, and never resizes maps. Channel 0, NumPy mean, and Torch unbiased variance with K=3 match the audited source convention.
- Named adaptations also remove unused constant mapper visualization buffers, render serially, and fail closed on empty/nonfinite/nonboolean source render output. Source MultiPolygon behavior is preserved, not repaired.

P0/P1/P4 are the primary frozen-input source-score comparisons. M0 is matched PATH_PATHCOST. MAPEX_ALIGNED uses the native probabilistic mean-map endpoint visibility and Euclidean denominator. M1/M4 and endpoint/path controls are scoring-only; they do not add rollouts.

Coverage is observed GT free space over the fixed initially reachable valid component. Q is the right-continuous coverage gain integral over the common remaining distance budget; early behavioral termination carries coverage forward. Infrastructure failure produces missing Q. Checkpoint means are aggregated within start, then equally across starts and maps. Positive screening requires all four maps, the sealed effect threshold and diversity/support criteria; partial results cannot qualify.

H4 is oracle headroom, not a deployable method. GT-250 and GT-2500 comparison uses the same declared scored poses. A residual between declared route visibility and actual first-goal observations is labeled `EXECUTION_SENSOR_CONTRACT_GAP`; it cannot isolate controller or ray count alone. H7 compares distinct tested actions only. H2/H3 are not evaluated; H5/H6 are deferred.

## Completed technical checks

The frozen evidence bundle records:

- 12 preliminary parity checks, extended source score/goal/mask parity, and a holes/degenerate-source ledger.
- Complete serialized-state controller replay using the real ensemble and a five-move technical fixture. This is not a full 100 m replay result.
- 15 scientific contract checks, including exact native MapEx endpoint scores, immutable inputs, alias keys, analytic Q/carry-forward behavior, and missing infrastructure outcomes.
- All 989 generator state keys loaded exactly from each of the three checkpoints.
- Real three-member CPU inference on 1326 x 1428 input: summed inference 158.25 s; monitored peak RSS 3450.80 MiB.
- Largest-map single-member probe on 1442 x 2327 input: 103.80 s elapsed; peak RSS 4725.40 MiB; minimum available memory 631.70 MiB. Shared generator architecture is checked across all members. This does not claim a simultaneous three-model largest-map memory measurement.
- Python 3.6 compilation passed.

Preflight raised the per-worker RSS guard from 4608 to 5120 MiB before scientific collection, retaining the 256 MiB available-memory guard, 512 MiB disk guard, 1200 s per-member timeout, exact models, and full map dimensions. Original guard-aborted measurements remain in the evidence bundle.

## Data locations and limits

The committed CSV evidence copy normalizes CRLF to LF; cell values are unchanged. Heavy rasters, complete states, and branch traces stay under `/home/dell/mx071_dell_20261009/artifacts/stage1`, outside git. The root includes:

`manifest.json`, `progress.json`, `source_outcomes.jsonl`, `snapshots.jsonl`, `candidate_scores.jsonl`, `branch_outcomes.jsonl`, `error_update_diagnostics.jsonl`, `map_start_summary.csv`, `screening_summary.json`, `h7_tested_action_summary.json`, and `direction_screening.md`.

The four fixed IDs are `50052750`, `50010535_PLAN2`, `50015847`, and `50037765_PLAN3`. The cohort is MAP_ID_ONLY / TRAIN_OVERLAP_UNVERIFIED; these are not four verified independent buildings or a held-out generalization benchmark.

Cache reuse changes compute cost, not action inputs. Worker timing sidecars are overwritten by later calls, and counters describe the current process segment after resume; complete cumulative cost telemetry is not claimed. Per-member inference retries are limited to one same-configuration retry. The runner does not durably cap repeated externally interrupted branch restarts.

This is an engineering pilot of PIPE_ALIGNED behavior. It is not a reproduction of native PIPE/controller performance, an accepted scientific result, an online STOP deployment, or MX072.
