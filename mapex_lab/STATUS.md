# mapex_lab status

_Last synchronized with `main`: 2026-09-17._

## Current focus

The active thesis direction is **Way2: frontier-utility-based early stopping for MapEx**.

- Way1 is closed.
- Way2 is implemented online and the candidate rule is **FROZEN**.
- Online sanity testing has been completed.
- Five Way2 online runs are currently stored in the repository:
  - New Room: `mpx_w2_001`, `mpx_w2_002`
  - Hospital: `hpx_w2_001`, `hpx_w2_002`, `hpx_w2_003`
- `mpx_w2_003` has not been collected yet.
- `mpx_w2_001` is the integration/sanity run and must not be counted as independent prospective validation evidence.
- Baseline `scripts/mapex.py` remains unchanged; Way2 is isolated in `scripts/mapex_way2.py`.
- The frozen thresholds must not be retuned from these online outcomes.
- The 17 historical runs used to develop/audit Way2 remain development data only.
- For New Room baseline comparison, `mpx_001` through `mpx_010` are one comparable runtime/setup cohort. Historical differences in `runtime_profile*` strings are provenance/naming artifacts and must not be used to split that cohort. See `experiments/mapex/RUN_CONTEXT.json`.
- **New Room coverage evaluator correction:** `new_room_connected_free_v1` is superseded because it rasterized SDF world coordinates directly against SLAM-start canvases even though New Room spawns at world `(0,3,0)`. New runs use frame-correct `new_room_connected_free_v2`. Historical New Room coverage values remain recorded as v1 until offline backfill from saved canvases is validated; time, distance, trajectory, navigation outcomes, MapEx scoring, and Way2 R/U are unaffected.

Frozen online rule:

```text
For each evaluable decision t over selectable frontiers f:

R_t = max_f information_gain(f) / distance_m(f)
U_t = max_f visible_unknown_cells(f) / distance_m(f)

base_valid_t := (R_t <= 0.30) AND (U_t <= 10.0)

Persistence / confirmation:
- any evaluable base-invalid state resets valid_count to 0;
- 1st consecutive base-valid state: continue;
- 2nd consecutive base-valid state:
    - if selectable candidate_count <= 1: STOP;
    - otherwise continue and require one more confirmation;
- 3rd consecutive base-valid state: STOP regardless of candidate count.
```

The rule is frozen as a **candidate validation rule**, not yet claimed as a validated final method.

---

# Current online Way2 runs

| Run | Environment | Termination | Coverage | Distance | Time | Policy decisions | Notes |
|---|---|---|---:|---:|---:|---:|---|
| `mpx_w2_001` | New Room | Way2 early stop | 0.837226 | 275.65 m | 766.09 s | 29 | Integration/sanity only. Old recorder wrote `exploration_complete` in summary, but the recorded Way2 state is `stop_second_valid_single_candidate`. |
| `mpx_w2_002` | New Room | Normal completion | 0.838430 | 309.93 m | 841.93 s | 35 | Way2 did not trigger; the last valid state was followed by no remaining frontier and baseline stable completion. |
| `hpx_w2_001` | Hospital | Normal completion | 0.995330 | 665.97 m | 1809.18 s | 57 | Recorded before per-decision `way2_checks.csv` was added. |
| `hpx_w2_002` | Hospital | Normal completion | 0.996342 | 704.92 m | 1870.23 s | 53 | Native `way2_checks.csv`; 48 evaluable Way2 checks. |
| `hpx_w2_003` | Hospital | Way2 early stop | 0.991547 | 722.76 m | 1897.09 s | 44 | Native `way2_checks.csv`; stopped on 3rd consecutive valid confirmation. |

Important interpretation:

- `mpx_w2_001` demonstrates that the frozen early-stop branch can trigger correctly.
- `mpx_w2_002` demonstrates the complementary behavior: if the confirmation sequence is not completed before frontiers disappear, Way2 does not force a stop and baseline completion remains responsible for termination.
- `hpx_w2_001` and `hpx_w2_002` reached normal completion.
- `hpx_w2_003` triggered Way2 using the third-consecutive-valid branch with final recorded values approximately `R_t=0.107752`, `U_t=1.09924 cells/m`, `candidate_count=1`, `valid_count=3`.
- Five runs are not enough for a final thesis-level statistical conclusion. Continue collecting frozen-rule runs before judging Way2 against MapEx or NF.

---

# Online implementation and recorder status

## Way2 policy

Implementation is in:

```text
scripts/mapex_way2.py
```

Behavior:

- uses the same MapEx prediction, frontier extraction, candidate scoring, distance preference, near-frontier fallback, execution suppression, Nav2 execution, recovery, revalidation, and baseline completion machinery;
- evaluates Way2 only when a non-empty selectable frontier set `F_t` exists;
- recovery retries and non-evaluable no-frontier/no-selectable states do not increment or reset the Way2 confirmation count;
- when Way2 says CONTINUE, frontier selection remains the unchanged MapEx `argmax information_gain/distance_m` rule;
- when Way2 says STOP, no new Nav2 exploration goal is issued.

## Recorder

Recorder wrapper:

```text
scripts/mapex_way2_run.py
```

Recorder termination-reason bug found during `mpx_w2_001` sanity was fixed in commit:

```text
29ac7f0ca87093fc544f78a9f756a49b4a168052
```

That bug affected recorded metadata only; it did not affect the online stop timing or Nav2 behavior.

Per-decision Way2 audit history was added in commit:

```text
128dbb0c61d8acb96cf1eef8f9efc91d0912328e
```

New runs using this recorder write:

```text
way2_checks.csv
```

The Way2 history is buffered in RAM during the experiment and written once at `finalize()`, avoiding per-decision disk I/O in the benchmark critical path.

Each evaluable Way2 check records:

```text
decision_id
mapex_policy_decision_id
sim_time_s
R_t and threshold
U_t and threshold
candidate_count
base_valid
valid_count
action
should_stop
R_t argmax frontier: x/y/distance/IG/score/visible_unknown_cells
U_t argmax frontier: x/y/distance/IG/score/visible_unknown_cells
```

Existing per-decision replay artifacts already include:

```text
policy_decisions.csv
candidates.csv
decisions/*/observed_map_raw.npz
decisions/*/observed_map_canvas.npz
decision_maps/
predictions/
trajectory / goals / plans / metrics
```

This is sufficient for an offline visualization/video pipeline showing the map, robot pose, selectable frontiers, selected frontier, `R_t`, `U_t`, confirmation state, and CONTINUE/STOP reasoning for each decision. PNG/video rendering should remain offline rather than inside the running benchmark.

Runs recorded before `way2_checks.csv` was introduced can be backfilled from `candidates.csv` + `policy_decisions.csv` where the necessary candidate-level fields exist.

---

# Development evidence used to freeze Way2

The development backtest remains historical context only:

```text
New Room: trigger 11/15, bad observed-IoU-loss runs 0/11
  median time saved       = 8.21%
  median distance saved   = 6.29%
  worst observed IoU loss = 0.001605

Hospital: trigger 2/2, bad observed-IoU-loss runs 0/2
  median time saved       = 3.38%
  median distance saved   = 2.09%
  worst observed IoU loss = 0.003713
```

The final frozen candidate was chosen from a local robustness region rather than a single isolated parameter point:

```text
lambda / R threshold = 0.30
Udmax / U threshold  = 10.0 cells/m
candidate cutoff     = 1
confirmation         = 2 states if <=1 candidate at second confirmation,
                       otherwise 3 consecutive valid states
```

Do not use online validation outcomes to change these constants.

---

# Way1 — CLOSED

Frozen Way1 rule:

```text
unknown_variance_p95 <= 0.23, K=1
```

Prospective New Room validation `mpx_011...mpx_015`:

- trigger: 5/5
- mean time saved: 24.37%
- mean distance saved: 25.36%
- mean IoU loss vs analyzer final reference: -0.00201
- worst IoU loss: +0.000053

Hospital cross-environment diagnostic:

- `hpx_001`: no trigger
- `hpx_002`: no trigger
- P95 remains near `1/3`

Conclusion: Way1 works in New Room but does not transfer cleanly to Hospital without retuning, so it is closed as the primary direction.

---

# Evaluation caveats

- Existing New Room run summaries/CSV coverage were produced with `new_room_connected_free_v1`, whose structural geometry was in Gazebo world coordinates rather than the SLAM-start frame. Treat those absolute coverage values as superseded/pending v2 backfill; do not compare v1 New Room coverage numerically with Hospital coverage.
- The frame-correct `new_room_connected_free_v2` uses the launch spawn `(0,3,0)`, transforms SDF geometry into the SLAM-start frame, and seeds connected free space at SLAM `(0,0)`. Validate an overlay against a saved final canvas before bulk rewriting historical coverage.
- Historical development runs must not be reused as independent validation evidence.
- `mpx_w2_001` is an integration/sanity run, not a prospective validation sample.
- Current simulator seed policy is intentionally uncontrolled Gazebo default; comparisons should therefore use multiple-run statistics rather than assume exact paired seeds.
- Observed-only occupied IoU is useful for structural-quality auditing and is not an online stop input.
- Offline `final_occupied_iou` from the current evaluator is a prediction-map-vs-structural-GT metric and should not be compared numerically as if it were the same metric as historical observed-only IoU loss.
- Way2 runs currently use `prediction_source = mean_map`; older MapEx/NF evaluation records may use `alltrain`. Check evaluator semantics before cross-method IoU comparisons.
- TU remains weakly discriminative and is not a primary stop signal.
- Time and distance should be compared as distributions over repeated runs, not from a single run.

---

# Current next actions

1. Generate and visually validate `new_room_connected_free_v2` against a saved New Room final canvas (recommended sanity run: `mpx_w2_002`), then backfill historical New Room coverage/coverage curves from saved canvas snapshots before interpreting absolute coverage.
2. Collect additional frozen-rule New Room runs beginning with `mpx_w2_003` and additional Hospital runs as needed.
3. Keep pushing the **full run directories**, including decisions, prediction maps, maps, CSV/JSON metadata, goals/plans, and audit files, so replay and visualization remain possible.
4. Do **not** change `R<=0.30`, `U<=10`, candidate cutoff `1`, or the 2/3-decision confirmation rule during this collection stage.
5. Build a Way2 replay/visualization script that joins `observed_map_canvas.npz`, `policy_decisions.csv`, `candidates.csv`, and `way2_checks.csv` into per-decision annotated frames, then renders video offline.
6. After enough independent runs are collected, compare Way2 against:
   - the full comparable New Room MapEx baseline cohort `mpx_001...mpx_010`;
   - the corresponding Hospital MapEx baseline data;
   - NF cohorts where applicable;
   using time, distance, coverage/structural quality, completion behavior, and run-to-run variability.
7. Only after repeated frozen-rule evaluation decide whether Way2 is accepted or rejected as the thesis stopping rule.

## Analysis scripts

- `scripts/analyze_way2_utility.py`
- `scripts/audit_way2_gain.py`
- `scripts/compare_way2_gain_definitions.py`
- `scripts/audit_way2_cost.py`
- `scripts/analyze_way2_lambda.py`
- `scripts/inspect_way2_lambda_focus.py`
- `scripts/inspect_way2_rebounds.py`
- `scripts/compare_way2_guard_k.py`
- `scripts/compare_way2_guard_smoothing.py`
- `scripts/evaluate_way2_candidate_quality.py`
- `scripts/inspect_way2_quality_failures.py`
- `scripts/inspect_way2_hospital_quality_trajectory.py`
- `scripts/audit_way2_observed_quality.py`
- `scripts/sweep_way2_lambda_quality.py`
- `scripts/audit_way2_visible_unknown_guard.py`
- `scripts/sweep_way2_visible_unknown_guard.py`
- `scripts/inspect_way2_guard_candidates.py`
- `scripts/inspect_way2_mpx009_quality_trajectory.py`
- `scripts/evaluate_way2_adaptive_confirmation.py`
- `scripts/sweep_way2_adaptive_candidate_count.py`
- `scripts/sweep_way2_adaptive_rule_neighborhood.py`
