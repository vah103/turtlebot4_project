# mapex_lab status

## MX071 R2 - DELL offline 2D pilot (2026-10-09)

- DONE: isolated PIPE/pyastar2d/range_libc dependencies; sealed maps, two starts each, and three model identities; source parity, complete-state controller replay, scientific contract checks, and full-size CPU feasibility.
- LATEST RESULT: first map `50052750` completed 2/8 source trajectories and 20 logical first-goal requests. Both sources used the 100 m budget. Six distinct snapshots cover seven target slots; one slot is retained as NA. A single map cannot satisfy the direction screening criteria.
- IN PROGRESS: Stage 1 resumed on DELL from completed outputs after an ASCII locale error in Markdown report writing. The same sealed Python code now runs with an explicit UTF-8 locale; protected completed source/snapshot/branch hashes are unchanged.
- NEXT ACTION: finish the remaining six fixed source trajectories and their bounded paired requests, verify final integrity, and interpret only the complete preregistered support. Stage 2 remains CLOSED.
- EVIDENCE: [runner and deployment notes](pilots/mx071_r2/README.md); [UTF-8 recovery ledger](pilots/mx071_r2/evidence/locale_utf8_resume_20261009.json). Heavy artifacts remain on DELL under `/home/dell/mx071_dell_20261009/artifacts/stage1`.
- BOUNDARY: engineering checkpoint only; H1/H4/H7 screening is incomplete. MAP_ID_ONLY / TRAIN_OVERLAP_UNVERIFIED. No result acceptance, online STOP, robot motion, or MX072 execution is claimed.

## D1 retrospective oracle STOP — W032 / H080 / H083

- REVIEWED / CANONICAL: same-checker W029 ACCEPT on exact PR #33 head `4f674a81add0c14eb15cc0ab95754130c5923ead`; the oracle lineage is now included in canonical technical main through MX008.
- TESTS: revised focused suite 29/29 PASS; 10/10 runs and 365 decision rows preserved; persistent OracleStop_1/5/10 decisions unchanged.
- RESULT: footprint-aware structural-GT oracle is accepted as retrospective/hindsight evidence only.
- BOUNDARY: this is not an operational STOP rule, uncertainty threshold, Gate-U result, Gate-R PASS, or `K_confirm`.

## D1 Gate R historical scoring — W034 / H084 / H085

- REVIEWED / CANONICAL: same-checker W029 ACCEPT on exact PR #34 head `e0e85b384564b03178f87b598daca9b476ebd644`; PR #34 includes the accepted PR #33 oracle ancestry and is now integrated into canonical technical main.
- TESTS: 28/28 focused tests PASS; 10 folds / 10 runs / 365 decisions preserved.
- HISTORICAL RESULT: `HISTORICAL_NEW_ROOM_FEASIBILITY_SUPPORTED_PENDING_CONFIRMATION`.
- BOUNDARY: this is **not full Gate-R PASS**. Gate U remains **FAIL — strong broad contradiction veto**. No operational remaining-area threshold, `K_confirm`, U×R online rule, confirmation scoring, or online STOP is authorized.

## D1 Gate U offline scoring — W027 / H074

- REVIEWED / CANONICAL: W016 same-checker ACCEPT on exact revised delivery `97b16e52048a4cfc47b808103cf25e7737e938e1`; PR #31 merged unchanged into canonical main at merge commit `34bc65817d7eb7a42a5ff3149a860748c91fa489`.
- TESTS: 27/27 focused tests PASS after bounded revision; first-fold production smoke PASS; full resume revalidated/skipped all 10 folds including both fold-summary and per-decision hashes.
- CANONICAL HISTORICAL RESULT: Gate-U = `FAIL` because the frozen strong broad-contradiction veto fired in exactly 6/10 held-out runs: `mpx_002, mpx_003, mpx_006, mpx_007, mpx_009, mpx_010`. General primary sufficiency passed; all folds selected `U_p95`; strict-late remains `LATE_PRIMARY_INSUFFICIENT`.
- BOUNDARY: Gate-U remains FAIL. A separately approved historical Gate-R diagnostic is now ACCEPTED, but it is not full Gate-R PASS and does not rescue Gate-U FAIL. Operational uncertainty/area/remaining-fraction thresholds, `K_confirm`, U×R online logic and online STOP remain unauthorized.

## D1 shared Phase-0 extractor — H071 / H072 delivery

- DONE (engineering): neutral evidence extraction on 10 historical runs; 365 unique rows; 18/18 focused tests PASS; validated resume skips all ten completed units.
- IN PROGRESS: independent W016 extraction-integrity review; completion is not yet ACCEPTED.
- LATEST RESULT: 218 runtime/U-evaluable rows; all 147 non-evaluable rows retained with reasons/NaN. Accepted R004 imported fields preserved exactly.
- NEXT ACTION: Chat 2 reads `docs/D1_W025_EXTRACTOR_COMPLETION.md` on branch `d1-shared-phase0-w025`. Gate-U scoring remains PARKED. H071/H072 supersede the historical inactive-preparation wording below only for neutral extraction.

## R003 — New Room paper500: implementation ready for independent review

USER reset the official R003 horizon on 2026-09-23. Canonical contract:
[R003_PAPER500_PROTOCOL_V1.md](docs/R003_PAPER500_PROTOCOL_V1.md).

- Profile: `new_room_mapex_paper500_v1`; one ROS adapted progress step = 0.30 m accumulated odometry, maximum 500 (150.0 m).
- Versioned structural GT/valid-space at 0.10 m; common alltrain IoU/TU evaluation for NF and MapEx.
- CODEX implemented the paper500 budget/evaluator/provenance path, public
  `--paper500` runner mode, and resume-safe 20-run batch/watchdog on branch
  `r003-paper500-h031`.
- Profile masks and 100 fixed TU goals remain unchanged; paper500 artifacts are
  under `ground_truth/new_room/generated/r003_paper500/`.
- Official cohort is `nf_p500_001..010` + `mpx_p500_001..010`.
- Current gate: H030 Stage-B independent review of the exact branch SHA. No
  official paper500 run or workbook update has been performed.
- Existing historical metrics remain auditable. R002 fairness investigation remains paused.


_Last synchronized with `main`: 2026-09-22._

## Current focus

The active thesis program is now **six MapEx early-stopping research directions (D1...D6)** defined canonically in:

```text
references/ES.md
```

Way1 and Way2 are both **closed historical branches**.

- **Way1:** closed because the global unknown-variance threshold did not transfer robustly from New Room to Hospital without retuning.
- **Way2:** closed as the primary research direction because the collected result was not sufficiently robust/compelling to justify continuing it as the thesis stopping rule.
- Existing Way1/Way2 code, runs, logs and frozen constants remain in the repo for audit/history only.
- Do **not** retune Way1 or Way2 and do not reuse their names for the new directions.
- Baseline `scripts/mapex.py` remains unchanged and is the comparison baseline for D1...D6.

The six active directions are:

1. **D1 — Uncertainty-Aware Predicted Map Completeness**
2. **D2 — Information-Gain Saturation**
3. **D3 — Prediction + Stagnation Hybrid**
4. **D4 — Prediction Reliability-Gated Stopping**
5. **D5 — Ensemble Risk / Missing-Area Consensus**
6. **D6 — Lightweight Learned Stop Predictor**

Implementation order is controlled rather than winner-first:

```text
common stopping infrastructure
→ D2 plumbing/baseline
→ D1 first major prediction-aware method
→ D3/D4 robustness extensions
→ D5 ensemble-consensus extension
→ D6 learned extension
```

No D1...D6 threshold is frozen yet. Threshold selection must use development data only; prospective validation begins only after each direction's parameter set is explicitly frozen.


## D1 Gate P status

**COMPLETE — DUAL CANONICAL STATE: P-direct FAIL / P-task PASS_TO_GATE_U**

- Gate P1 `later_observed` remains a **PRELIMINARY PASS**, but its late-stage evidence is heavily right-censored.
- Gate P2 `structural_gt` has now been executed on the full `mpx_001...mpx_010` cohort with `--reference both`, ensemble-mean prediction and threshold `0.5`.
- Smoke test and full 10-run execution both completed with exit code 0.
- P2 evaluated 365 decisions and passed the implemented prediction/raw-map identity and provenance hard checks.

**LATEST RESULT**
- P1 ensemble-mean run-macro accuracy ≈ **0.8889** overall and ≈ **0.9079** over last-10 evaluable decisions.
- P2 ensemble-mean run-macro accuracy ≈ **0.7120** overall and ≈ **0.5573** over the last 10 decisions.
- P2 macro IoU ≈ **0.4688** overall and ≈ **0.2817** over the last 10 decisions.
- P2 MAE ≈ **0.2893** overall and ≈ **0.4327** over the last 10 decisions.
- Most important for direct D1 completeness: last-10 P2 free recall ≈ **0.1360** and free IoU ≈ **0.0076**.
- Late P2 support is strongly occupied-dominated, so class-specific free-space metrics are more informative for D1 than headline accuracy.

**INTERPRETATION**
- Gate P2 provides **strong negative evidence for late-stage free-space fidelity** over the broader structural unknown region.
- The positive P1 result does not generalize to the region D1 would need for direct predicted-remaining-free-space completeness.
- Under the current direct formulation, Gate P therefore **does not pass**.
- Preserve this as a negative result; do not tune it away.

**AMENDED CANONICAL STATE — 2026-09-24**
- **P-direct / broad fidelity = FAIL / DOES NOT PASS.** The historical P1/P2 negative evidence above is preserved unchanged.
- **P-task / D1 task-aligned continuation = PASS_TO_GATE_U only.** Independently reviewed R004 evidence is sufficient to continue Phase-0 evaluation on robot-reachable remaining-free-space semantics.
- The task-aligned continuation is deliberately narrow: late source-based ReachableFutureFreeRetention is 1.0 but only **n=2** runs contribute; earlier/middle topology losses remain material.
- Prediction is **not** validated as a standalone STOP oracle.

**CURRENT DECISION**
- Gate U: **ELIGIBLE FOR SEPARATE USER ACTIVATION — NOT ACTIVE**.
- Gate R: **PARKED**.
- Threshold selection, `K_confirm`, online D1 and rule development: **BLOCKED**.
- Canonical feasibility order remains `Gate P → Gate U → Gate R`.

**NEXT ACTION**
- Only a separate USER activation may open the shared evidence contract / Gate-U methodology path.
- Do not run Gate U, select thresholds, choose `K_confirm`, implement online D1 or start Gate R merely because this Gate P wording is synchronized.

Canonical evidence and full numerical detail are recorded in:
- `mapex_lab/analysis/d1/D1_RESEARCH_LOG.md`
- result commit `e14807bbd66bc26f437b2b31c4c40b498cc239df`
- research-log commit `564af0c7c81e501f5e1fb7def669a105e260a2c0`


## D1 R002 task-aligned GT-semantics diagnostic

**IMPLEMENTATION REVIEWED / ACCEPTED — ONE-RUN EVIDENCE ONLY**

R002 is a post-hoc diagnostic/reformulation motivated by the negative Gate P2 result. It does **not** replace Reference A or erase the existing Gate P2 finding.

Canonical implementation:
- `analysis/d1/r002_gt_semantics.py`
- `analysis/d1/test_r002_gt_semantics.py`
- accepted source commit: `feb94eaa9c1ba5aa4f9993792dae454bc4edcd60`
- independent implementation review: **ACCEPT**
- pre-score validation: Python compile PASS, **12/12 unit tests PASS**

Frozen semantics preserved:
- prediction threshold = `0.5`;
- Reference B primary tolerance = `0.10 m`, sensitivities = `0.05 m` and exact;
- deterministic maximum-cardinality one-to-one B matching, then minimum total Euclidean distance, then lexicographic tie-break;
- C0 canonical topology grid = `0.05 m`;
- C0 connectivity = 4-neighbor;
- C0 seed fallback = observed-known-free only, Euclidean radius <= `0.10 m`;
- `U_t = Unknown_t ∩ StructuralGT.evaluation_mask`;
- `C0_TopologyDomain_t = StructuralGT.evaluation_mask ∩ DecisionMapSupport_t`.

**FIRST EXECUTION — `mpx_001` ONLY**
- exit code: `0`;
- decisions: `35/35`;
- Gate-P / DecisionMapSupport parity: `True`;
- topology-invalid decisions: `0`;
- preregistered overlays: decisions `6 / 18 / 29`.

Key one-run metrics:
- overall A free IoU ≈ **0.5001**;
- overall B F1 @ 0.10 m ≈ **0.4178**;
- overall C0 free IoU ≈ **0.5065**;
- overall C0 signed area error ≈ **-5.9140 m²**;
- late C0 free IoU ≈ **0.0445**;
- last-10 C0 free IoU ≈ **0.0335**;
- last-10 B F1 @ 0.10 m ≈ **0.3725**.

Interpretation:
- the one-run evidence is **mixed/negative late**;
- R002 confirms that a large share of Reference-A false-free error lies in structural interior, but C0 classification quality still collapses late;
- this does **not** rescue the current direct D1 stopping formulation;
- do not tune methodology using `mpx_001` while retaining it as confirmatory data;
- do not automatically expand to `mpx_002...mpx_010`; further expansion requires an explicit USER/WORK decision.

Detailed chronology and reproducibility notes are in:
- `analysis/d1/D1_RESEARCH_LOG.md`
- `analysis/d1/README.md`.

### Metric-calibration caveat — 2026-09-22

USER ran the existing observed-only structural audit on final `mpx_001`:

- final observed occupied IoU vs `new_room_structural_gt_v2` = **0.3886051561**;
- final coverage = **0.998540146**;
- decisions 31..35 returned the same final observed IoU/map state.

Current conclusion: raw pixel-wise structural occupied IoU remains useful as a strict diagnostic, but should **not be used alone as an absolute claim that MapEx prediction is good or bad** until the metric is calibrated against observed-map reproducibility. This is a limitation on interpretation, not a reversal of Gate P2 or Reference A.

Further calibration is **paused by USER**. Do not automatically run a Reference-B final-observed "ceiling" test or multi-run expansion.

**NEXT ACTION**
- preserve the reviewed R002 implementation and one-run result as canonical technical evidence;
- USER/WORK decides whether to authorize a frozen confirmatory expansion or stop/reframe D1;
- Gate U remains blocked under the current direct D1 formulation unless a separately approved revised direction is defined.

---

# Archived Way2 online runs

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
- These five runs are retained as historical evidence only. No further Way2 collection is planned under the new six-direction program.

---

# Archived Way2 implementation and recorder status

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

# Archived development evidence used to freeze Way2

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
- New `mpx_001` calibration evidence: the nearly-complete final observed SLAM map (final coverage `0.998540146`) scores only **0.3886051561 occupied IoU** against `new_room_structural_gt_v2` under the existing observed-only audit. Therefore raw structural-GT occupied IoU is not yet calibrated well enough to be used alone as an absolute prediction-quality verdict; it may mix prediction error with SLAM/alignment/discretization/wall-thickness/GT-semantic mismatch. This does not invalidate Reference A or erase Gate P2.
- Offline `final_occupied_iou` from the current evaluator is a prediction-map-vs-structural-GT metric and should not be compared numerically as if it were the same metric as historical observed-only IoU loss.
- Way2 runs currently use `prediction_source = mean_map`; older MapEx/NF evaluation records may use `alltrain`. Check evaluator semantics before cross-method IoU comparisons.
- TU remains weakly discriminative and is not a primary stop signal.
- Time and distance should be compared as distributions over repeated runs, not from a single run.

---

# Current next actions

1. Treat `references/ES.md` as the canonical implementation specification for D1...D6, while respecting the newer Gate P2 evidence recorded above.
2. The focused D1 diagnostic has now identified a structural-IoU calibration caveat: final observed `mpx_001` itself scores only `0.3886051561` occupied IoU vs structural GT despite `0.998540146` final coverage. Further calibration (for example final-observed Reference-B boundary F1 and multi-run replication) is **paused by USER**; do not start it automatically. Gate U remains blocked for the current direct-D1 formulation.
3. Shared stopping infrastructure and D2 plumbing may continue where they are independent of the failed direct D1 free-space assumption.
4. Do not implement the current direct D1 remaining-free stopping rule unless the diagnostic reveals a concrete evaluation artifact or USER approves a revised formulation.
5. Use development data to choose and freeze any surviving/revised direction's thresholds before prospective validation. Do not reuse frozen Way1/Way2 constants by default.
6. Add D3/D4 only after their required inputs and methodology are explicitly defined and approved; do not silently reinterpret the failed D1 signal as a new canonical method.
7. Add D5 using the current **three-member consensus rule**; do not describe it as a calibrated 5% tail probability.
8. Build D6 only after D1-D5 feature logs exist and evaluator semantics for labels are reconciled.
9. Preserve `scripts/mapex.py` unchanged as the baseline throughout this program.

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
