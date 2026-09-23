# R003 — Approved New Room paper1000 protocol

Status: USER-APPROVED / CONSTRUCTION CONTRACT FROZEN on 2026-09-23.
Approval evidence: USER replied "duyệt" to WORK's proposal in this session.
Approved source: vah103/chat-gpt/project_management/mapex/R003_METHOD_PROPOSAL_V1.md, blob 2ea77ba214a3fe7dd6efc2b578d83fbff201ebeb, proposal commit 58166e0c0a6af5574f137a468b246c24ecff64e9.
Profile: new_room_mapex_paper1000_v1; budget: odom_progress_0p30m_v1.

## Current authority and next gate

The numbered proposal below is retained verbatim as the approved specification. Its historical statements "proposed", "pending USER approval", and "not canonical" describe the pre-approval snapshot and are superseded by this header. Technical semantics are unchanged by this sync.

CODEX may implement and test this contract. Generated masks/counts/hash/visual verification remain required before smoke; WORK acceptance of 1 NF + 1 MapEx smoke remains required before 10+10. No execution result is implied by approval. Scope is simulation New Room only. WORK remains manager; CODEX remains engineering owner.

Historical Hospital EXPERIMENT_PROTOCOL.md remains applicable to Hospital. This file is the authoritative R003 New Room profile supplement; record effective current runtime configuration as specified below.

## USER-approved amendment A3 — physical-deadlock watchdog for unattended bulk

Approved by USER on 2026-09-23 before R003 bulk execution.

A distinct **technical deadlock** class is added for unattended COM1 bulk runs.
This is narrower than an ordinary algorithmic/navigation failure.

Technical-deadlock intent:
- the robot becomes physically wedged/stuck (for example after trying to enter a
  passage that is too narrow);
- an exploration/navigation action remains active or repeatedly attempts to
  continue;
- odometry shows no meaningful translational progress for a preregistered
  watchdog interval;
- the run would otherwise remain stuck indefinitely rather than reaching a
  normal protocol terminal outcome.

When the frozen watchdog declares this technical deadlock:
1. stop the run and clean up ROS/Gazebo normally;
2. checkpoint the run ID, host, source SHA, watchdog evidence and invalidity
   reason;
3. classify the attempt as technically/infrastructure invalid;
4. delete that invalid run directory;
5. rerun with the **same official run ID** under A2.

This rule must not convert ordinary algorithmic outcomes into retryable
infrastructure failures. In particular:
- a Nav2/action failure that terminates normally under the existing policy is an
  official algorithmic/navigation outcome;
- poor metric values are never a retry reason;
- natural completion is never a retry reason;
- a finite recoverable stall is not a retry reason.

Before any 002..010 bulk run, the watchdog detector and its exact quantitative
criteria (progress measure, tolerance, time window, action-state requirement,
and reset conditions) must be explicitly frozen and independently reviewed.
Codex must not use subjective visual judgment or change the watchdog threshold
after seeing run metrics.

## USER-approved amendment A2 — invalid technical attempt reuses the official run ID

Approved by USER on 2026-09-23 before any accepted R003 official run-1 result.

This amendment supersedes the A1 retry-ID rule only.

For an attempt that is determined to be **technically/infrastructure invalid**
(for example wrong launch/setup, broken capture/provenance, corrupted runtime,
or another condition that means the attempt is not a valid execution of the
frozen protocol):

- stop and diagnose the attempt before rerunning;
- record the invalidity reason in the management/runtime checkpoint;
- the invalid run directory may then be deleted;
- rerun using the **same official run ID**, e.g. `nf_p1000_001` again rather
  than `nf_p1000_001_retry01`;
- the official ID always refers to the valid attempt ultimately retained for
  that slot.

This does **not** permit performance-based replacement. A protocol-valid run
with poor Coverage/IoU/TU/AUC, natural early completion, or an algorithmic /
navigation failure remains an official outcome and must not be deleted merely
to obtain a better result.

The decision to delete/retry must be based on technical validity, not metric
quality. The reason for each deleted invalid attempt must be checkpointed before
deletion so the intervention remains auditable even though the raw invalid run
directory is removed.

## USER-approved amendment A1 — qualification runs may become official run 1

Approved by USER on 2026-09-23 before any R003 ROS/Gazebo runtime attempt under this profile.

This amendment changes only the run-sequencing/inclusion rule. GT, valid-space,
budget, sampling, prediction, metric, threshold, TU, AUC, and aggregation
semantics are unchanged.

The first NF and first MapEx runtime attempts are now **qualification attempts
for official run 1**, rather than disposable smoke-only runs:

- first IDs are `nf_p1000_001` and `mpx_p1000_001`;
- if an attempt is **runtime-valid** under the preregistered integrity/provenance
  checks, it is retained as official run 1 regardless of metric quality,
  completion time, or whether the algorithm succeeds or fails;
- an algorithmic/navigation failure is an official outcome and must not be
  discarded or replaced merely because performance is poor;
- only an **infrastructure-invalid** attempt (for example launch/runtime
  corruption, missing required data, broken capture/provenance, or evaluator
  impossibility caused by infrastructure) may be rerun for the same official
  slot;
- A2 supersedes the historical new-retry-ID wording: after the invalidity reason
  is checkpointed, the invalid run directory may be deleted and the same
  official run ID reused;
- acceptance/retry decisions must be based only on preregistered runtime
  integrity criteria, never on Coverage/IoU/TU/AUC values;
- the independent checker reviews the qualification evidence before runs 2..10
  proceed;
- if both first attempts are accepted, the remaining bulk is **9 NF + 9 MapEx**
  official attempts, completing 10+10 total.

This amendment supersedes the historical section-9 wording that required a
separate disposable smoke namespace followed by a fresh 10+10 namespace.
The historical proposal text below is retained for audit.

---

# R003-METHOD — New Room paper1000 proposal v1

Date: 2026-09-23
Owner/session role: WORK
Status: COMPLETE_PENDING_REVIEW — proposed contract, NOT USER-approved or canonical.
Parent: R003. Next engineering owner: CODEX/W009 after approval.
Scope: New Room only; 1 NF + 1 MapEx smoke, then separately reviewed 10+10.

## 1. Decision requested

Approve a ROS **distance-progress adaptation** of MapEx's simulator horizon, with:
- profile: `new_room_mapex_paper1000_v1`;
- budget semantics: `odom_progress_0p30m_v1`;
- step k = floor(accumulated odometry translation / 0.30 m), maximum 1000;
- a separate new structural GT / valid-space profile at 0.10 m;
- the same all-training LaMa evaluator for NF and MapEx.

This is a proposed controlled ROS comparison. It is not an exact recreation of the paper's simulator or a basis for expecting the same absolute scores. USER approval freezes this construction contract; generated raster/hash and smoke acceptance remain later gates.

## 2. Source audit

Pinned original: `castacks/MapEx@53636bd1c79153acc3c74a532837d78c926bae5e`.
- `configs/base.yaml`: mission_time=1000; pixel_per_meter=10.
- `scripts/explore.py:263-266,294,465-480`: initial observation, 1000 loop iterations, A* with allow_diagonal=False, controller index 3.
- `scripts/sim_utils.py:21-27`: selected path index=min(3,len(path)-1). Hence a normal iteration advances up to three cardinal grid edges, nominally up to 0.30 m. Short endpoint moves also consume a loop iteration.
- `explore.py:718-732`: observed map saved BEFORE next motion/observation. Files indexed 0..999 are not automatically equivalent to an after-1000-motion state.
- `sim_utils.py:29-60`: occupancy 2x2 min, padding occupied; valid-space 2x2 max, padding invalid.
- `calc_metrics_subdirectory.py:96-111,142-158`: known free OR occupied / valid-space; IoU is occupied-class with unknown prediction treated non-occupied.
- `topological.py`: random.sample without replacement, 100 valid-space goals, fixed initial start per map/start condition, 4-neighbour A*. Original catches exceptions without counting them; R003 must instead report evaluation errors and must never silently shrink denominator.
- `explore.py` loads a whole-training predictor separately from policy ensemble; current project `predict_alltrain_offline.py` likewise supports NF and MapEx.

Paper: https://arxiv.org/abs/2409.15590 (v2 reviewed). Its idealized setup assumes known pose/noise-free ranges; current ROS SLAM is a further adaptation.

Technical prerequisites inspected on main:
- `mapex_lab/docs/MAPEX_COMPATIBLE_GT_SPEC.md` blob e1e89660ca2b6af556ff92243e03507b919812b3.
- `ground_truth/new_room/structural_gt_v2.yaml` (read on main; full implementation revision must be pinned in the manifest).
- `scripts/generate_new_room_ground_truth.py`, `scripts/_nf_run_core.py`, `scripts/nf_run.py`, `launch/slam.launch.py`, `scripts/predict_alltrain_offline.py`.
- Current launcher overrides runtime resolution to 0.10 m, max SLAM range to 12 m, map publication interval to 1 s, and enables adaptive anchor. The older Hospital protocol says 20 m range and is not an exact New Room runtime specification. Record effective runtime parameters rather than copy those historical values.

## 3. Budget semantics

Start accumulation at first policy decision BEFORE compute, once map/TF/Nav2 readiness passes; retain the latest pre-start odometry pose as the origin.
- D accumulates every finite, time-ordered /odom XY increment using Euclidean norm in odom frame.
- Count all translation: forward, backward, recovery, revisits. Rotation in place adds zero.
- Do not derive distance from map-frame pose, planned path length or selected-frontier distance.
- k=min(1000,floor(D/0.30)); retain continuous D and residual.
- Planning/paused/stationary time adds no steps; simulation and wall time are still logged.
- Carry residual across goal changes and recovery. No per-goal rounding or artificial increment on a decision.
- Reject nonfinite odometry, time reversal/frame change; log odometry reset/gaps as integrity faults. Do not silently drop a reset and certify the run.
- A budget observer must remain responsive during inference/scoring. A single callback thread blocked by model inference is insufficient.
- On first crossing D>=300 m, latch cutoff timestamp/distance and the latest map received at or before cutoff; prevent further exploration dispatch and request normal action cancellation through the shared execution adapter. Save post-cancellation map separately.
- Cutoff metrics use the latched map; do not substitute a later map that benefited from extra motion/observation.
- Log overshoot at detection and after cancellation, odom timestamps, map age. For smoke acceptance: detection overshoot <=0.10 m and cutoff map age <=2 sim seconds. These are proposed engineering acceptance limits, not paper constants.
- Missing valid cutoff capture means invalid budget evidence and blocks bulk acceptance.

Meaning: the ROS adaptation gives each method 300 m of accumulated translation at its budget cap. This differs from the original loop because short endpoint steps, grid-constrained motion, continuous LiDAR, and planning waits behave differently. The original is NOT officially a fixed 300 m stop rule. Axis label must say "ROS adapted progress step (0.30 m)" and also expose distance/time.

Alternative considered: fixed seconds is simpler but changes the resource from movement to time. Exact replay of original loop requires replacing the ROS execution/sensing loop and is beyond this bounded task. Do not disguise either as the present proposal.

## 4. Termination / inclusion

Codes:
- `budget_1000_reached`: cutoff latched before natural completion;
- `exploration_complete_before_budget`: existing stable completion predicate finishes at k<1000;
- `navigation_failure`: unrecovered execution failure;
- `abnormal_termination`: interrupt, crash, missing data, watchdog.

Use current stable completion/revalidation logic shared by NF and MapEx. Remaining near frontiers alone do not imply completion. All proposed stop rules are simulation-only here.

Log every attempted run. Infrastructure-invalid attempts can be rerun under a new ID with documented reason; retain their records. Algorithmic failure is reported, never silently replaced to obtain 10 successful runs. 10+10 means ten official attempts per method under the approved contract; report completed, early-complete and failed counts. Failure curves stop at failure and show available n; no post-failure hold.

## 5. GT / valid space / coordinates

New IDs:
- `new_room_mapex_structural_v1`;
- `new_room_mapex_valid_space_v1`;
- `new_room_mapex_eval_010_v1`.

Geometry construction: source `mapex_lab/map/new_room.sdf`, model mini_hospital_structure, collision boxes crossing world z=0.20 m; cell-centre rasterization. Spawn world (0,3,0); transform R(-yaw)*(world-spawn). Start in evaluation coordinates is (0,0).

Source canvas: 1504 columns x 2123 rows, resolution 0.05 m, origin (-25.6,-60.1), inherited geometry from hospital_canvas_v1 despite the legacy name.
- Occupancy derives from structural collisions.
- Candidate valid-space = existing structural evaluation footprint: axis-aligned outer bounds of transformed collision geometry. Includes structural interiors; never filter with connected-free.
- This bounding footprint is a proposed New Room analogue, not proof of identity with KTH valid-space annotation.
- Before smoke, render GT/valid-space overlays against SDF and a saved observed map. Confirm footprint includes intended building only, alignment and start pose. If shape/footprint is wrong, return to WORK for versioned correction before freezing hashes.

2x2 blocks anchored at source row/col (0,0). Pad odd final row to even height (2124) with occupied=1 and valid=0; width already even.
- occupancy reduced by any occupied (equivalent min for original labels);
- valid reduced by any valid;
- output 752 columns x 1062 rows, 0.10 m, same lower-left origin.
- Explicitly mask source cells outside structural footprint before scoring; they must not become GT free by default.
- Freeze separate support E10=any(source evaluation footprint) and V10=any(source valid). They coincide for this proposal, but remain distinct fields/contracts.
- Score IoU on E10; Coverage on V10. Fixed domain cropping is a documented ROS adaptation to avoid the enormous unrelated logging canvas.
- TU search domain E10; outside E10 blocked, a documented adaptation preventing routes through unscored exterior.

Manifest must contain full git commit, source/config SHA-256, frame, transform, dimensions, masks/hashes, source/reduced counts (occupied/free/valid/valid-and-occupied), start cell and generation command. Verify derived odd-row padding does not unexpectedly intersect New Room valid support.

## 6. Observed and prediction projection

Use immutable raw map at each sample, with origin/resolution/yaw and source stamp.
- Project runtime observed map onto 0.05 m canonical cell centres; outside runtime support unknown. Axis-aligned mapping only; fail unsupported rotation instead of guessing.
- Reduced observed block: occupied if any source cell occupied (>0); otherwise free if any source cell known-free (=0); otherwise unknown. Coverage known = any known subcell, a declared ROS aggregation choice.
- Alltrain prediction: preserve probability, remove model padding using explicit offsets, project to source canvas, then max-pool occupancy probability in 2x2. Unrepresented cells get 0/non-occupied with their support fraction reported; nonfinite outputs are errors.
- Threshold is strictly p>0.5 after reduction. No threshold tuning.
- Model input maps observed >0 to occupied, =0 to free, <0 to unknown, consistent with existing alltrain loader.
- Log projection/support and ensure identical transforms for both methods.

## 7. Metric definitions

Coverage=count(V10 & observed_known)/count(V10).
Known fraction=count(observed_known)/size(full fixed reduced canvas); report denominator.

Occupied IoU=TP/(TP+FP+FN) within E10. Empty union -> 0, matching original implementation; log empty-union flag.
Primary prediction source for BOTH methods = same offline alltrain checkpoint/config/preprocessing and hashes. Policy ensemble remains the MapEx policy input. Observed-only or ensemble-mean diagnostics must have distinct names.

TU:
- sample 100 unique cells uniformly without replacement from V10, deterministic seed=3001; save coordinates (authoritative), RNG/library version, mask and goal SHA.
- fixed path start = evaluation cell containing initial SLAM (0,0), NOT current moving robot position.
- no resampling occupied/unreachable goals. Count and report these cases.
- uniform-cost 4-neighbour A* over predicted non-occupied cells within E10; no dilation.
- start/goal predicted occupied or no path => failure; any path cell GT occupied => failure; otherwise success.
- TU=success/100. A* exceptions or missing/nonfinite prediction => evaluation error, not a smaller denominator.
- Freeze A* implementation/version and neighbour order; multiple equally short paths can affect collision verdict.
- Validate GT start is free and E10 contains it; otherwise profile construction fails.

## 8. Common measurement samples

Capture raw initial map, each k=10,20,...,1000 crossing, and natural-completion/cutoff final map. Each record joins step, distance, receive time, source map stamp and hash. If multiple crossings occur between callbacks, record same available map with explicit repeated-sample flag; do not invent intermediate observations.
Compute alltrain/IoU/TU offline on these common samples for BOTH methods. Deduplicate inference by raw-map+model+transform hash. Keep policy-decision maps/timing separately.
The present alltrain script only consumes decision logs: CODEX must add support for snapshot manifests and the latched final sample. Its current last decision is not guaranteed to equal budget final.
Curves use recorded samples at common step support, no future-map interpolation. Only legitimate natural completion may hold its final metric to 1000; mark every held row. Report raw and held support counts.

## 9. Implementation / smoke / bulk handoff

After USER approval:
1. CODEX creates isolated profile/evaluator/budget instrumentation and focused tests.
2. Tests: transforms, occupied+valid overlap, odd row, strict 0.5, unknown support, budget residual and recovery, callback cutoff during scoring, early-completion vs budget priority, TU goal reuse/start/collision/no-path/error, final-snapshot selection.
3. Generate candidate masks/manifests and visual audits; WORK reviews before smoke.
4. Simulation 1 NF + 1 MapEx smoke in separate smoke namespace. Verify actual launch world/spawn, runtime params, model/checkpoint, 100 goals, cutoff responsiveness, map freshness, outcome and evaluator parity.
5. WORK reviews smoke. Only then 10 NF + 10 MapEx in new namespace nf_p1000_001..010 / mpx_p1000_001..010.
6. Alternate method order across repetitions; simulator seed remains explicitly uncontrolled unless separately changed before freeze.
7. Aggregate run-macro mean ± sample std (ddof=1), counts/failures and **full common-step metric curves**. No pixel-pooled substitute for run-macro.
8. Curve-first comparison is required:
   - Coverage(t), occupied-IoU(t), and TU(t) over the common adapted-step axis are primary outputs;
   - compute per-run Coverage AUC and occupied-IoU AUC, then report run-macro mean ± sample std;
   - compute TU AUC as an additional project-side scalar summary of the TU curve, clearly labeled as such unless an original-paper TU-AUC statement is separately verified;
   - `Coverage@1000`, `IoU@1000`, and `TU@1000` are secondary endpoint summaries only and must not replace curve/AUC analysis;
   - use deterministic trapezoidal integration on the recorded common-step samples and record support/integration provenance;
   - legitimate natural completion may use explicit post-completion hold; algorithmic failure may not.
9. WORK approves aggregates/labels before appending P1000 sheets to the current Experimental Results.xlsx.

Record effective New Room launcher, SLAM parameters, Nav2 merged YAML, policy/config/checkpoint, simulator version and clean technical commit. All code/config/grid choices identical across methods except selection policy.
Old v2 results and sheets retained. Hospital, D1 threshold work and paused R002 fairness investigation remain outside this scope.

### Curve/AUC reporting contract

The R003 result package must preserve MapEx's curve-oriented evaluation style.

Required canonical outputs:
- per-run Coverage(t), occupied-IoU(t), TU(t);
- aggregate mean ± sample std curves across official runs at common adapted-step support;
- per-run and aggregate Coverage AUC;
- per-run and aggregate occupied-IoU AUC;
- per-run and aggregate TU AUC as an explicitly project-added summary scalar;
- secondary endpoint summaries at budget/legitimate early completion.

Do not present a single final metric as the primary performance comparison. If a curve row is held after legitimate natural completion, mark it. If a run terminates by algorithmic failure, stop support at failure and report available n rather than holding the last value.

## 10. Completion / limits / routing

Routing: session WORK; role file agents/WORK.md; assigned R003-METHOD; board owner WORK; SELF.
Validation: read-only source audit and contract consistency review performed. No implementation, mask generation, local ROS test, smoke, bulk or workbook result claimed.
Local environment inspected earlier is isolated from /home/dell/turtlebot4_project; no robot/session access established.
Usage: unknown remaining quota; restored availability per USER; recommended Astra Low, escalate only on unresolved methodology conflict. No subagents used.
Canonical impact now: MANAGEMENT_ONLY. If approved, MAPEX_LAB_SYNC_REQUIRED before canonical use.
Next: USER reviews this explicit adaptation; WORK records approval and canonical protocol handoff; CODEX then implements. R003 remains ACTIVE.
