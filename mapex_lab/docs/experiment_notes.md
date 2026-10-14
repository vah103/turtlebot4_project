# Experiment Notes

## 2026-10-14 — Work1 single-owner COM1 R1: terminal correctness kiểm thử, native chưa chạy

- **What changed:** một sibling chuẩn bị native MapEx 2D, [exact 7421e0168837d7bdd18c05dcd854aebb4f440334](https://github.com/vah103/chat-gpt/blob/7421e0168837d7bdd18c05dcd854aebb4f440334/company/projects/mapex/mx072_com1_work1_ready_for_review_r1_20261014/README.md), giữ nguyên source adapter terminal-fix 12/10. Fresh empty pool dừng trước score/argmin; invalid-all reselection dừng outer loop trước None vào A*. Source original checkout clean. New code chỉ đổi ba deployment bindings và parent resource/owned-child peak guard, không thêm launcher service.
- **Why / evidence:** historical MX072_SCIENCE_ONE bị broadcast exception (0,)/(2,) sau 1195 scans. Native region eligibility >10 không còn candidate ở snapshot cuối (components9/7/5); đây là source terminal eligibility, không chứng minh hết unknown space. Corrected terminal giữ raw mission_complete=false; exception/collision/resource/persistence failure vẫn INCOMPLETE.
- **Maker validation:** 48/48 stdlib trên real COM1 Python3.6.13 PASS, explicit source-shape/observer mocks; 21 syntax files, 3 pinned adapter builds, 28 sealed files, 36 historical assets, 2,905 external rehashes PASS. 2,708 runtime files bound, 49 publication files khớp physical/Git bytes. Initial fixture failure retained. Chưa native ABI/model/NumPy/full-trajectory hoặc full numeric cold decode.
- **Affected runs / preservation:** **không có run mới**. Frozen root950a656e... rehashed, original vẫn SOURCE_EXCEPTION_INCOMPLETE (1195/22/92). Original code14/14, acquisition, weights, prefix và ROSbadHEAD giữ nguyên. Historical7 native fixture jobs/61anchors chỉ bằng chứng 12/10, không rerun hôm nay. Không thay Hospital ROS protocol; không nối RAE.
- **Resources / next:** RAM snapshot3,943,526,400 B dưới gate5,637,144,576 B, thiếu1,693,618,176 B (~1.58GiB); disk103,995,764,736 B. Không dừng job khác. [Final submission #6164080517](https://github.com/vah103/chat-gpt/pull/53#issuecomment-6164080517) cho RM08 terminal method và IR1 exact code/resources; [formal handoff](https://github.com/vah103/chat-gpt/blob/39e277052d7afff58b22eb3fbb5ac543be1b52b7/company/operations/HO-20261014-MX072-COM1-WORK1-READY-R1-TO-RM08-IR1.md). **READY_FOR_REVIEW / QA_PENDING / LIVE_RAM_BLOCKED / NOT_ADMITTED**. Sau QA cần fresh resource check và USER GO riêng COM1; old permissions không transfer. Không cập nhật teacher report vì chưa verified science milestone.

## 2026-10-14 — Work2 one G1 FIX1 ALL22 shadow; maker completed / IR1 pending

- [Exact result f5cf3fafb4298e1acc9786940248762c76af9875](https://github.com/vah103/chat-gpt/blob/f5cf3fafb4298e1acc9786940248762c76af9875/company/projects/mapex/rae_g1_r2_lite_fix1_shadow_workagent2_20261014/README.md),79/79 Git leaf+handoff SHA readback; RM08 METHOD ACCEPTe6e089d for9fd97df verified;44 source/copy hashes+22 adapter contracts PASS; pre-output freeze249b363f18/18 verified. [IR1 request #6161965022](https://github.com/vah103/chat-gpt/pull/53#issuecomment-6161965022); company checkpoint5d629146, journal§26.66. Code branch not merged.
-22decisions/1makerpass,15scenes generate382candidate-time groups per arm,1plausible/168defer/213unsupported. E16–E22 FACE_BUDGET_EXCEEDED preserved, not absence labels. State changes L0/L1/L2=0; preliminary incremental side in80groups but no actual promotion, variance support0/NA.299localgroup-budget and142binary/mean disagreement reasons overlap. PositiveE02-C0016 observed-wall only, no path/room truth.
- [Paired R1/R2](https://github.com/vah103/chat-gpt/blob/f5cf3fafb4298e1acc9786940248762c76af9875/company/projects/mapex/rae_g1_r2_lite_fix1_shadow_workagent2_20261014/PAIRED_SCENE_COMPARISON.csv) retains frozenR1 B1/B2/B3/B4=459/7/23/23 over575unknown-component/interface units; R2 face groups differ and7censoredscenes, not accuracy/count improvement. Six manual references after numeric seal only:5defer/E19resourceNA. No R1 rerun, outcome tuning, heldout validity or savings.
- Actual shadow7.397167s/childCPU7.383227s/SELF+kernel101,986,304B (~97.26MiB), one thread/native modules[], raw79.3MB<250MiB; hardCPU120s/AS2GiB. Direct-child /proc sampled8MB cannot prove complete worker memory coverage, kernel postcheck is measured peak; no cgroup hardRSS/native admission. Originalstdout freeze then appendedCOMPLETE only; prefix proof and all26numeric files unchanged, originalseal preserved. [Failure/resource ledger](https://github.com/vah103/chat-gpt/blob/f5cf3fafb4298e1acc9786940248762c76af9875/company/projects/mapex/rae_g1_r2_lite_fix1_shadow_workagent2_20261014/FAILURE_AND_RESOURCE_LOG.json).
- Original monolithic plots676,388,864B RSS FAIL retained; bounded renderer initiallyNameError retained, then fresh-child-per-scene22PNGsPASS/peak256,081,920B and all22byte-identical, detectorcalls0. Streaming maker audit22scenes/1,146armrows/allRLE/lineage/figures/hashesPASS in0.778s/56,696,832B. Initial descriptive analysis peak unmeasured. [Standalone gallery](https://github.com/vah103/chat-gpt/blob/f5cf3fafb4298e1acc9786940248762c76af9875/company/projects/mapex/rae_g1_r2_lite_fix1_shadow_workagent2_20261014/G1_R2_FIX1_ALL22_GALLERY.html) and raw/review ZIPs/CRC/source provenance available.
- **ONE_MAKER_SHADOW_COMPLETE / SCIENTIFIC_UTILITY_INCONCLUSIVE_RESOURCE_CENSORED / IR1_RESULT_QA_PENDING / NO_NATIVE_GO**. No source/R1/ALL22/protocol change or teacher-facing verified science milestone. No second maker pass/cap rescue/G2/G3/model/native/robot/simulation/newdata/STOP/accuracy/savings/generalization claim.

## 2026-10-14 — Work2 đóng bounded correction RM08-1/2/3 G1 R2-lite; no actual shadow

- [Exact corrected sibling 9fd97dfdc61f3a0a2b3cca6e86627b219aeb7fcc](https://github.com/vah103/chat-gpt/blob/9fd97dfdc61f3a0a2b3cca6e86627b219aeb7fcc/company/projects/mapex/rae_g1_r2_lite_workagent2_rm08_fix1_20261014/README.md); predecessor dd96/R1/ALL22/source/raw immutable. Actual ACK6160935502 và [focused rereview #6161156895](https://github.com/vah103/chat-gpt/pull/53#issuecomment-6161156895). Company metadata [570441a](https://github.com/vah103/chat-gpt/commit/570441a11557d0826ba1069968843b89f0a7cbf3), journal§26.65.
- RM08-1: outer-mask half-edges và support provenance riêng; unknown predicted side phải là đóng góp thêm thực sự, local contradiction không dilute. RM08-2: variance count/mean/mask/NA chỉ theo cạnh actual L1 promotion, interior pooling diagnostic. RM08-3: local mask-dilation quality, scene clutter diagnostic. Thresholds/adapter/time-t/44 refs byte-identical; schema v2 explicit.
- 61/61 maker synthetic PASS=44 unchanged+1 justified replacement+16new. Original45-on-fix44PASS/1 obsolete interior-variance assertionFAIL preserved. Five predecessor/corrected synthetic comparisons PASS; nonrectangular downstream example mocks extractor explicitly. Final wall0.573169425s/CPU0.572422s/SELF50,982,912B; comparison0.364865729s/SELF51,937,280B; CPU120s/AS2GiB, RSS256MiB post-check, one thread; not native or actual22-scene overhead. Early inherited7×2 diagnostic repeated then removed from final runner; no selection/retuning.
- 34/34 artifact/operation Git SHA readback; [freeze](https://github.com/vah103/chat-gpt/blob/9fd97dfdc61f3a0a2b3cca6e86627b219aeb7fcc/company/projects/mapex/rae_g1_r2_lite_workagent2_rm08_fix1_20261014/METHOD_CANDIDATE_FREEZE.json), [receipt](https://github.com/vah103/chat-gpt/blob/9fd97dfdc61f3a0a2b3cca6e86627b219aeb7fcc/company/projects/mapex/rae_g1_r2_lite_workagent2_rm08_fix1_20261014/TEST_REPORT.json), [counterexamples](https://github.com/vah103/chat-gpt/blob/9fd97dfdc61f3a0a2b3cca6e86627b219aeb7fcc/company/projects/mapex/rae_g1_r2_lite_workagent2_rm08_fix1_20261014/RM08_SYNTHETIC_COUNTEREXAMPLES.json). Actual numeric inputs read0/no new DELL/COM1 workload/re-extract/R1 rerun; model/native/planner/raycast/robot/simulation/newdata0. No protocol change/baseline rerun/teacher-facing science milestone.
- **MAKER_CORRECTION_COMPLETE / RM08_FOCUSED_REREVIEW_PENDING / E01_E22_SHADOW_BLOCKED / SHADOW_COUNT0.** Genuine exact-method ACCEPT still required before shadow. Code branch not merged, no G2/G3 integration GO or claimed room truth/completion/savings/generalization.

## 2026-10-14 — G1 R2-lite code/tests; pending method gate before actual shadow

- [Work2 exact dd96e5c9def444c8f943bb8e13a444b266e977ec](https://github.com/vah103/chat-gpt/blob/dd96e5c9def444c8f943bb8e13a444b266e977ec/company/projects/mapex/rae_g1_r2_lite_workagent2_20261014/README.md): R2-L0 orthogonal observed wall lines/bounded faces, L1 cached LaMa fragments/binary-mean conflict, L2 unknown-wall variance defer. R1/ALL22 immutable.
- Maker45/45 synthetic PASS; final0.878587s/SELF48,193,536B;28/28 Git SHA readback. Read-only44 input hashes checked1,692,280B/0.023s; numeric actual snapshots not loaded, no1195scanre-extract/R1shadowrerun. Initial failures and synthetic-parameter sensitivity retained.
- [RM checklist](https://github.com/vah103/chat-gpt/blob/dd96e5c9def444c8f943bb8e13a444b266e977ec/company/projects/mapex/rae_g1_r2_lite_workagent2_20261014/METHOD_REVIEW_CHECKLIST.md); [actual request #6160489054](https://github.com/vah103/chat-gpt/pull/53#issuecomment-6160489054). **METHOD_REVIEW_PENDING / NO_ACTUAL_SHADOW / SCIENTIFIC_UTILITY_NOT_ASSESSED**. Stop before one proposed E01–E22 shadow; no fake RM ACK/ACCEPT or dummy22 overlays.
- action=null, physical_safety=NOT_CERTIFIED, tracking/extended/G2 not implemented. No native/model/raycast/planner/robot/simulation/science/newdata/source edits/cleanup/host switch or Work1 interference. Original MX072 SOURCE_EXCEPTION_INCOMPLETE/mission_complete=false/frozen retained.


## 2026-10-14 — RAE G1 automatic offline hypotheses: complete R1, scientific inconclusive

- What changed: Work Agent 2 completed native-2D G1 A–D from existing MX072 data, separate from ROS2 Hospital loop. [Exact 946f56d1d28cf6889ddc6a4f02f363404f1ac0d7](https://github.com/vah103/chat-gpt/blob/946f56d1d28cf6889ddc6a4f02f363404f1ac0d7/company/projects/mapex/rae_g1_work_agent2_20261013/complete_r1_20261014/README.md); B1–B4, strict time-t boundary, full-scene geometry proposals, causal IDs and complete22 gallery. No manual-window proposals or future/GT policy input.
- Why: RAE needs a reviewable room-hypothesis component before route/viewpoint stages; retrospective hand windows are not an automatic detector or room truth.
- Evidence: maker44 component tests PASS before first real shadow; method/input/output freezes; full19,969-event metadata and1,195-scan cache-age inventory. Actual22 policy caches same-source age0. Shadow575 correlated candidate-time records: B2=7 plausible, B3/B4=23, no-variance=72; all six highlighted reference cases remain ambiguous. No independent labels or semantic accuracy/savings claim.
- Decision: retain negative results and frozen thresholds. Scientific status EXPLORATORY_ONE_RUN / INCONCLUSIVE, independent QA pending. [Handoff](https://github.com/vah103/chat-gpt/blob/4f1ae458cf471401c37253a29a48f08dc1146125/company/operations/HO-20261014-RAE-G1-OFFLINE-R1-WORK2-TO-CHAT-RM-IR-REVIEW.md) requests actual Chat Agent + RM/IR pickup; heldout gates/operating point must be approved before independent data are opened.
- Affected runs / baseline rerun: none newly executed; original root950a656e… remains SOURCE_EXCEPTION_INCOMPLETE / mission_complete=false. No new native/model/planner/raycast/robot, host switch or source/raw/ALL22 edits; Work1 authority separate.
- Reporting: this is an exploratory maker package, not an independently verified research verdict; no teacher-facing Google Docs update. Failure/resource ledger retains first synthetic failure and repaired local ZIP integrity failure.


## 2026-10-14 — Guided native 2D R5R1: bounded pre-execution correction

- What changed: following IR1 successor06 REVISE at R5 `13c282bd`, Work1 corrected the current reviewer role and exact guard parent binding to the real private-interpreter `runtime_entry.py guard` kernel argv. [Immutable R5R1 `9da8aaa5`](https://github.com/vah103/chat-gpt/blob/9da8aaa5622de3ee61beadd9b90ea4d40ccefed6/company/projects/mapex/mx072_guided_fixed22_r5_bounded_repair_r1_20261014_work1/README.md), [return to IR1](https://github.com/vah103/chat-gpt/pull/53#issuecomment-6151640885).
- Why: predecessor05's R4 approval cannot approve a new R5 head; runpy changes Python sys.argv but does not insert guided_guard.py into Linux /proc/PID/cmdline.
- Validation: 58/58 stdlib tests, 21 syntax files; actual parent/child fixtures test the production predicate. NOT_ADMITTED remains blocked. Runtime/R4/source hashes and consumed R3 claim/reserve preserved.
- Affected runs: none executed; native qualification/science not started. This is native MapEx 2D support, separate from ROS2 Hospital protocol and RAE G1.
- Baseline rerun: none authorized or needed for this bounded code correction. Independent exact-head review is pending; new native qualifier needs its own specific USER consent, existing ONE science consent stays conditional/unspent.


## 2026-09-22 — TU goal-domain differs from original MapEx

- Original MapEx TU samples 100 goal cells directly from `valid_space == 1` for each map/start condition, stores those coordinates, and reuses the same goal set across methods/timesteps.
- The public implementation does not additionally require the sampled goal cell to be GT-free at the sampling step. Because `valid_space` and occupancy are downsampled with different rules, some valid-space cells can overlap GT-occupied cells; such goals are effectively predisposed to TU failure.
- This project intentionally uses a different goal domain for New Room / Hospital TU: 100 deterministic goals are sampled from the connected structural GT-free ROI (fixed seed), so every goal is a physically traversable free-space location in the scored environment.
- Consequence: this project's TU is **MapEx-style but not byte-for-byte equivalent** to the original paper implementation. Absolute TU values should not be compared directly with the paper without noting this goal-domain difference.
- Rationale: restricting goals to connected GT-free space is more meaningful for navigation because the metric then asks whether predicted topology supports valid robot destinations, rather than allowing goals that may lie inside walls/obstacles due to raster/downsampling semantics.
- Fairness within this project is preserved because the same frozen goal set is reused across compared methods on the same environment/evaluation profile.
- No metric change is made by this note; it documents the current intentional adaptation and its interpretation.

## 2026-09-22 — MapEx fixed-horizon vs project completion mismatch

- Observation: the original MapEx paper evaluates exploration under a fixed budget of **1000 timesteps**. The reported Coverage curves therefore measure how much of the environment is observed within that fixed horizon, not the final Coverage after exhaustive exploration.
- Current project behavior: New Room / Hospital runs are generally allowed to continue until the shared exploration-completion condition (or an explicit early-stop rule) is reached. Final Coverage values from these runs therefore represent near-completion/completion behavior rather than Coverage@1000.
- Consequence: directly comparing paper MapEx final plotted Coverage (about 82–83% at the 1000-timestep horizon) with this project's near-completion values (often close to 100%) is misleading because the stopping horizon is different.
- Important interpretation: the paper's lower final Coverage does **not** by itself mean MapEx cannot explore the remaining area; it mainly reflects the fixed evaluation budget. Conversely, this project's near-100% final Coverage does **not** demonstrate a better exploration policy unless compared at a matched budget/horizon.
- Reporting rule: distinguish at least **Coverage@fixed-budget** from **final/completion Coverage**. Do not present them as directly comparable metrics without matching the exploration horizon.
- No protocol change yet: this note records the mismatch only. Any future reproduction study should either evaluate this project at the same 1000-timestep horizon or explicitly justify another matched time/distance budget.

## 2026-09-22 — Coverage GT/ROI semantics differ from original MapEx

- Observation: the current project Coverage denominator is based on connected-free ROI masks such as `hospital_connected_free_v1` / `new_room_connected_free_v2`, i.e. reachable/free-space support only.
- Original MapEx behavior: the public MapEx evaluator computes Coverage inside `valid_space.npy`, and counts a cell as covered when the observed map is no longer unknown. The MapEx `valid_space` mask is not equivalent to free-space-only occupancy GT; it can include both free and occupied cells inside the valid evaluation region.
- Practical meaning: our current Coverage asks approximately "how much of the reachable free-space ROI has become known", while original MapEx asks approximately "how much of the valid environment region has become known".
- Consequence: current absolute Coverage values are **not metric-semantics-equivalent to original MapEx Coverage** and should not be described as an exact reproduction of the paper metric without qualification.
- Scope: this note does **not** invalidate within-project Nearest-vs-MapEx comparisons when both methods use the same frozen ROI; it affects cross-paper comparability and interpretation of the absolute Coverage number.
- No protocol change yet: do not replace the existing ROI automatically. If exact MapEx-style metric reproduction is required, define and validate a separate MapEx-style valid-space mask for New Room/Hospital, version the metric/ROI, and decide whether historical values need backfill.
- Evidence checked: original `castacks/MapEx` coverage code uses `known & valid_space` divided by `valid_space`; inspection of an original KTH test-map pair confirms `valid_space` is not identical to free-only occupancy support.
- Next action: USER/WORK decides whether to keep current connected-free Coverage as the project metric, add a separate MapEx-style Coverage metric, or migrate under a new protocol/metric version.

## 2026-09-22 — Canonicalize reviewed R002 GT-semantics diagnostic

- What changed: synced the independently ACCEPTED R002 evaluator and toy tests into technical `main` byte-identically from accepted implementation `feb94eaa9c1ba5aa4f9993792dae454bc4edcd60`; updated D1 STATUS/README/research log with the reviewed one-run `mpx_001` evidence.
- Validation: pre-score compile PASS; 12/12 unit tests PASS; `mpx_001` exit code 0 with 35/35 decisions, Gate-P support parity True, topology-invalid 0; independent H010 code review verdict ACCEPT.
- Scientific scope: R002 remains a post-hoc diagnostic. Reference A / negative Gate P2 are preserved. One-run evidence is mixed/negative late and does not rescue direct D1.
- No methodology changes: threshold 0.5, B tolerances, one-to-one matching order, C0 0.05 m / 4-connectivity, topology domain and seed rule remain frozen.
- No cohort expansion: `mpx_002...mpx_010` were not run; further expansion requires explicit USER/WORK authorization.
- Raw first-score artifacts remain local/untracked; canonical repo stores accepted evaluator/tests plus reviewed summary/reproducibility notes.
- Next action: USER verifies fetched `origin/main` and local status without overwriting unrelated work; then close R002-SYNC if verification passes.

## 2026-09-21 — Fix Gate-P2 fractional origin reprojection

- What changed: removed the exact integer-lattice origin assertion in `analysis/d1/d1_gate_p.py`; P2 now uses the same nearest-cell rounding reprojection as the canonical MapEx evaluator.
- Why: the first `mpx_001 --reference both` smoke test failed because SLAM raw-map origins can have fractional 0.05 m canvas offsets.
- Observation: this was an analyzer alignment bug, not a run-data provenance failure.
- Audit: P2 records x/y origin-rounding residuals; for the original failing decision they are approximately +0.01523 m and -0.02458 m.
- Affected runs: offline Gate-P2 analysis only; no raw run changed.
- Does baseline need rerun?: no.
- Next action: rerun the one-run smoke test, then full 10-run P1+P2 if clean.

## 2026-09-21 — Match Gate-P predictions to exact raw maps

- What changed: Gate-P prediction loading now hard-checks shape, resolution, origin, source map timestamp and G1/G2/G3/mean member identity against the exact decision raw map.
- Why: shape-only validation could miss a stale/wrong-decision prediction with identical map extent.
- Environment policy: metadata/prediction environment mismatch is WARN-only; GT ID/canvas/resolution and prediction/raw spatial/source identity remain hard-fail.
- Affected runs: no raw runs changed; offline analyzer only.
- Does baseline need rerun?: no.
- Next action: smoke test `mpx_001 --reference both --no-figures`, then run the full cohort if clean.

## 2026-09-21 — Harden D1 Gate P2 provenance before cohort run

- What changed: added structural-GT provenance validation and expanded P1-vs-P2 class-specific/support comparison in `analysis/d1/d1_gate_p.py`.
- Provenance policy: missing legacy metadata warns; present contradiction in environment/GT ID/GT file/canvas/resolution/runtime resolution fails.
- Comparison policy: overall + last-10 now include free/occupied precision/recall/IoU in addition to accuracy/macro-IoU/MAE.
- Support policy: compare P1/P2 primarily by evaluated area and unknown-area fraction because their cell resolutions differ.
- Threshold unchanged: Gate P uses `<0.5 free`, `>=0.5 occupied`; legacy evaluator boundary semantics are documented separately.
- Affected runs: no raw runs changed; offline analyzer only.
- Does baseline need rerun?: no.
- Next action: smoke test then execute `--reference both` on `mpx_001...mpx_010`.

## 2026-09-21 — D1 Gate P2 structural-GT reference added

- What changed: extended `analysis/d1/d1_gate_p.py` with `later_observed|structural_gt|both` modes; default is `both`.
- Why: Gate P1 is right-censored because it only evaluates `Unknown_t ∩ EventuallyObserved`; P2 must evaluate the broader valid structural-GT unknown region.
- Structural target domain: `Unknown_t ∩ structural_gt.evaluation_mask`.
- Alignment: runtime 0.10 m cells are nearest-neighbour expanded to the canonical 0.05 m structural-GT canvas using the existing evaluation-grid convention.
- Important methodological decision: connected-free ROI is not the P2 classification mask because it contains free-space support and would discard occupied targets.
- Affected runs: no raw runs changed; offline analysis only.
- Does baseline need rerun?: no.
- Observation: code implemented; P2 numerical execution is still pending on local NPZ artifacts.
- Next action: run Gate P with `--reference both` on `mpx_001...mpx_010` and compare P1 vs P2 before Gate U.

## 2026-09-09 — All-training evaluation moved offline

- Restored three-model online config/worker/bridge/recorder; removed alltrain startup, saving and metadata requirements.
- Added `predict_alltrain_offline.py`: loads only the whole-training predictor, consumes saved raw decision maps, publishes complete hash-linked manifest, and preserves original run records.
- Evaluator uses completed offline predictions for primary IoU/TU with secondary ensemble-mean metrics; incomplete/stale manifests are rejected.
- Four ROS-free tests pass. Real inference remains pending absent model weights/environment. No robot motion or formal research result was produced.
- The earlier four-model online implementation is superseded; alltrain no longer adds decision time or GPU memory during exploration.

Ghi ngắn gọn các quyết định hoặc sự cố có thể ảnh hưởng kết quả.

## Template

### YYYY-MM-DD — <run or change>

- What changed:
- Why:
- Affected runs:
- Does baseline need rerun?:
- Observation:
- Next action:

## 2026-09-07 — Adaptive Temporal Anchor V1 after hard-chain diagnostic

- What changed:
  - Added `launch/toolbox_adaptive.launch.py` as a clean A/B variant of the previously stable `toolbox.launch.py`; scan cadence, Karto scan matching, near-chain graph construction, conservative loop-closure settings, whole-graph Ceres optimization and Nav2 remain unchanged.
  - Integrated Adaptive Anchor V1 into the vendored `slam_toolbox/solvers/ceres_solver.cpp`, default disabled. `toolbox.launch.py` explicitly sets `adaptive_anchor_enabled=false`; `toolbox_adaptive.launch.py` enables it.
  - Only strict sequential graph edges (`node_gap<=1`) receive `w(n)=1+2*exp(-n/50)`, i.e. `3x -> 1x`. Long-gap edges are never strengthened.
  - V1 intentionally does not apply a second covariance confidence multiplier because Karto covariance already forms the Ceres information matrix.
  - `LinkInfo` does not expose edge origin/type to `ScanSolver`, so V1 uses `node_gap>=30` only as a conservative loop-evidence candidate fallback. Evidence is captured at `AddConstraint()` before Karto loop correction can optimize away the initial residual.
  - Strong release evidence requires at least 3 recent independent long-gap observations with consistent correction and squared normalized/Mahalanobis residual `>=9.0`. Multiple edges attached to essentially the same new node cannot count as independent votes.
  - Strong evidence releases only the related trajectory interval from factor `1.0 -> 0.5`, then Ceres solves the whole graph. Stage 2 recomputes current residuals for the same evidence edges; persistent strong conflict releases that interval to factor `0.0`, returning its sequential edges to ordinary Toolbox `1x`, followed by another whole-graph solve. Release is one-way for the mapping session.
  - Added `src/slam/adaptive_anchor_v1.md` with the exact algorithm, limitations and runtime validation checklist.
- Why: the strict hard-chain test was not satisfactory because it removed too much of Toolbox's ability to distribute correction. The new diagnostic keeps Toolbox's normal mechanisms and adds only a reversible soft temporal preference, with a later-evidence escape path so an incorrect early trajectory is not protected indefinitely.
- Affected runs: Adaptive Anchor V1 diagnostic runs only. Existing Toolbox/Nearest/MapEx data are not reclassified by this change.
- Does baseline need rerun?: no official `hospital_v2` protocol change has been made. For an Adaptive-vs-Toolbox diagnostic, both variants should use the same newly built solver binary, with only `adaptive_anchor_enabled` differing.
- Observation: implementation is committed but compile/runtime validation on the TurtleBot4 Jazzy machine is still pending. The explicit edge-type limitation remains: long-gap evidence is a fallback heuristic because Karto's `LinkInfo` API does not identify loop origin.
- Next action: pull, rebuild `slam_toolbox`, verify `toolbox.launch.py` logs `enabled=false`, verify `toolbox_adaptive.launch.py` logs `enabled=true`, then run ordinary `scripts/nf_basic.py` and inspect stage-1/stage-2 release logs before judging map quality.

## 2026-08-29 — Treat GOAL_OCCUPIED (206) like NO_VALID_PATH (208)

- What changed:
  - `scripts/nf_basic.py` now classifies main-goal `206 = GOAL_OCCUPIED` and `208 = NO_VALID_PATH` as planner-blocking failures.
  - Either code clears the current main frontier, suppresses candidates within `0.10 m` during ordinary selection, and lets exploration choose another frontier instead of retrying the same `(x,y)` forever.
  - Suppression remains reversible: when ordinary candidates are exhausted, terminal `ComputePathToPose` revalidation checks the full eligible set again and can restore a reachable frontier.
  - Controller/execution failures such as `105 FAILED_TO_MAKE_PROGRESS` still use the existing path-guided recovery path when a usable main `/plan` exists.
- Why: `nearest_004` reached frontier `(11.38,7.82)`, then `NavigateToPose` repeatedly returned `206` with no usable `/plan`. The previous generic failure branch kept the same main frontier pending, so the 1 Hz exploration loop resent it indefinitely until manual Ctrl+C.
- Affected runs: `nearest_004` attempt stopped at coverage `0.854499037`, distance `159.13 m` is invalid/incomplete and should be replaced. `nearest_001`–`nearest_003` predate this bugfix; `nearest_003` recorded one `206` but still completed.
- Does baseline need rerun?: `nearest_004` yes, from scratch. For a strictly identical-code formal batch, earlier runs should be treated as pre-fix provenance; current local-window runs are still debug-profile data relative to formal `hospital_v2`.
- Observation: code fix committed as `6c81982`; runtime validation is pending.
- Next action: rerun `nearest_004`; if `206` occurs, verify the immediate sequence is `Main frontier abandoned after GOAL_OCCUPIED (206)` followed by selection of another frontier, with no repeated same-goal loop.

## 2026-08-28 — Event-driven global replanning for `nf_basic.py`

- What changed:
  - Replaced the debug BT's periodic `RateController hz=1.0` global replanning with a plan-once-then-follow sequence.
  - `ComputePathToPose` now runs once when a NavigateToPose attempt starts; map updates alone no longer regenerate the global path while `FollowPath` is progressing.
  - If `ComputePathToPose` or `FollowPath` fails, a `RecoveryNode` waits `0.25 s` and retries the whole plan/follow sequence once, which triggers one fresh global plan using the latest map.
  - If that event-triggered retry also fails, NavigateToPose returns failure to `nf_basic.py`, so the existing path-guided intermediate-goal fallback remains available.
- Why: Cartographer map updates can change the global costmap frequently; periodic 1 Hz replanning makes the displayed/global path jump even when the current path is still usable. The new BT replans only when execution actually fails.
- Affected runs: debug `nf_basic.py` execution only; no official `hospital_v2` run affected.
- Does baseline need rerun?: no official baseline. If this execution policy is later adopted for benchmark runs, it must be protocol-versioned and shared across Nearest/MapEx/proposed methods.
- Observation: implementation committed; runtime behavior still needs validation.
- Next action: run `carto.launch.py + nf_basic.py`, verify `/plan` is normally published once per navigation attempt, then place/observe a newly discovered obstacle on the path and confirm a fresh plan appears only after planner/controller failure.

## 2026-08-28 — Revert Cartographer smoothness tuning after no improvement

- What changed:
  - Restored `TRAJECTORY_BUILDER_2D.use_online_correlative_scan_matching = true`.
  - Restored Cartographer OccupancyGrid `publish_period_sec` from `3.0` back to `1.0`.
- Why: disabling online correlative scan matching and slowing `/map` publication did not remove the visible robot jerk, so those Cartographer changes are no longer justified as performance fixes.
- Affected runs: Cartographer debug only; no official `hospital_v2` run affected.
- Does baseline need rerun?: no. Cartographer remains a debug mapping alternative.
- Observation: final `/cmd_vel` Twist values were changing smoothly, while the robot still appeared jerky. Together with the low wall-time `/scan` and `/odom` rates, the current working hypothesis is Gazebo/physics real-time performance rather than Cartographer local-matching load.
- Next action: keep the restored accuracy-first Cartographer config fixed and diagnose Gazebo real-time factor / headless rendering separately.

## 2026-08-28 — Disable online correlative scan matching for smoothness A/B test

- What changed: `TRAJECTORY_BUILDER_2D.use_online_correlative_scan_matching` changed from `true` to `false` in `config/cartographer_hospital_2d.lua`.
- Why: the Cartographer debug stack navigates correctly but the simulated robot appears visibly jerky; measured wall-time topic rates were about `/cmd_vel=14 Hz`, `/odom=14 Hz`, `/scan=2.6 Hz`, suggesting the simulator is running well below real time. This A/B isolates the relatively expensive online correlative local matcher without changing frontier logic.
- Affected runs: Cartographer debug only; no official `hospital_v2` run affected.
- Does baseline need rerun?: no. Cartographer is still a debug mapping alternative and has not been promoted to the official protocol.
- Observation: runtime result pending. Wheel odometry, IMU fusion, Ceres scan matching, submaps, pose-graph optimization and loop closure remain enabled.
- Next action: rerun the same Cartographer + `nf_basic.py` setup, compare visual smoothness and `/scan`/`/odom` wall-time rates, then revisit old corridors to check whether map alignment remains acceptable.

## 2026-08-28 — Fix Cartographer IMU sensor-frame TF mismatch

- What changed:
  - First `carto.launch.py` runtime showed Cartographer repeatedly failing to transform IMU messages because `/imu.header.frame_id` is `turtlebot4/imu_link/imu`, while robot TF exposes the physical sensor link as `imu_link`.
  - Added an identity static transform `imu_link -> turtlebot4/imu_link/imu` in `launch/carto.launch.py`. The IMU sensor has zero extra pose relative to `imu_link`, so this reconnects the Gazebo sensor-scoped message frame to the real robot TF tree without disabling IMU fusion.
- Why: Cartographer started and loaded its trajectory successfully, but the missing IMU source frame prevented normal sensor processing, so `map` was never created and Nav2 subsequently timed out waiting for `map -> base_link`.
- Affected runs: Cartographer debug only; no official `hospital_v2` run affected.
- Does baseline need rerun?: no official baseline. The failed Cartographer launch is diagnostic only.
- Observation: runtime error was a TF integration problem, not Cartographer package/config loading failure.
- Next action: pull and rerun `carto.launch.py`; verify the IMU TF warning disappears, `/map` is published at 0.10 m/cell, and `map -> odom -> base_link` is available before running `nf_basic.py`.

## 2026-08-28 — Add Cartographer 2D debug mapping alternative

- What changed:
  - Added `config/cartographer_hospital_2d.lua` and `launch/carto.launch.py` as a replacement mapping stack for debug validation while retaining the same Hospital simulation and stock-style Nav2 debug profile.
  - Cartographer consumes `/scan`, `/odom`, and `/imu`; tracking frame is `imu_link`, Gazebo keeps publishing `odom -> base_link`, and Cartographer publishes the loop-closed `map -> odom` relation.
  - Local Cartographer submaps use `0.05 m/cell`; the ROS `/map` OccupancyGrid exposed to Nav2 and `nf_basic.py` remains `0.10 m/cell`.
  - Loop closure remains enabled through Cartographer's pose graph. Online correlative scan matching is enabled and pose-graph optimization is requested every 60 nodes for an accuracy-first first test.
  - LiDAR used by Cartographer is limited to `12 m` rather than the full simulated 20 m to reduce long-range ambiguous corridor structure during scan matching.
- Why: the latest long exploration run returned to previously mapped space with visible map misalignment, after which MPPI repeatedly failed with `PATIENCE_EXCEEDED`; the working hypothesis is accumulated SLAM drift/map inconsistency rather than a frontier-ranking failure.
- Affected runs: debug Cartographer runs only. `hospital_v2` official protocol still names `slam_toolbox` until Cartographer is runtime-validated and deliberately adopted as a new protocol version.
- Does baseline need rerun?: not yet because no official Cartographer protocol has been adopted. If Cartographer becomes the official mapping stack, Nearest/MapEx/proposed-method official runs must all use it and the protocol version must change.
- Observation: code/config implemented; runtime validation is still pending.
- Next action: install Jazzy Cartographer packages, run `carto.launch.py + nf_basic.py`, verify `/map`, `map -> odom`, `/imu`, and `/odom`, then drive/explore far enough to revisit old corridors and compare map overlap against the slam_toolbox run.

## 2026-08-28 — Robust conservative completion guard for `nf_basic.py`

- What changed:
  - `nf_basic.py` no longer prints completion after a single empty frontier scan.
  - COMPLETE now requires zero frontier regions larger than 10 cells across `5` distinct `/map` generations, with terminal sweeps spaced by at least `2 s`, navigation idle for at least `10 s`, and node age at least `20 s`.
  - Any reappearance of a large frontier resets terminal verification immediately.
  - If large frontier regions still exist but every representative is inside the debug `MIN_DISTANCE_THRESHOLD=0.5 m`, the node publishes `BLOCKED_BY_MIN_DISTANCE` and explicitly does **not** claim exploration completion.
  - A pending main frontier or recovery subgoal also prevents completion; this keeps the no-blacklist/same-main-goal recovery semantics conservative.
  - Added transient-local `/frontier_exploration_complete` (`std_msgs/Bool`) and `/frontier_exploration_status` (`std_msgs/String` JSON) so terminal state is machine-readable and visible to late subscribers while the node remains alive.
  - After COMPLETE, markers/path are cleared and the node permanently stops issuing new navigation goals.
- Why: a single transient SLAM map with no detected frontier can create a false completion, while the debug 0.5 m filter can hide still-existing frontier regions. The completion signal should prefer non-termination over falsely declaring the map explored.
- Affected runs: debug `stock.launch.py + nf_basic.py` only; not current `hospital_v2` official baseline semantics.
- Does baseline need rerun?: no official benchmark uses this debug file. If adopted later, completion semantics must be protocol-versioned and shared across compared methods.
- Observation: implementation committed; runtime terminal validation pending.
- Next action: run until the end, verify `VERIFYING_COMPLETE` progresses across fresh map updates, then confirm exactly one `COMPLETE` and `/frontier_exploration_complete: true`, with no later goal dispatch.

## 2026-08-28 — Debug path-guided intermediate-goal recovery for `nf_basic.py`

- What changed:
  - Added `behavior_trees/navigate_to_pose_subgoal.xml`, a lightweight NavigateToPose tree that keeps 1 Hz global replanning but omits the stock Nav2 recovery loop so planner/controller failure returns to `nf_basic.py` quickly.
  - `nf_basic.py` now keeps the selected frontier as a persistent **main goal**, stores only the latest `/plan` produced while navigating to that main goal, and does not blacklist or switch frontier after execution failure.
  - When the main NavigateToPose fails and a usable main-goal path exists, `nf_basic.py` picks a temporary subgoal on that path. Target distance is half the remaining path, capped at `0.70 m`, normally at least `0.25 m`, while trying to keep `0.15 m` separation from the main frontier.
  - If the temporary subgoal succeeds, the exact same frontier is sent again as the main goal so Nav2 replans from the new robot pose. If the subgoal also fails, the same main frontier remains selected and is retried; no alternate frontier/blacklist is introduced.
  - Ctrl-C shutdown now checks `rclpy.ok()` before calling `rclpy.shutdown()` to avoid the previous double-shutdown RCLError.
- Why: the stock BT was observed to report `Failed to make progress` repeatedly while internally cycling through recovery actions for a long time. The test hypothesis is that a short path-aligned motion can move the robot out of a local controller deadlock without changing Nearest Frontier ranking.
- Affected runs: debug `stock.launch.py + nf_basic.py` only. This is not yet `hospital_v2` official benchmark execution semantics.
- Does baseline need rerun?: no official run exists under this debug recovery. If this recovery is adopted for the benchmark, it must become a protocol-versioned shared execution adapter and be applied to Nearest, MapEx and the proposed method before official comparisons.
- Observation: implementation is committed but not yet runtime-validated.
- Next action: reproduce the previously stuck frontier, verify log sequence `Main goal failed -> Path-guided recovery -> Subgoal reached -> retry same main frontier`, and check that the robot actually changes pose before replanning.

## 2026-08-27 — Switch Hospital runtime map/policy grid to 0.10 m while preserving evaluation grid

- What changed:
  - Protocol tăng từ `hospital_v1` lên `hospital_v2`.
  - `hospital_slam.yaml` và `hospital_slam_no_loop.yaml` đổi OccupancyGrid runtime resolution `0.05 -> 0.10 m/cell`.
  - Nearest và MapEx config được pin runtime frontier/prediction resolution `0.10 m/cell`.
  - `hospital_canvas_v1` vẫn giữ `0.05 m`, `1504 x 2123`, origin `(-25.6,-60.1)`; `hospital_connected_free_v1`, denominator `215435` và ROI SHA không đổi.
  - Generic `exploration_recorder.py` và legacy `mapex_nearest_full.py` giờ chấp nhận source map có resolution là integer multiple của `0.05`; map `0.10` được nearest-neighbour expand thành `2 x 2` cell trước khi paste lên fixed canvas.
- Why: MapEx gốc làm policy/prediction trên khoảng `0.10 m/pixel`. Dùng runtime SLAM `0.10` làm Nearest frontier, MapEx frontier và LaMa prediction chung một grid, loại bỏ bước downsample `0.05 -> 0.10` trong `control_tb4_mapex.py` và tránh candidate-set mismatch giữa Nearest/MapEx.
- Affected runs: mọi run mới sau thay đổi phải ghi `protocol_version=hospital_v2`. Các pilot v1 chỉ giữ làm diagnostic/history và không được trộn thống kê với v2.
- Does baseline need rerun?: có đối với mọi comparison chính thức. Chưa có official baseline hợp lệ nên chưa mất benchmark chính thức; cần chạy lại Nearest dưới v2 trước khi so full MapEx.
- Observation: evaluation ROI không cần regenerate vì runtime grid và evaluation grid được tách rõ. Raw map vẫn lưu 0.10; metric dùng reprojected fixed canvas 0.05.
- Next action: rebuild runtime, xác nhận `/map.info.resolution=0.10`, chạy Nearest v2 pilot và verify raw snapshot resolution 0.10 + fixed canvas shape `(2123,1504)` + coverage hợp lệ.

## 2026-08-26 — Finalize Nearest official-run schema before benchmark

- What changed:
  - Thêm `mapex_nearest_ros_official.py`: benchmark clock bắt đầu trước candidate computation đầu tiên, lưu exact exhausted/no-candidate decision, thêm stable `candidate_id` và selected-candidate topic.
  - Thêm `research_recorder_official.py`: đồng bộ `t=0` theo policy timestamp, hash source + installed runtime files, ghi rõ simulator seed policy.
  - Siết `validate_nearest_run.py`: kiểm tra metrics/snapshot integrity, candidate join, PENDING/failure reason, provenance và clean git cho official runs.
  - Đồng bộ `EXPERIMENT_PROTOCOL.md`, `DATA_SCHEMA.md`, `ROADMAP.md`, `config/nearest.yaml` với Hospital below-1m adaptation.
  - Raw `experiments/**` được ignore toàn bộ để pilot output không làm git worktree dirty.
- Why: tránh bias thời gian decision đầu, tránh mất exact termination state, tránh join candidate bằng suy đoán và tránh phải rerun vì thiếu provenance/data.
- Simulator seed: intentionally uncontrolled Gazebo default; dùng repeated-run statistics thay vì giả định fixed seed.
- Affected runs: mọi pilot trước `nearest_pilot_007` không có toàn bộ final schema. Không pilot nào dùng làm official benchmark.
- Does baseline need rerun?: chưa có official baseline nên không mất kết quả chính thức.
- Next action: chạy `nearest_pilot_007`; preflight + live validator + final validator đều PASS thì khóa code và bắt đầu `nearest_001...nearest_010`.

## 2026-08-26 — Disable MapEx 1 m rejection for Hospital

- What changed: thêm `scripts/mapex_nearest_ros_hospital_adapted.py` và chuyển Hospital runtime sang semantics này. Hospital vẫn detect/cluster/represent/rank frontier theo MapEx, nhưng candidate `<1 m` không còn bị reject chỉ vì khoảng cách; nó được gửi sang Nav2 `ComputePathToPose` như candidate bình thường.
- Why: `nearest_pilot_005` chứng minh startup deadlock do rule gốc: `534` frontier cells, `24` regions, chỉ `1` large region `>10`, chỉ `1` representative ở `0.469 m`, candidate duy nhất bị reject trước Nav2.
- Affected runs: `nearest_pilot_005` invalid cho benchmark; dùng làm evidence cho protocol adaptation.
- Does baseline need rerun?: chưa có official run nên chưa mất benchmark nào. Tất cả official Nearest và full MapEx Hospital sau này phải dùng cùng below-1m adaptation để đảm bảo fairness.
- Observation: vấn đề nằm trước Nav2; candidate 0.469 m chưa từng được gửi planner trong pilot_005.

## 2026-08-26 — Replay-safe logging before official Nearest runs

- What changed:
  - Exact frozen `decision_map` được policy process lưu trực tiếp, gồm raw OccupancyGrid + fixed-canvas copy.
  - Lưu map-frame robot pose dùng cho ranking, full candidate set/rank, `below_1m` flag, Nav2 path status, planner timing, selected path length và execution result.
  - Recorder lưu periodic raw/canvas snapshots mỗi 10 s + final map để tính occupied IoU/TU offline.
  - `decisions.csv` thêm `policy_decision_id`, coverage, map-frame pose, navigation detail/failure reason.
  - Metadata thêm git dirty state và SHA-256 code/config quan trọng.
- Why: tránh phải rerun Nearest chỉ vì thiếu snapshot/candidate/failure/computation data khi chuyển sang stage analysis, oracle/ablation hoặc offline IoU/TU.
- Affected runs: pilot cũ trước schema này không dùng làm official benchmark.
- Does baseline need rerun?: chưa có official baseline.

## 2026-08-26 — Replace custom WFD Nearest with official MapEx Nearest policy

- What changed: research Nearest baseline không còn dùng `frontier_autonomy.launch.py` / WFD custom. Thêm `scripts/mapex_nearest_ros.py` port policy `nearest` từ `castacks/MapEx` commit `53636bd1c79153acc3c74a532837d78c926bae5e`.
- Why: mục tiêu là so Hospital MapEx với chính nearest-frontier baseline semantics mà implementation MapEx công bố, tránh thay đổi frontier generator làm confound kết quả.
- Official policy pinned: free cell kề unknown theo 8-neighbour; frontier regions 8-connected; giữ region khi size `>10`; representative là frontier cell gần arithmetic mean nhất; score bằng Euclidean distance; MapEx gốc có `cur_pose_dist_threshold_m=1.0`; no A* path thì reselect next-lowest cost frontier.
- ROS adaptation: Hospital thay simulator execution bằng Nav2 `ComputePathToPose` + `NavigateToPose`. Goal vẫn là đúng frontier-center, không dùng WFD standoff/goal shifting. Sau pilot_005, Hospital disable riêng 1 m rejection như note phía trên.
- Affected runs: WFD pilot chạy dở trước thay đổi này bị dừng/xóa và không được tính.
- Does baseline need rerun?: chưa có official benchmark.

## 2026-08-26 — Freeze Hospital ROI v1

- What changed: generate và freeze `hospital_connected_free_v1`.
- Denominator: `215435` cells.
- Mask SHA-256: `05d45b7aba66cb6dbb71e0406005f4a3e21875901af3ae72164b17f1d3add8d1`.
- Structural audit: `1040` unique wall segments, `2` elevator blockers.
- Why: coverage cần một denominator cố định giống hệt giữa Nearest và MapEx.
- Affected runs: toàn bộ run dùng protocol `hospital_v1`.
- Does baseline need rerun?: chưa có official baseline run. Sau khi baseline bắt đầu, mọi thay đổi ROI yêu cầu protocol/ROI ID mới và baseline rerun.

## 2026-08-26 — Freeze Hospital canvas and ROI definition

- What changed: chốt `hospital_canvas_v1` và định nghĩa `hospital_connected_free_v1`.
- Why: coverage cần denominator cố định, không phụ thuộc crop/map extent của từng run.
- Affected runs: toàn bộ Nearest/MapEx run mới trong lộ trình `mapex_hospital_research`.
- Does baseline need rerun?: chưa; benchmark mới chưa bắt đầu.
- Observation: full Hospital stack đã được xác nhận chạy bình thường; fixed canvas là 0.05 m, 1504 x 2123, origin (-25.6, -60.1).

## Initial note

Workspace mới được tạo để làm lại nghiên cứu từ đầu. Chưa có run official mới và chưa chốt bottleneck.
# 2026-09-23 — R004 offline prediction-quality execution

R004 V3 initially could not score the full future-observed target universe
because historical predictions have dynamic geometric support. CODEX stopped
before scoring; V4 was then independently accepted to report conditional
accuracy on scoreable support and expose unsupported coverage separately.

The final V3+V4 evaluator ran on all ten historical MapEx runs with no
exclusion or final-snapshot fallback. Results remain pending independent review
and must not be treated as a scientific prediction-quality verdict yet. Fixed
total-support and class-composition sensitivities had no eligible final-bin
decisions under the preregistered sample sizes; the evaluator reports this
without relaxing the rule.
