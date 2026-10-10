# MX071 follow-up R2 — seven-question causal execution method

Date: 2026-10-10. Status: **DESIGN CANDIDATE FOR FULL IR1 REVIEW; NO EXECUTION AUTHORITY**.

## 0. Authority, lineage and reading order

USER's direct instruction on 2026-10-10 at 13:44 +07 authorizes this drafting session to complete the new design and send it to IR. It supersedes waiting for PM pickup **for this bounded drafting action only**. The author is the original Agent1/Codex maker support, not a self-appointed PM or Research Methodologist; STAFF and formal task owners are unchanged. PM and RM07 remain informed through the existing Hub route. Scientific acquisition, model/resource probes, simulations, branches, engineering adoption and merge need their subsequent explicit gates.

Binding requirements:
- [USER R2 requirements, exact published version](https://github.com/vah103/chat-gpt/blob/bab350b7e655e4c659643aa0f3d37b073604f56e/company/projects/mapex/MX071_FOLLOWUP_R2_SEVEN_QUESTION_CAUSAL_EXECUTION_REQUIREMENTS_20261010.md).
- [Existing PM-first handoff](https://github.com/vah103/chat-gpt/blob/32d5d07ec63eac61e829644d27affa490c35676d/company/operations/HO-20261010-MX071-R2-METHOD-HANDOFF.md).
- [IR1 R1 ACCEPT](https://github.com/vah103/turtlebot4_project/pull/42#pullrequestreview-5477923365), exact prior target 5f9a60339ebd4e96ffcf115e8f9ad520ab66f405.
- [IR1 carry-forward and next gate](https://github.com/vah103/chat-gpt/blob/f730f5c984b65a3aa2a366aef228ac3416f08195/company/operations/HO-20261010-MX071-FOLLOWUP-R1-ACCEPT-NEXT-GATE.md).

R1 and all R0/readiness/old MX071 files remain byte-identical. R2 extends their accepted definitions; it does not retrospectively declare R1 executable. The older accepted MX071 R2 and its 75/80 request counts are a different experiment. The original completed evidence is PIPE_ALIGNED, not native MapEx.

Read this file, QUESTION_CONTRACTS_R2.md, RESOURCE_AND_PREFLIGHT_R2.md, METHOD_CONTRACT_R2.json, then USER_CAN_ANSWER_R2.md. The JSON is a method specification, not a runnable collection entry point. The requested review is **full affected-scope executable-method QA**, not four-finding R1 closure.

## 1. Scientific objective and frozen scope

For each open MapEx question (1, 2, 3, 4, 5, 6, 8), establish separately:
1. Is the mechanism present on nominated native states?
2. Does changing that single factor change a feasible action?
3. Does executing that action change same-budget coverage/route/latency?
4. Can the robot detect/select the correction from available observations and history?
5. Does benefit survive its computational and movement costs?
6. Is there enough independent support to justify prospective research?

Question 7 remains DROPPED_BY_USER. PIPE is audit-first and separate, with **zero** PIPE sources/branches in this contract. PIPE_AUDIT_R1.md is preserved. MX072 / New Room / COM1 / its active-deliberation R6 is unchanged.

This is a finite initial KTH cohort. There is no significance test, unseen-training claim, building-independent sample claim, global optimality claim, STOP deployment or thesis acceptance. Mechanism, action, task harm, online recovery and novelty are different conclusions.

## 2. Source, configuration, geometry and model contract

Pinned source: castacks/MapEx at 53636bd1c79153acc3c74a532837d78c926bae5e; LaMa submodule b61dcb33e063fe9586b50e2dc7b70f97d5046c1d. Source bytes read directly from git objects on DELL, without importing/running the model or simulator. Exact source-file SHA256 values and weights are in METHOD_CONTRACT_R2.json.

Resolve from pinned configs/base.yaml, with exactly these overrides: CPU; modes_to_test=[visvarprob]; eight manifest map/start assignments; headless incidental output; run-specific output root; sequential frozen model residency. No resize, crop-to-canonical-frame, one-cell movement, unknown-blocked planner, extra near-goal fallback or predicted pathwise PIPE score.

| Item | Binding semantics |
| --- | --- |
| Native loop | mission_time=1000, t in 0..999; restore original t and phase; extra shadow rescoring consumes CPU, not a source control step |
| Scoring | Ensemble G1/G2/G3 inpainted channel 0; NumPy mean; Torch sample variance with K=3 / unbiased=True; native cost is negative visible-unknown variance mass / Euclidean distance in source cells |
| G/all-train | Retain actual G computation and preliminary binary raycast cost; G is auxiliary, not final visvarprob prediction |
| Actual G auxiliary extraction | Capture its output immediately, before subsequent forwards mutate shared batch dictionaries. The preliminary map follows native visualize_prediction RGB-to-BGR and channel-0 >128 conversion; do not silently substitute ensemble channel-0 semantics |
| Predicted renderer | Pinned get_vis_mask, 250 rays, 20m, 10 pixels/m, hit_prob_threshold=0.8, full polygon/flood-fill and boundary behavior |
| Important code quirk | get_free_points resets accum_hit_prob **inside the per-cell loop**. Preserve the effective per-cell 0.8 test; do not replace it with accumulated opacity/product probability. This is a source fact, not a newly demonstrated weakness |
| Physical scan | Pinned range_libc/PyOMap path, 2500 rays, 20m; same source angular endpoints including duplicate 0/2pi, origin/order, polygon behavior and mapper accumulation |
| Planning | Source 3x3 occupied dilation; use_distance_transform_for_planning=True; dt_floor_val=10; positive weighted four-connected pyastar2d A*, allow_diagonal=False |
| Unknown | YAML requests true, but get_inflated_planning_maps invokes inflate_map(..., unknown_as_occ=False); preserve effective false |
| Controller | path[min(3,len(path)-1)] endpoint; center-cell GT collision check, then successful pose append, then scan; original valid-lock/reselection/fallback and strict <1m release |
| Source frames | KTH block_reduce 2, occupancy min / valid-space max, 500-cell raw-grid padding; Albumentations centered pad-to-16; source candidate origins remain unshifted on padded prediction tensors |
| Mutable model state | Freeze/eval, no training. Hash parameters/buffers and RNG state before/after forward. Unexpected stateful mutation blocks replay until captured, not ignored |

The paper is a primary conceptual reference ([MapEx v3](https://arxiv.org/html/2409.15590v3)); this experiment tests the pinned code and explicitly retained quirks. It is not an exact replication of every paper equation.

Native ties use the exact NumPy/source ordering and argmin. Diagnostic epsilon never changes native decisions. Nonfinite source results are retained and the native event traced; an intervention with invalid numerical support is not repaired using a new fallback heuristic.

### 2.1 Fixed cohort

Use the raw assets/start coordinates in METHOD_CONTRACT_R2.json, copied from the protected MX071 asset manifest. Raw asset SHA256 is binding; every derived source GT/valid/component hash must be recomputed by the pinned native transform and compared before acquisition. A mismatch blocks collection; do not borrow an ALIGNED prepared-map identity.

| Map | S1 row,col on source padded grid | S2 row,col |
| --- | --- | --- |
| 50052750 | 606,621 | 724,776 |
| 50010535_PLAN2 | 569,504 | 646,633 |
| 50015847 | 534,612 | 656,1155 |
| 50037765_PLAN3 | 528,531 | 584,846 |

All eight are native visvarprob acquisitions. No replacement map/start for poor outcomes or missing support. MAP_ID_ONLY / TRAIN_OVERLAP_UNVERIFIED; building identities are UNKNOWN. Starts are not eight independent buildings.

P1 outcome world is the source 0.10m raster. P2, when topology support is evaluated, is the original 0.05m KTH occupancy/valid raster with the explicit 2x transform; it is not New Room structural GT. P2 disagreements are raster labels, not prediction errors.

## 3. Shared acquisition and nominations

One native run per map/start, one lightweight immutable ledger covering every source step, scan, score, lock, retry and terminal. No diagnostic changes native inputs, goal, timing-based control or map accumulation.

Keep all R1 nominations:
- **NATIVE_SCORE_EPOCH (S):** just after native prediction/pool/cost computation, before native selection; first such event with d>=20/40/60/80 and d<=100. Several targets can alias one state; record all alias IDs.
- **OBSERVATION_CONTROL_STATE (O):** immediately after a successful scan, before the next control iteration. First bookkeeping-eligible event in [20,40), [40,60), [60,80), [80,100]. Valid native lock, a newer scan generation than the lock score-input generation, and no native rescore on that observation. Do not require new free area, GT error, rank flip or positive utility.
- Keep the immediate pre-scan execution state for each O nominee: after successful endpoint pose append, before observe_and_accumulate. It has the new actual pose and previous observed-map generation; it is not a completed observed endpoint or a scientific fork. Retain the preceding completed observation state separately. Unchanged-map scans are zeros; unavailable nominees are NA, not replaced.

Stage A therefore has at most 32 S target slots +32 O windows, with aliases retained. Initial score provenance is retained but adds no new nominated scientific slot.

Stage B uses **one slot per run per relevant stratum**:
- S1 start: S target20 or O window[20,40).
- S2 start: S target60 or O window[60,80).
This samples early and later distance strata by a fixed start-ID rule. Never substitute target40/80 because a prescribed action was unchanged, unsafe, missing or unfavourable. Record all four strata diagnostically even though only the fixed subset branches.

Additional paired eligibility: the nominated state has remaining endpoint budget R>=20m; a complete replay seal; valid source/GT frame; a legal source first action where the question requires one; and question-specific support. These checks do not use future branch utility. A selected S state first appearing at 95m stays nominated in Stage A but is BUDGET_SUPPORT_NA for Stage B.

No-divergence cases retain their complete request rows and compute costs. They are not discarded. Branch execution can be reused only after proving complete behavioral equivalence, not merely equal goal coordinates.

## 4. Complete-state schema and deterministic replay

A serialized state is a versioned object graph with typed array bytes/dtypes/shapes, alias IDs, parent-state hash, source phase and reconstruction roots. It includes:
- World/source GT and valid-space identity; exact source configuration, library/binary/env IDs, model/config hashes.
- Mapper obs_map, ordered accum_hit_points **including duplicates**, all live mapper fields, prior observation input and generation; PyOMap rebuild recipe and query hashes.
- Actual successful pose list, current pose, any attempted/collision endpoint, t, pending next t/phase, source end/failure flags and next scan generation.
- Active lock coordinates/ID, exact lock validity state, eligible and cached/unscored frontier pools in source order, filtered map/region labels, total_cost, score input and prediction generation, retry/deletion order.
- Native G auxiliary output, member channel-0 outputs, mean/sample variance, padded obs/model-input tensor+mask and true alias graph; source auxiliary maps and the complete live inference state.
- Current planned path, controller index, inflation/DT configuration and cache; previous successful scan/input; model frozen state/buffers; Python/NumPy/Torch RNG state and deterministic execution context.
- Event order, endpoint and planned-arc clocks, measurement timers separated from source decision state.

A liveness inventory must map every source read to a serialized field or an exact decoder recipe. Incidental plot buffers, filenames, OS PIDs and real wall timestamps can be excluded only with a documented no-control-consumer proof; no map-plus-pose shortcut. An undocumented field is STATE_COMPLETENESS_FAIL.

Before any scientific alternate, reconstruct **KEEP for every one of the at-most-16 Stage B S/O states**, through the full remaining suffix, and compare with the original source acquisition:
- every pose/path, ordered candidate/pool/cost, scan/hit ordering, obs bytes, lock/retry/release, score-input/member/mean/variance, native step and terminal;
- exact numeric bytes on the pinned deterministic CPU configuration; wall-time/PID differences are telemetry only;
- primary Q/C and crossing records.
Any mismatch stops all affected branches. This is stronger than the old four/five-step stand-in fixture.

Decoder inference caching is permitted only by exact model/config/input/dtype/device/library/RNG identity. Recomputing a source output is permitted after byte verification; copying future baseline **scans** into an alternate is forbidden. Each alternate executes its own control, obtains its own physical scan and evolves its own mapper.

## 5. First-decision intervention and common continuation

The per-question chooser reads immutable copies. Diagnostic treatment arrays never modify the source acquisition. The branch starts from a complete clone, same P1 world, same t, same actual observed state, same remaining 100m budget and source step budget.

**S-state one-first-goal contracts:** compute each question's declared winner with its prescribed scorer/pool. Selection/revalidation follows source order, observed inflation and source A*. Record invalid/reselected proposals. No GT-collision lookahead filters a goal unless GT is explicitly the oracle input for that question.

Once a different legal goal is chosen:
- Preserve the native input, pool and cached values as provenance. Compute native cached dispatch order by repeatedly applying the source NumPy argmin/delete rule to a copy of its costs, preserving its exact tie/nonfinite semantics. Move the declared legal chosen candidate to the front of that order, leaving every other candidate in native order. Install float64 ordinal costs0..F-1 by this permutation as one labelled INTERVENTION_DISPATCH_PRIORITY; use the source retry/delete implementation. The unchanged NATIVE arm keeps its original numeric costs.
- This overlay and its exact priority array are part of the declared first-decision treatment, not relabelled as native cost. At a native scoring epoch, the untouched KEEP branch uses its original exact cost array.
- Retain the native commitment until goal reached/invalid/unreachable under native rules; no repeated intervention, fixed-time persistence, extra scan or forced goal-reaching.
- At the next native rescore after that commitment/fallback, remove the overlay and use unmodified visvarprob forever.
- If winner, pool and relevant cache are exactly the native contract, do not insert a priority overlay. Mark INITIAL_ACTION_SAME and verify full suffix equivalence before assigning structural zero.
- Pool expansion is only #6; fixed-pool #1/#3/#4/#8 cannot import #6 candidates.

**O-state contracts:** #2 uses symmetric forced decision opportunities to isolate prediction freshness, not KEEP-versus-rescore; #5 uses actual source KEEP versus a single safe fresh decision opportunity. Their exact cache/persistence rules are in QUESTION_CONTRACTS_R2.md.

Paths are replanned by the native weighted A* each source step. The previously saved path is not followed blindly after map updates. First-goal release/failed attempts/near-goal cases remain part of the outcome. A branch cannot be rescued by GT selection or additional retargeting.

## 6. Budget, geometry, movement and Q

Let d_i be cumulative **successful endpoint-chord displacement** between source appended poses (0.10m/cell), starting at d_0=0 after the initial scan. This is not old MX071 moves/10.

Primary B=100m: the latest complete successful pose+scan with d_i<=B. An exactly-at-B completed scan is included. If the next native step crosses B, execute at most that one source-order crossing step when the native loop permits it; retain actual endpoint/scan/overshoot solely as POST_CROSSING_ENDPOINT sensitivity. No artificial partial pose/scan or interpolation.

For x<=B, C_B(x) is coverage at the latest completed scan with d_i<=x, holding that observation between endpoints. At equal d use the last completed observation at that d. Primary data never credit a scan beyond the cutoff.

W is the fixed initially reachable **four-connected P1 GT-free and valid-space component** of that start, before any scan. N=|W|>0, hash-bound. Primary:
C(s)=count((obs_map!=0.5) AND W)/N.
Also report observed-free-on-W and false occupied/free labels; coverage alone is not prediction correctness. All source events use the same denominator. Policy never receives W or GT labels.

For a fork s at d_s<B, R=B-d_s:
Q_R(s,a)=(1/R) integral from u=0 to R of [C_a(d_s+u)-C(s)] du.
Integrate exact piecewise endpoint-held intervals; retain zero/negative gains if present. Duplicate-distance scans do not invent travel. Q is dimensionless; 0.01 means one percentage point in mean additional coverage over the remaining endpoint budget.

Each arm has the same absolute B and remaining **source iteration count** from its original phase. Extra treatment inference does not buy more source steps. An early native terminal is a labelled absorbing outcome: carry its last completed observation to B. This does not assert it physically travelled the unused distance.

Also record:
- C at last primary endpoint, residual prefix gap, source and actual crossing displacement, planned A* prefix arc length and source retry/failure counts.
- Distance to preregistered absolute coverage targets 0.50/0.75/0.90, and C(s)+0.05 if <=1. First complete in-budget scan reaching the target; already-reached is zero/undefined percentage saving. Unreached is right-censored, never replaced by B or a best-case time.
- Revisit endpoint/path-prefix length from repeated directed edges, and time/compute below.

### 6.1 Honest time and cost contract

The native pseudo-controller has no physical velocity or yaw dynamics. Report separately:
1. Measured cold/hot model load, forward, rendering, scoring, A*, physical-scan CPU/wall, source policy wall and process CPU seconds.
2. Recorder/codec/replay/research shadow and branch wall, excluded from a proposed online policy's latency but included in research resource totals.
3. **Standardized virtual motion time**, not measured robot time: traverse the actual selected native A* prefix at v=0.20m/s; stop-and-turn at each cardinal heading change with omega=0.50rad/s; initial heading 0; add 0.10s per executed scan. This bookkeeping does not alter native selection or scanning.
4. Validate the traversed prefix against P1 GT center cells and the declared source 3x3 footprint. A skipped/intermediate collision or clearance violation is MOTION_CONTRACT_GAP. Native coverage/chord outcomes remain, but physically meaningful virtual-motion/net-time comparisons are NA; do not fix the native controller.
5. When valid, T_eff = virtual motion+scan time + measured **serial source-policy/treatment computation**, including sequential CPU reloads. This is a declared DELL stop-compute-move clock, not ROS/Gazebo/robot time. Sensitivities v=0.10/0.30 and omega=0.25/1.00 reuse the same trace, adding no simulation requests.

Oracle generation is research cost. Any claimed deployable replacement must charge its actual online detector/selector cost; an oracle GT method has no supported online T_eff. Simulated time, actual study elapsed time and native step counts are not interchangeable.

## 7. Missingness, failures, aliases and finite-population analysis

Every logical slot/request has a disposition; no silent row loss. The full registered request table exists before dedup/Stage B execution.

| Disposition | Primary treatment |
| --- | --- |
| Equal executable contract, verified full behavioral identity | Structural zero Q/travel, retain actual extra selector CPU; one physical replay may serve both requests |
| Different labels/targets for exact state | Alias reference, one evidence unit; no extra N |
| No qualifying S/O state, R<20, empty common support, absent GT topology patch/control, insufficient #8 actions | NA with reason; cannot become a negative mechanism finding |
| Native no-frontier/exhausted candidates, center collision, original source step limit | Native absorbing terminal; complete Q by last-observation hold; preserve raw source failure flag and reason |
| Freshly recomputed empty native pool raises the pinned score_frontiers empty-shape ValueError | NATIVE_EMPTY_POOL_EXCEPTION absorbing failure; retain traceback/state and count behavioral failure. It is not relabelled as normal completion |
| Invalid alternative numeric/path/pool/renderer | INVALID_TREATMENT with explicit provenance; no convenient goal substitution; causal contrast missing/inconclusive |
| Infrastructure/OOM/codec/disk/time/identity/technical guard | TECHNICAL_ABORT; primary Q missing; keep observed prefix, costs, error and bounds; no absorbing success or zero substitution |
| Cross-budget scan | Sensitivity only; never primary endpoint/target/Q credit |

Automatic infrastructure retries = **0** in this candidate. A later correction/retry needs a bounded reviewed amendment; all attempts remain in the registry.

The fixed P09 classification is: loop-guard empty unscored pool -> NATIVE_NO_FRONTIER with raw frontier_region_no_large_region flag; native retry/delete exhausts the pool -> NATIVE_NO_REACHABLE_FRONTIER with raw frontier_region_centers flag; new native pool has length0 and score_frontiers throws the empty-array broadcast ValueError -> NATIVE_EMPTY_POOL_EXCEPTION; endpoint GT==1 -> NATIVE_COLLISION with raw hit_wall; ordinary loop exhaustion -> NATIVE_SOURCE_STEP_LIMIT, retaining its original mission_complete flag. A zero/one-node path is a source no-move/scan event and consumes the original iteration; do not add old ALIGNED STUCK50 termination.

All other exceptions, including model/library failures, unexpected IndexError/ValueError and interactive renderer traps, are TECHNICAL_ABORT / SOURCE_CONTRACT_GAP, with missing Q. P09 tests this fixed table; engineering cannot choose a classification after seeing Q. No extra scientific retry or silent source fix is implied.

Coverage missing after technical abort: retain observed integral L and bound the remaining interval using 0<=C<=1 (or the proven monotone envelope only if independently validated). Q bounds can be negative down to -C(s); never assume a favorable monotone upper bound to rescue a claim. Different-arm bounds propagate by interval subtraction. Coverage-target time/distance saving under censoring is an interval/NA, not a point estimate.

Aggregate nominations within start for descriptive frequency; Stage B has one primary nominated test per start. Average start effects within map, then equal-map macro across the four maps. Report each run/map, worst loss, every failure and support denominator. Oracle/random/placebo arms are controls, not independent replicates. No snapshot pooling or independence inflation.

Use different denominators explicitly: nominated slots -> unique states -> eligible mechanism support -> distinct legal action contrasts -> complete paired outcomes -> virtual-time-valid pairs. A reduction anywhere is a finding, not an exclusion hidden in the headline.

## 8. Preregistered decisions and falsification

These are ex ante **finite-cohort research-screening criteria**, not statistical population confidence. A one-percentage-point Q yardstick is scientifically interpretable and matches the existing project's material-utility scale; it is not fitted to new MapEx outcomes. Five-percent time/travel and 0.5pp final-coverage harm margins are explicit cost/safety tradeoffs proposed for IR1, not a promised positive result.

Numeric precision: prediction epsilon=1e-6; diagnostic score epsilon=1e-6*max(1,max absolute score); Q comparison epsilon=max(1e-6,0.5/N). Native argmin remains exact. #8 gain ties use integer-cell resolution as specified separately.

| Label | Prospectively required evidence |
| --- | --- |
| SUPPORTED_MECHANISM | At least one valid nominated mechanism difference above its precision tolerance; report finite frequency and support, no task-benefit implication |
| SUPPORTED_ACTION_MISMATCH | At least one valid change of the tested executable first-decision/commitment contract, with unchanged action/cost cases retained; goal/rank alone insufficient |
| SUPPORTED_CAUSAL_TASK_HARM | Valid single-factor/control-specific contrast; four map means present; >=6 complete non-equivalent map/start contrasts across >=3 maps; macro deltaQ>=0.01, >=3 positive map means; no behavioral-failure increase; no map mean final-coverage loss >0.005. #3 adds its specificity contrasts. This is same-distance task harm/headroom at nominated states |
| SUPPORTED_ONLINE_RECOVERY | Previous criterion plus no oracle/future information in detector/chooser; measured selector cost; valid virtual-time support in >=6 runs across >=3 maps; macro paired T_eff reduction>=5%, or >=5% matched-target arc saving with T_eff nonworsening; safety conditions remain. One-off feasibility is not a repeated-policy/prospective win |
| NOT_SUPPORTED | >=6 eligible complete run contrasts and all four maps; the preregistered material-utility gate fails with no unresolved technical missingness that could change it. Means not promoted on this cohort, **not population proof of no effect** |
| NO_EXECUTABLE_EFFECT_ON_TESTED_SUPPORT | Exact full behavioral equality on every eligible nominated contract, including zeros and cost; conditional source-support statement only |
| INCONCLUSIVE_INSUFFICIENT_SUPPORT | Any material eligibility/control/replay/frame/time gap, insufficient independent support, missing map, or failure bounds spanning the material threshold |
| MECHANISM_PRESENT_BUT_NON_ACTIONABLE | Mechanism supported, but correction depends on GT/future utility or no feasible online detector; may coexist with measured oracle headroom |

Supported mechanism/action and NOT_SUPPORTED utility can coexist. A harmful correction is reported as WORSENS_TESTED_TASK with its negative effects, never retuned into a winner. A required unsupported threshold or unsealed field makes METHOD_NOT_READY; do not tune thresholds after Stage A/B outcomes.

These gates do not guarantee absence of a rare weakness. Strong thesis promotion also needs a separately reviewed literature comparison and prospective validation on verified independent data.

## 9. Stage gates and scope choices

A0: independent full R2 design review, then explicit engineering/preflight authority.
A1: full-size source/model parity, complete-state codec/replay and resource contract tests in RESOURCE_AND_PREFLIGHT_R2.md; no scientific collection under a fixture label.
A2: separately authorized shared eight-source acquisition and all-seven diagnostic screen. Publish support/NA and pre-outcome exact Stage B request registry; complete KEEP seals.
B: only after method/preflight/acquisition validity and explicit B execution authority, run the preregistered bounded pairs. Eligibility/action divergence and exact alias checks can save work; never choose by observed Q.
C: CLOSED. At most one supported, actionable mechanism is nominated for a separately reviewed prospective repeated-policy validation design. No current Stage C trajectories or training.

Resource specification gives **176 scientific first-decision requests +16 complete KEEP replay requests =192 replay requests before any dedup/NA**, plus eight acquisitions. Native semantic step limit is 1000; technical per-trace inference ceiling is separate and produces technical censoring. No silent borrowing of the previous 80 ceiling.

Full-scope and staged-core options are costed separately. The current USER requested FULL_7 design; the staged option is only a PM/USER alternative if resources fail, not automatic narrowing. See USER_CAN_ANSWER_R2.md for exactly what would remain unanswered.

## 10. Current implementation and review status

R2 is a concrete design candidate. No new real-model probe, fixture, source trajectory, alternative simulation or scientific branch has run during drafting. Current DELL metadata exposes inadequate free disk for either proposed execution option; resource feasibility is **uncertified**, not a PASS.

Return exact PR #42 head + document/blob hashes to IR1 successor04 for independent full-scope review. Preserve R1 ACCEPT; no self-issued ACCEPT, READY, START, merge or execution decision. The accepted R1 semantics, old MX071 data and separate MX072 chain remain intact.

