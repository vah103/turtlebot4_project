# Seven executable question contracts — MX071 follow-up R2

This file is part of PLAN_R2.md. All questions inherit its complete-state replay, source renderer/controller, 100m endpoint-held Q, original remaining 1000-step budget, failure/NA/alias analysis and full IR1 review gates. All actions below are **designed, not executed**.

## Summary and explicit per-question request caps

S means the fixed Stage B native-score nominee; O means the fixed active-lock post-scan nominee. There are at most eight nominees of each type, one per map/start. One request runs both its 10m diagnostic (when applicable) and remaining-budget outcome; these are not two rollouts.

| Question | Diagnostic | Matched arms actually continued | Outcome/cost | Allowed claim | Ceiling before dedup/NA |
| --- | --- | --- | --- | --- | --- |
| 1 Aggregation | Raycast(mean) versus mean(member raycasts), U/V/input/pool/d fixed | NATIVE; MEMBER_VIS_MEAN | Same-budget Q/C/route; 3 versus 1 probabilistic masks per candidate | Source aggregation action effect; online-visible one-off correction | 8x2=16 |
| 2 Observation update | Immediate pre/post ensemble, common still-unknown support; error separate from change | PRE_UPDATE_DECISION; POST_UPDATE_DECISION, both at post-scan state with symmetric fresh pool/opportunity | Q/C/route and full inference cost; KEEP relation reported separately via #5 | Causal effect of using newly conditioned prediction at a matched decision, not learning or an expected-future-update policy | 8x2=16 |
| 3 Topology | P1 erroneous barrier/door/connection port relation; P2 raster qualification | NATIVE; TOPO_REPAIR; NONTOPO_REPAIR; RANDOM_NEUTRAL_REPAIR; ROUTE_DENOMINATOR | Full Q/route; repair specificity and cost | Local GT-topology headroom if matched controls qualify; no online topology oracle | 8x5=40 |
| 4 Visibility | Native mean versus predictor-only GT substitution; physical renderer gap separate | NATIVE; UNKNOWN_GT_VIS | Q/C/route and renderer cost; fixed V and d | Matched visibility oracle headroom; not practical online recovery | 8x2=16 |
| 5 Commitment | Valid locked post-scan state, stale native scoring input | KEEP; FRESH_ONCE; FRESH_SHAM_KEEP | Q/C/revisits/travel; fresh inference/replanning/turn time | Causal one-off safe reconsideration on online-visible data, with compute-only control | 8x3=24 |
| 6 Viewpoint | Representative versus three deterministic proposals per source cluster | NATIVE; IN_CLUSTER_BEST; EXPANDED_POOL_BEST; CHEAP_IN_CLUSTER | Q/C/route; enlarged-pool rendering/planning cost | Fixed-cluster representative effect versus separately labelled pool/cluster effect | 8x4=32 |
| 8 Horizon | Realized G10/Q10 versus Q_remaining for distinct legal proposals | A0_NATIVE; A1_NEAREST; A2_MAX_MASS; A3_FARTHEST | Tested-action regret, unique/tied reversals, full route/cost | Limited tested-action positioning headroom; native short-horizon weakness only with source-action alignment | 8x4=32 |

Scientific requests: **176**. Complete KEEP suffix seals: **16**. Total replay requests: **192**, independent of how many aliases/zeros save physical work. Eight source acquisitions are additional. No seven independent baseline batches.

## 1. Aggregation before nonlinear visibility

**Hypothesis.** Threshold/flood-fill on a mean occupancy prediction can disagree with averaging the visible masks from individual predictions. The disagreement matters only if it changes an executable first decision and task outcome.

**Exact native mechanism.** At S, source probabilistic get_vis_mask(mean_map, native candidate origin, range=200 cells, num_laser=250, threshold=0.8) returns flooded_grid; visible means flooded_grid==0. Intersect with source padded obs==0.5 and sum the native sample variance, normalized by source Euclidean distance. Preserve the per-cell threshold reset and flood-fill code; there is no substituted analytic visibility model.

**Nomination/support.** All S strata diagnostically; fixed Stage B S slot for continuation. Require complete member/mean/V arrays, valid fixed pool/frame, R>=20 and a legal source action. Zero V, equal masks, equal actions and native retry cases remain.

**Changed factor/control.**
- NATIVE: Phi_mean(j).
- MEMBER_VIS_MEAN: compute Phi_i(j) with exactly the same renderer on each G_i channel-0 tensor; define weight Phi_bar(j,p)=count_i(Phi_i(j,p))/3. Use **frozen native V**, U, pose, distance and candidate pool.
- Sum V*Phi_bar on nonzero-weight U cells in the same row-major order as the native indexed reduction. Use Torch float32 weights and native cost conversion; do not accidentally average member-specific variance or harden the averaged mask.
- When all three masks equal the mean mask, the implementation must reproduce native mass bytes; dense reduction over inserted zero entries is not an acceptable changed numerical contract.

**Action/execution.** Both choosers follow the source validity/A* retry order. Execute their first-goal contracts and common native MapEx continuation to the same B and source-step remainder. No endpoint snapshot comparison without motion.

**Diagnostic and metrics.** Per candidate: visible-mask symmetric difference, weights histogram, variance-mass/cost/rank, initial/executed action mismatch; full Q/C, endpoint/arc travel, revisits, failures, virtual-time-valid support. Charge variant three probabilistic renders rather than native one, retain native G preliminary render, and separately charge acquisition/paired diagnostic overhead.

**Verdict/limits.** Mask or score change -> mechanism only. Task harm/recovery uses PLAN_R2 gates. A valid all-equal replay cohort can yield NO_EXECUTABLE_EFFECT_ON_TESTED_SUPPORT; a small Q difference fails material promotion without disproving nonlinear aggregation in other structures. This tests KTH source support, not New Room complexity or all possible map ensembles.

**Resource cap.** Eight x two requests; Stage A <=32 S nominees, <=4 probabilistic renders/candidate for paired diagnostic plus the native preliminary call. F_native<=256 is a technical bound, not a pool truncation.

## 2. Observation-conditioned global inference update

**Hypothesis.** A newly acquired scan changes the global ensemble prediction outside newly observed cells, sometimes improving or worsening subsequent decisions. This is inference with fixed weights, not online learning.

**State/diagnostic.** At each O nominee, save S_pre immediately before the successful scan, after the new endpoint was collision-checked/appended, and S_post immediately after it; use the **post-scan actual pose** in both diagnostic render/score copies. S_pre is a complete PRE_SCAN execution state with the previous observed input, not a completed observed endpoint/fork. Record raw pre/post G1/G2/G3, mean/V and actual G auxiliary output separately from the older native cached scoring input.

Let U_common = unknown(pre) AND unknown(post) AND NOT newly sensed cells, in the exact padded tensor frame. GT-valid evaluation uses U_common intersect transformed valid-space. Empty support is NA; unchanged input and zero prediction change are valid zeros. Verify unknown(post) is a subset of unknown(pre); violation is MAP_ACCUMULATION_GAP, not a silently corrected support.

Freeze current pose, **post-observation native candidate pool**, native distance, ray origins/settings and U_common. V is allowed to change with the ensemble, because that is part of the tested update. Shadow values are MATCHED_SUPPORT_SHADOW; do not overwrite the actual native cache.

Report, separately:
- Per-member and mean absolute change, V change, precision-level change frequency.
- GT MAE and Brier residuals, threshold-0.8 false occlusion/free errors on the exact same common support; GT accuracy never selects the action.
- Visibility/mass/cost/rank and executable chooser differences.
An accuracy improvement does not imply a better action; a changed action does not imply utility.

**Matched causal arms.** Both fork the same **post-scan complete state** and are given one symmetric fresh decision opportunity:
- PRE_UPDATE_DECISION uses the ensemble predicted on the immediately pre-scan input on U_common.
- POST_UPDATE_DECISION uses the ensemble predicted on the post-scan input.
- On already observed/newly observed/padding-known cells, both use identical post-input known values via the source inpainting input/mask convention. No historical newly-known occupancy remains in a ray to confound outside-view updating.
- Compute mean and sample V from each arm's unknown member tensors; keep native current known values fixed. Source padded U_post and U_common must match on score support; otherwise mark support gap.
- The same post-scan pool is recomputed once on observed geometry, and the same validity/controller/sensor contract applies. The difference is **prediction conditioning age used by the chooser**, not one arm having a rescore opportunity and the other having a lock.
- The common post-decision continuation input/cache contract is native post-state scoring provenance with the arm's labelled chosen first priority. Future native score epochs always infer on that branch's actual new observed map. Do not reuse pre-scan future scans.

**Online/cost boundary.** Pre observation/history and post observation are robot-visible. These arms isolate whether using an update can affect a decision; they do **not** establish the native cached-goal benefit. #5 is the separate actual KEEP-versus-fresh decision test. There is no expected-future-update selector or GT-hindsight expected learning benefit in R2. Charge keeping the previous input/prediction and the current inference; do not call offline shadow outputs free online computation.

**Stopping/conclusions.** Prediction numeric epsilon=1e-6; report error changes without a positive-only eligibility filter. Common task gates for PRE vs POST; POST can worsen Q and accuracy. Empty common/GT support, missing legal fixed-pool action or uncontrolled cache difference gives INCONCLUSIVE, not evidence that updates do not matter. No weights update, training or calibrated uncertainty claim.

**Resource cap.** Eight x two requests. At most 32 O nominees x two full four-model input batches in Stage A before exact-input reuse. Native stale cache versus immediate-pre prediction has an explicit age/input ID, not another rollout.

## 3. Topology and connectivity with dose-matched controls

**Hypothesis.** Local incorrect predicted barriers/doorways/connections influence scoring and produce task loss beyond equally sized non-topological prediction errors or a simple travel-denominator change.

**Important source limit.** Native A* plans on **observations**, not the predicted mean. A predictor repair can change visibility/goal choice and later observed routing; it does not directly edit planner obstacles or the physical world.

**Predeclared taxonomy.**
- FALSE_BARRIER / MISSED_DOORWAY: GT-free boundary ports connect in P1, but the prediction's blocked cells disconnect them.
- FALSE_OPENING / HALLUCINATED_CONNECTION: prediction connects a GT-separated port pair.
- MISPLACED_CONNECTION: both missing and false relations within the patch.
- PIXEL_ERROR_TOPO_NEUTRAL and RASTER_TOPOLOGY_DISAGREEMENT are separate labels.

Use four-connected free-port reachability, prediction blocked at mean>=0.8 (native per-cell ray threshold). A 0.5 graph is an explicitly secondary descriptive sensitivity, not another intervention. A port is a contiguous GT-free boundary run of >=3 P1 cells (>=0.3m); use all port pairs. Observed cells are identical anchors, never repaired.

**Outcome-independent patch nomination.**
- At each S diagnostic, enumerate 21x21 P1-frame patches centred at P(input-frame) of the first <=64 native cluster representatives in native order. Preserve all remaining native candidate outputs; this is a bounded topology-screen region, not a full-map search.
- P is the actual centered padding transform. Semantic topology/GT comparisons use P; source ray origins remain their original unshifted coordinates.
- Ignore duplicate exact patches by alias; determine port relation mismatch using GT, without branch Q.
- For Stage B, select the first qualifying patch in that predetermined order, not the largest utility uplift.
- Require GT-valid patch/ports and the matching 0.05m raw P2 port relation after explicit 2x mapping. A source/raw raster disagreement remains P1-specific evidence but does not qualify the topology attribution gate.
- Correction set E is exactly **all** unknown AND GT-valid cells in the selected patch where (mean>=0.8) differs from GT occupancy; size1..256. No smallest-cut/culprit search is performed. Verify that correcting this whole set restores the intended GT port relation. Oversize/unresolved patches are NA, not cropped to a favorable dose. This is a topology-carrying patch repair, not proof that every repaired pixel individually caused harm.

**Five arms, identical initial state/U/V/pool/d/source control.**
1. NATIVE.
2. TOPO_REPAIR: set mean to the source-grid GT 0/1 on E only; keep known cells, all other mean values, native sample V, distance and planning map unchanged. Re-render the **whole** mask/rays; do not patch only IG totals.
3. NONTOPO_REPAIR: correct exactly |E| GT-wrong unknown cells from the preregistered ROI union excluding E, without changing any checked port relation. Match occupied-to-free/free-to-occupied counts, abs(mean-GT) bins of width0.1 and distance-to-native-winner bins of1m. Process bins lexicographically; choose the first required number of row-major individually-neutral cells in each bin, without replacement; verify neutrality of the union once, with no backtracking. Require total L1 correction dose within0.05*|E| of E. Failure gives CONTROL_SUPPORT_NA, not a different matched control search.
4. RANDOM_NEUTRAL_REPAIR: same counts/bins/dose, seeded sampling without replacement from that GT-wrong neutral pool; NumPy RandomState/MT19937 seed is the uint32 value of the first8 hex digits of SHA256(state_id+"MX071_R2_TOPO_CONTROL"). Iterate matching bins lexicographically and candidates row-major; sample once per bin without replacement, then union. Verify the selected set is topology-neutral **as a whole**. One draw only; no redraw to improve Q or to make a convenient control pass.
5. ROUTE_DENOMINATOR: no pixel changes. Replace Euclidean-cell denominator by the full geometric length, in cells, of the observed-map **native weighted-A*** route; keep visibility/V/pool fixed. Weighted path search cost and geometric path length are not interchanged. No-path candidates retain explicit invalid support.

Non-topological/random controls are dose-matched; route-denominator is a separately labelled travel-cost control, **not** falsely called pixel-dose matched. No matching variable uses future outcome. If a dose control cannot be constructed, that contrast is CONTROL_SUPPORT_NA; native-versus-topo can still describe oracle headroom but cannot establish topology specificity.

**Action/continuation/outcomes.** Generate source-legal first decisions for each arm, then actual native scans/common continuation. Record changed-pixel dose, port confusion matrix, variance mass/rank, chosen action, downstream trajectory/coverage/revisit and cost. GT repair is oracle-only and never installed in a deployed model or mapper.

**Specificity gate.** In addition to PLAN_R2 task support, TOPO_REPAIR must exceed each of NONTOPO_REPAIR and RANDOM_NEUTRAL_REPAIR by macro deltaQ>=0.005 with >=3 positive map contrasts and required complete support. If ROUTE_DENOMINATOR reaches comparable utility (difference within0.005), report NON_UNIQUE_CAUSAL_EXPLANATION; do not market topology as uniquely isolated. Missing controls or mixed raster support -> INCONCLUSIVE for specificity.

**Online/novelty limit.** Positive GT repairs alone give localized structural headroom. A GT-free contradiction/risk detector and prospective correction selector remain Stage C requirements. MECHANISM_PRESENT_BUT_NON_ACTIONABLE is valid even for a large oracle result.

**Resource cap.** Eight x five requests; <=64 patches per S; patch441 cells; repair<=256 cells; one seeded random control, no seeds sweep.

## 4. Prediction-dependent visibility/gain error

**Hypothesis.** Wrong unknown-cell occupancy values in the native mean distort the same-renderer visible-variance score enough to cause exploration loss.

**Nomination/support.** All S diagnostically, fixed S for pairs. Same source pool/origin/pose, U, V, Euclidean d, source preliminary G ray path and full flood-fill. Exact source P1/padded alignment is mandatory.

**Factor/control.**
- NATIVE uses native mean.
- UNKNOWN_GT_VIS replaces mean **only on unknown AND GT-valid P1 input-frame cells** by padded source-grid GT0/1; all known/padding-invalid values stay identical. Keep V fixed, including zero/low-V regions. Do not replace the entire observed map, variance, A* obstacles, candidate pool or controller.
- Recalculate all ray hits, polygon, visibility, variance mass, cost/rank. A direct numeric IG correction or GT-visible-area goal without the source renderer is not this arm.

**Layer separation.**
- Compare native predicted250 mask to UNKNOWN_GT_VIS250 on the same padded source origins/settings to isolate predictor substitution.
- Separately compare matched GT250 to actual physical2500 visibility after mapping the raw physical mask through P. Retain source unshifted-origin discrepancy, angular/range/raster/mapper differences as EXECUTION_SENSOR_CONTRACT_GAP.
- These residuals are not charged to prediction. Source-fidelity does not mean renderer-vs-physical parity.
- Report visible-unknown count and frozen variance-mass false-positive/negative contributions. Predicted weighted IG is not measured m2 or a sensor probability calibration.

**Execution/cost.** Execute NATIVE and UNKNOWN_GT_VIS first decisions and common native MapEx continuation. A source-legal goal can still collide physically; retain it. Compare Q/C/route/T where valid, plus generation/render costs. GT substitution is non-online, so never return SUPPORTED_ONLINE_RECOVERY for this oracle arm.

**Verdict/limits.** Oracle Q material uplift establishes tested visibility headroom only after replay/frame/control support. No practical MapEx weakness is established by shadow-score error alone. No action change/full suffix equality yields executable zero, with oracle compute still recorded. A physical sensor-contract gap prevents attributing all actual gain error to occupancy prediction.

**Resource cap.** Eight x two requests; <=32 S diagnostic native/oracle pairs; no expanded candidates or model retraining.

## 5. Active locked-goal reconsideration

**Hypothesis.** After a new observation, native MapEx can retain a still-valid goal without rescoring; a single safe fresh decision can sometimes save later exploration cost.

**Exact native mechanism.** Validity is the actual source center occupancy plus strict <1m rule. The next source iteration otherwise keeps the cached goal/cost/input. Native validity is not a prediction-rank check. R1 O nominations are preserved even if the currently locked A* route is unavailable; the additional paired action-support test then records NA, not a new favorable nominee.

**Eligibility.** At the fixed O nominee: current native lock still source-valid; new scan generation since score input; no native rescore on that scan; source KEEP has a nonempty executable path under the current observed map; R>=20; complete replay. Do not use GT, new-area threshold or a predicted rank flip for nomination.

**Three arms.**
- KEEP: exactly restore source state and finish the current loop/next native t without a refresh.
- FRESH_SHAM_KEEP: on a read-only copy, perform the exact four-model prediction, current frontier extraction, scorer and source-valid/path checks; record costs and recommendations, then **discard the copy**. Keep original goal, old native cache/pool/RNG/mapping state. This is a computation-only control.
- FRESH_ONCE: do that same one read-only fresh decision. If no finite legal candidate survives the native checks, retain the complete original KEEP state and label NO_VALID_RETARGET_KEEP, charging all compute. If winner is the incumbent, retain original control/cache and label FRESH_INITIAL_ACTION_SAME. If winner differs, atomically install its fresh native pool/cost/input cache and valid lock with a labelled intervention score epoch. This is the tested one-time fresh decision, not a native score event.

**Persistence/safety.** New valid goal normally executes at least one source step, then the exact native reached/invalid/unreachable release/fallback rules apply. No extra minimum metres, indefinite persistence or forced arrival. At most one authorized fresh decision per branch; subsequent scans never retrigger this intervention. A new path is replanned each native step; no extra scan before movement. No repeated retargeting/oscillation loop is introduced; record source alternating-goal/revisit patterns and failed revalidation.

FRESH_ONCE vs KEEP measures practical one-off benefit plus added cost. FRESH_ONCE vs SHAM isolates the executed decision/cache change with matching refresh computation. KEEP vs SHAM must have byte-identical behavioral suffix and Q; a failure is SHAM_STATE_CONTAMINATION_FAIL and stops #5.

**Metrics/decision.** Cache age/input ID, original/fresh pool/goal, lock release reason and successful steps to release, Q/C, endpoint/arc/revisit length, failures; model/render/replan/virtual turn cost. A rank flip without improved executed Q/cost is not harm. Use common task gate and online-recovery cost gate; 100m/source-step right-censoring limits claims about journeys beyond this prefix.

**Resource cap.** Eight x three requests. Refresh at most once; the SHAM refresh is counted even when its future control trace is reused from KEEP.

### Worked #5 request — execution contract, not a measured result

For 50052750/S1/O_[20,40), capture a hypothetical eligible complete S_post after scan g+1, with native lock L scored at input g0<g+1, native t=k already completed. Do not invent coverage or a new score.

1. Full KEEP replay resumes at the saved next-source phase and must reproduce the source suffix.
2. Both fresh arms take immutable clones of S_post and run the same current-input scorer once.
3. If fresh recommendation L'=L, both retain KEEP control bytes, but their extra model/renderer cost remains.
4. If L' differs and passes native observed-map/path checks, FRESH_ONCE installs the fresh cache/lock; SHAM discards it and keeps L.
5. Each physical branch moves using path index3, checks endpoint collision and scans its own world/map. No baseline post-fork scan is copied.
6. Integrate each independent endpoint-held trace to absolute100m; record the crossing separately. A long/failed locked goal does not buy extra budget.
7. Report all three outcomes. Without real trajectories there is no Q improvement, savings or verdict to fill in.

## 6. Cluster representative and alternate viewpoints

**Hypothesis.** A source cluster's nearest-to-mean representative can miss useful views; changing the viewpoint, or explicitly expanding the pool, may improve actual exploration.

**Native mechanism.** Frontier cells are observed free adjacent to unknown via the source8-neighbour convolution; source connected-region labelling is8-connected; region size strictly>10; representative is the region cell nearest its arithmetic centroid, with exact argmin ordering.

**Proposal rule, no GT/future outcomes.** For every native source region at S, predeclare up to three cells:
1. Lexicographically smallest (row,col) region cell.
2. Lexicographically largest.
3. Region cell nearest the current pose in Euclidean distance; lexicographic tie.
The native representative remains. Exact duplicates become alias rows, not replenished candidates.

For each proposal, require source valid-lock distance>=1m, not inflated occupied, and a nonempty source weighted-A* path. For alternate-viewpoint eligibility additionally require the **actual source-selected path** to lie wholly in observed free, non-inflated cells; do not force an unknown-blocked replan to rescue it. Invalid/near/unknown-route proposals are retained with reasons and **no refill search**. Physical GT is not consulted to construct this pool.

**Four arms.**
- NATIVE: original source-validated first goal.
- IN_CLUSTER_BEST: fix the source-winning cluster; choose by native visvarprob score among its representative and valid proposed alternatives. This isolates within-cluster viewpoint selection.
- EXPANDED_POOL_BEST: same source scorer over all representatives + eligible alternatives, then native selection/revalidation. Enlarged pool and possible cluster switch are explicit; this is a separate contrast, not a fixed-pool experiment.
- CHEAP_IN_CLUSTER: fix the source-winning cluster and choose the shortest **actual weighted-A* path's geometric length** among valid non-representative proposals, source-order tie; if none, retain source goal with EMPTY_ALT structural-zero reason. No prediction visibility used by this cheap selector.

Keep U/V/predictions, scorer, sensor, source planner/controller and continuation identical. Tie the scoring chooser to native incumbent first for equal values, using an explicitly labelled stable treatment order; never choose by realized GT Q.

**Outcome/causality.** IN_CLUSTER_BEST vs NATIVE isolates source representative loss. EXPANDED vs IN_CLUSTER separates extra pool/cluster opportunity. CHEAP vs NATIVE tests whether a simple travel choice explains improvement; prediction-specific promotion requires IN_CLUSTER_BEST to exceed CHEAP by macro Q>=0.005 with adequate valid support. If not, report VIEWPOINT_SUPPORT_WITH_SIMPLE_GEOMETRY_CONTROL, not a prediction-specific contribution.

Record native/expanded pool sizes, all proposals/aliases/rejections, same-cluster and cluster-switch flags, first action and successful commitment, coverage/Q/travel/revisit/failures, added renderer/A* CPU and online cost. Extra model inference is zero at S; the original four-model batch is shared. Different later observations naturally trigger native new models independently.

**Support/limits.** No alternate is NA for mechanism availability, not evidence every possible view is equivalent. All proposed alternatives invalid -> EMPTY_ALT; equal legal winner contracts can be structural zero only after full cache/suffix identity. Source representative may itself use an unknown route, so report the conservative known-route restriction on alternatives. This finite three-proposal study is not an optimization over all viewpoints.

**Resource cap.** Eight x four requests; F_native<=256 and expanded F<=4F<=1024, without dropping native outputs. No more than three A* proposal checks/region.

### Worked #6 pool construction — illustrative geometry only

Suppose a source region is the 15 observed-free cells (650,c), c=700..714, and current pose (645,695). This is a worked contract example, **not a real recorded state or simulation**.

- Native representative is (650,707).
- Smallest cell is (650,700); largest is (650,714); nearest-to-pose is again (650,700).
- The nearest proposal aliases the smallest; it does not cause a fourth proposal.
- (650,700) is only sqrt(50)*0.1m away, so source <1m rejection records NEAR_GOAL. Do not replace it with a favorable interior cell.
- (650,714) may enter only after its actual source weighted path is verified nonempty and observed-free; otherwise it stays invalid. No GT test participates.
- IN_CLUSTER_BEST compares native score at the representative with this surviving view, using the same G/mean/V and physical renderer. If the representative still wins, retain the zero decision; CHEAP can select the surviving alternative independently.
- EXPANDED adds the qualified view to the explicitly enlarged multi-cluster pool. All other source candidates remain.
No unseen-coverage gain or Q number follows from these coordinates alone.

## 8. Immediate gain versus downstream positioning/value

**Hypothesis.** The best realized first10m gain among a small predefined action set need not yield best longer remaining-budget exploration. This does not automatically imply native MapEx is greedy in true gain.

**Native context.** Native score is predicted visible-variance/distance, not the measured future10m gain. The investigator's100m distance prefix differs from native mission_time1000. Keep both budgets and the native-choice relation visible.

**Exact bounded action proposals, all from online-visible source state.**
- A0_NATIVE: source's actual source-valid/path-supported winner after native retries.
- A1_NEAREST: minimum Euclidean-distance legal source representative, native order tie.
- A2_MAX_MASS: maximum native unnormalized visible-unknown variance mass among legal source representatives, native order tie.
- A3_FARTHEST: maximum Euclidean-distance legal source representative, native order tie.

Use only the original source pool, same U/predictions/path validity/controller/sensor. A* source feasibility does not use GT collision lookahead. Preserve distinct first-decision dispatch contracts; duplicate goals/paths/contracts are alias requests. Fewer than two distinct actions -> INSUFFICIENT_ACTION_SUPPORT, no replacement viewpoint. #6 cannot enlarge this tested set.

Execute each action once to the full common R; obtain 10m values from that same trace, not a separate short simulation:
- G10 = C at latest complete scan at d_s+10 minus C(s), with the accepted endpoint hold.
- Q10 uses the same integral/endpoint rule with horizon10.
- Q_R is primary remaining-budget Q; all share source-step remainder and technical dispositions.

**Ties and censoring.**
- Primary immediate winner is by G10 integer gained-cell count. Tie epsilon =0.5/N+1e-12; one cell is a strict difference. Canonical tied display winner follows proposal order A0,A1,A2,A3 after alias removal.
- Long/Q10 tie epsilon =max(1e-6,0.5/N).
- Strict reversal requires a **unique G10 winner** and another tested action with strictly larger Q_R than it beyond epsilon. Report the winner's gap and full actions, not only positive examples.
- With tied G10 maxima, report TIED_PREFIX_DIVERGENCE, the best/worst long Q across the entire tied set and resolver-dependent regret. It is never a strict reversal.
- Q10 ordering is a declared secondary diagnostic. A Q10-only reversal cannot be substituted for failed primary G10 evidence.
- Native absorbing terminals before10m/100m remain valid source outcomes under endpoint hold; technical aborts censor. Missing actions carry bounds; they are not silently removed to invent a unique winner.

**Regret and attribution.**
- Tested native regret =max_tested Q_R - Q_R(A0), nonnegative only for complete comparable actions; report interval if any required action is censored.
- Prefix-winner regret and native regret are separate.
- Set NATIVE_IS_UNIQUE_G10_WINNER explicitly. A strict immediate/long reversal with A0 not the unique short winner demonstrates horizon disagreement in the tested set, **not native MapEx short-gain harm**.
- Native short-horizon task-loss attribution additionally requires A0=unique G10 winner, a feasible distinct tested alternative and common task-support/materiality gates.
- Ties can still show source-choice headroom, but cannot establish a strict horizon-order failure.

**Online/claim limit.** Proposal generation is online-visible; hindsight best Q_R selection is oracle-only. Best tested is not global best, an offline route optimum or an online lookahead policy. Promotion needs a separately reviewed GT-free budget/position selector and prospective costed validation. No native PIPE budget-weakness claim comes from this MapEx experiment.

**Resource cap.** Eight x four requests; exactly one realization per action; no extra horizon repetition, random action draw or GT-best candidate expansion.

