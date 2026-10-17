# Run 07 — Root-cause audit: premature room exits, near-goal stops, repeated viewpoints

Date 2026-10-17. Source-bound audit of COM1 native 2D New Room Run 07, using its full synchronized source+logs on DELL. **No changes to frozen Run 07 data.** This report distinguishes proven code-flow faults, correlated observations, and yet-to-be-tested counterfactual benefits. NOT independent IR1 QA.

## Authoritative frozen sources

- On DELL: `/home/dell/mx072_g2_r4_native_com1_run07_20261017/`.
- `explore_g2_r4_lama3_smallroi_run06.py` (Run07 worker executes this exact copied explorer; relevant lines 338–340, 357–362, 431–442).
- `g2_native_integration.py` (`needs_replan()`, `choose()`); `g2_r4_policy_sidecar.py` (`Service.choose()`, `distance_cells <= 5`, early invalidation, R4 action).
- `g2src/r4_episode.py`, `g2src/r4_adaptive.py` (`conservative_gate=True` and `NO_POSITIVE_ROBUST_ROOM_CHAIN`).
- Per-decision `room_r41/decisions/decision_*.json`, `room_r41/steps/tick_*_plan.json`, `room_r41/inference/decision_*_input.npz` and `*_semantic.npz`, `g2_r4/decisions/decision_*.json`, `global_obs/*.png`, `odom.npy`.
- Native base config on COM1: `/work/com1/mapex_single_audit/source/MapEx/configs/base.yaml`, `cur_pose_dist_threshold_m: 1` (grid 0.1 m/cell). The G2 sidecar records arrival only when within **5 cells = 0.5 m**. Both code paths were checked.

## Root cause A — native goal arrival condition conflicts with G2 goal completion

Unmodified MapEx resets a locked goal when **distance < 1.0 m** at the beginning of a native tick. The native G2 adapter also replans if its goal is <1.0 m away; this threshold is passed directly from the same config. The G2 policy sidecar recognizes a completed viewpoint ONLY if the robot is <=0.5 m away, otherwise clears pending commitments and records `INVALIDATED_EARLY_NO_FALSE_GOAL_RECEIPT`.

**Observed distance to most-recent goal at next decision:** D03 **0.73 m**, D14 **0.71 m**, D16 **0.70 m**, D21 **0.98 m**, D25 **0.89 m**. All five are among the user-identified valuable viewpoint choices. Their endpoint was **not physically visited in the discrete native route**.

Out of 23 G2 decision records: **14 `INVALIDATED_EARLY_NO_FALSE_GOAL_RECEIPT`**, **2 `COMPLETED_GOAL_OBS_* `** events, plus new-room/no-prior and selected-goal-invalid states. Because ledger updates and `CommitmentController.state.visits` updates occur on recorded completion, most successful-looking partial visits are never evaluated as completed views. This undermines both failure-angle suppression and G2's expected-vs-realized evidence. This is a **confirmed integration bug**, not a claim that no actual map cells were observed.

A separate symptom: five proposed LOCAL goals were **already within 1.0 m at selection**: **D08 (0.80m), D10 (0.80m), D15 (0.89m), D18 (0.89m), D19 (0.57m)**. A goal below the native acceptance radius may last only one 0.3m step before replanning.

## Root cause B — viewpoint churn / replan thrashing, not physical standstill

D08 (tick190) goal global (524,512), t+1 D09 goal (517,512), D10 goal (518,512); each leg only **0.3 m / one native step** and the D09–D10 goals are **0.1 m apart**. Actual observation UNKNOWN continued changing in this sequence, so do not label it a total map-update stall.

In room 3, the selected goals at **D14 (562,732), D15 (563,735), D17 (562,730), D18 (563,727), D19 (562,727)** occupy a small approx. 0.8 m diameter cluster. In between, G2 chooses **D16 (506,643)**, creating **16.78 m** outward travel (D16→D17) and **16.18 m** return (D17→D18), yet the original room UNKNOWN stayed **101 cells** from D17 through D20. Specific near-duplicate pairs include D14–D17 0.2 m, D17–D19 0.3 m, D18–D19 0.1 m. Failure to confirm arrivals/memory visits allows repeated candidates to pass the R3.1 repetition filter; whole-chain cost optimization alone did not suppress this.

## Root cause C — G2 DEFER unconditionally forces GLOBAL, even when RoomNode has a safe reachable LOCAL WFD candidate

The decisive source-level line is `candidate_g2_goal = g2_run.choose(...)` followed by `if candidate_g2_goal is None: ... room_run.recover_global(t)`. The `recover_global()` function changes scope from current `LOCAL` to `GLOBAL`, even if Room Node's native WFD on the same snapshot already produced **reachable** local frontier candidates. This conflates **"G2 cannot justify its structural goal chain"** with **"no safe/useful local exploration remains"**.

| Decision | G2 original-room UNKNOWN | G2 DEFER reason | RoomNode mode/reachable LOCAL WFD | Executed scope |
|---|---:|---|---|---|
| D05 (t95) | 320/3889 | NO_POSITIVE_ROBUST_ROOM_CHAIN | LOCAL, 1 | GLOBAL |
| D20 (t454) | 101/1176 | No usable structural chain in search budget | LOCAL, 1 | GLOBAL |
| D22 (t583) | 11/109 | No usable structural chain in search budget | LOCAL, 1 | GLOBAL |
| D24 (t712) | 73/779 | No usable structural chain in search budget | LOCAL, 1 | GLOBAL |
| D26 (t806) | 72/215 | NO_POSITIVE_ROBUST_ROOM_CHAIN | LOCAL, 2 | GLOBAL |
| D30 (t1159) | 64/64 | selected goal invalid (NO_LONGER_USEFUL) | LOCAL, 1 | GLOBAL |

At D22 the nearest chosen local WFD frontier was **about 1.8 m straight-line** from the robot; at D24 it was **about 1.3 m straight-line**. The eventual fallback global goals were respectively **about 20.5 m** and **13.7 m straight-line** away. These distances are straight-line lower bounds, *not* verified motion-plan path lengths; the RoomNode LOCAL candidate's A* reachability was independently screened by the original RoomRun code.

There are **two different G2 defer mechanisms**: (1) R4 `PathValue.choose` uses `objective>0` and conservative `conservative_objective>0` gates; (2) R3.1 fails to produce an accepted structural chain. Both are surrogate utility gates, neither certifies room exploration is finished.

G2 uses `exit_anchor=robot` rather than the actual global MapEx return/door anchor; this biases cost evaluation and is not yet an accepted handoff metric. Do not misrepresent this as the proven cause of every DEFER without further counterfactuals.

## Confirmed trajectory reentries into the *old same-snapshot spatial accepted-room mask*

For each DEFER we extracted the connected `semantic==ROOM and observed!=wall` component containing that decision's robot (the frozen SAME-decision mask). Checking **actual saved odom at every subsequent native tick**:

- D20 at tick454: after exiting, robot reentered this frozen accepted-room spatial mask at **tick847**, during the D26→D27 leg.
- D22 at tick583: reentered at **tick928**, during D27→D28.
- D24 at tick712: reentered at **tick1058**, during D28→D29.
- D05 at tick95: **no later reentry after first exit**, but LiDAR from outside changed some previously unknown cells within its same-snapshot semantic mask.
- D26 at tick806: reentered same accepted room mask at **tick1137**.

This is a geometric mask/intersection claim, NOT a semantic proof that the new epoch gives the room an unchanged RoomNode ID. These events strongly corroborate the user's visual interpretation of late returns.

## Empirical value of selected viewpoints (not just subjective quality)

The current frozen original-room unknown decreases after several chosen goal legs:
- D03 start 1103 -> D04 342: 761 original-room cells updated while traveling toward goal, despite ending **0.73 m** shy.
- D14 start 240 -> D15 193: 47 original-room cells; mission-region unknown changed **914 -> 392**, reflecting expanding/updated room scope and additional observations. Do not conflate original and mission counts.
- D16 start 193 -> D17 101: 92 cells.
- D21 start 109 -> D22 11: 98 cells.
- D25 start 215 -> D26 72: 143 cells.

These support the claim that many chosen viewing corridors yielded substantial realized new observations. **They do not prove that the final 0.7–1.0m would observe every remaining unknown cell**; that requires a causal follow-through simulation at a revised goal tolerance.

## Prioritized repairs (proposed; NOT implemented or assessed here)

1. **Per-policy arrival semantics**: G2 goal completion must be consistent with native motion controller. Prefer a goal tolerance tied to the 0.3m discrete step size / reachable A* endpoint (e.g. 0.2–0.3m in evaluation, not a hard-coded guarantee). Do NOT change MapEx/global frontier tolerance; do NOT merely relax G2 ledger 0.5m to 1.0m or it would falsely credit incomplete viewpoints. Log endpoint residual distance and reason each time a goal is unlocked.
2. **Local G2 fallback**: G2 `DEFER_ROOM` must not automatically invoke `recover_global()`. If RoomNode LOCAL has safe reachable WFD frontiers, select the best verified local MapEx/WFD candidate and stay LOCAL, with duplicate/failure guards and logs. Switch GLOBAL only on meaningful local exhaustion / safe exit rule. Never require 100% of unobservable room pixels or peek at GT.
3. **Dedupe & meaningful progress**: detect near-duplicate viewing poses (especially within <=0.8m), penalize a chain that backtracks to a recent goal without a verified marginal gain, and preserve observation-based region memories across replans. Accept revisiting when new evidence alters likely coverage; do not ban all repeat visits.
4. **Separate heavy planning from frequent checks**: avoid re-running both full MapEx frontier scoring and heavy G2 beam search redundantly each time; use cached LaMa and event-driven fast checks. Include actual physical turning/time and true exit anchor in cost.
5. **Preflight isolated tests**: replay D08–D10 and D14–D19 event sequences with exact saved snapshots, assert no 1-tick goal thrashing or missed receipts; replay D05/D20/D22/D24/D26 with R4 DEFER and RoomNode LOCAL frontier still available; only then run a new full causal native2D result against frozen Run04/05/07. Keep originals immutable.

**Causality limitations:** The thresholds and DEFER-to-GLOBAL handler are definite code-flow explanations. The effect of hypothetical additional travel or WFD fallback on time-to-coverage, path safety, or completion percentage requires a new causal run; cannot be inferred from static images. A single native run cannot establish statistical generalization.
