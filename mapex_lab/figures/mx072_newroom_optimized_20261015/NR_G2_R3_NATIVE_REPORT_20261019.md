# G2-R3 — Native2D evaluation verdict, 19 October 2026

**Trial:** NR-G2SEQ-02  
**Reference:** NR-RN-04, original MapEx + G1 V4-R4.1  
**Status:** SOURCE_TERMINAL, returncode 0; D01/D02 numeric and G1 scope parity PASS.

## Design frozen before execution

G2-R3 Global Conditional Frontier Sequence modifies **GLOBAL choice only**. It starts from MapEx's original far frontier F, considers up to six closer official WFD candidates N and two distinct currently valid continuation endpoints E. Using unchanged native clearance-weighted A*, same-epoch observed map and LaMa ensemble, it compares:

- Near: \(C_N=d(S,N)+d(N,F)+d(F,E)\)
- Far proxy: \(C_F=d(S,F)+d(F,E)+r\max(0,d(F,N)+d(N,E)-d(F,E))\)

Here \(r\) is the share of forecast variance visibility unique to the near-first route compared with the original route. **Not a calibrated persistence probability.** Added legs must traverse observed FREE. R3 changes the initial goal to N **only if BOTH current continuation endpoints E exhibit strictly positive conditional margin**; otherwise it keeps original MapEx F. Only one goal is executed and replanned after LiDAR. Original G1 room detection, LOCAL candidate/scorer, LaMa model and A* are unchanged. No GT/future pose/previous run waypoint used in online decisions. Complete frozen source and design in this branch.

## Shadow findings

Read-only shadow on archived GLOBAL decisions from three native runs (NR-RN-04, NR-G2PVU-02, NR-G2PVU-03): no tested post-initial decision satisfied the whole switch guard. One synthetic adversarial unit test constructed an off-route observed-UNKNOWN pocket and two consistent continuation endpoints, proving that the code can activate a first-goal change; unit PASS. 3-model LaMa smoke PASS. The rule was **not changed after seeing shadow**.

## Full native result

| | Run04 MapEx+G1 | G2-R3 |
|---|---:|---:|
| Terminal outcome | SOURCE_TERMINAL | SOURCE_TERMINAL |
| Odometry | 199.107583 m | **199.107583 m** |
| Observed P1 | 51,304 | **51,304** |
| Native ticks | 709 | **709** |
| Total decision | 25 | **25** |
| LOCAL | 19 | **19** |
| GLOBAL | 6 | **6** |
| GLOBAL scorer R3 evaluations | – | 6 |
| Goal changes by R3 | – | **0** |
| Decision reasons | – | 5 NO_CERTIFIED_GAIN; 1 NO_CLOSER_WFD |

Strict offline match audit over **all 709 tick** pairs reports:
- Complete odometry array identical;
- 709/709 observed maps and scan poses bit-identical;
- All 25 decision timestamps, modes, room IDs, poses and reasons matched;
- All 25 native selected goals and A* routes matched;
- D01/D02 official guard parity PASS.

Hence no route improvement: **Δ distance = 0.000000 m, Δ observed = 0 cells**. R3 **NO_ACTION / NEGATIVE UTILITY** for this native New Room trial. The fact that it produces the same result does not imply MapEx+G1 is globally optimal, and does not prove the goal ranking would benefit a different environment.

## Main lesson

R3 is too conservative **on this tested run**: requirement for two independent continuation endpoints to agree, together with a low estimated residual-visibility fraction and observed-FREE detour constraint, led to zero interventions. This avoids the large late GLOBAL regret induced by earlier PVU variants, but cannot exploit the D21-style opportunity. Changing these conditions requires **a newly named** R3.1/R4 design and a fresh ablation precommit, not retroactively redefining R3.

A valid next experiment should establish whether predicting frontier persistence is reliable on New Room **and Hospital**, and whether there is a robust cost-of-return condition that does not depend on arbitrary scenario-specific thresholds. No further native run automatically warranted without a shadow capable of intervening and parity on alternative map layouts.

## Immutable artifacts (DELL)

- \`/home/dell/DELL_NATIVE2D/runs/NR-G2SEQ-02/\` — frozen staged source, SHA-256 manifest, worker stdout, scans, decisions, receipt and parity audit;
- \`/home/dell/DELL_NATIVE2D/analysis/g2_autonomous_research_20261018/NR_G2_R3_GLOBAL_CONDITIONAL_SHADOW.json\` — 3-run shadow;
- \`/home/dell/DELL_NATIVE2D/analysis/g2_autonomous_research_20261018/NR_G2_R3_NATIVE_VS_RUN04_AUDIT.json\` — exhaustive 709-tick comparison;
- \`/home/dell/DELL_NATIVE2D/analysis/g2_autonomous_research_20261018/audit_r3_vs_run04.py\` — independent reproduction;
- \`/home/dell/DELL_NATIVE2D/analysis/g2_autonomous_research_20261018/RESEARCH_JOURNAL.md\` — research log.

This is an honest scientific negative result: implementation validated, hypothesis not confirmed.
