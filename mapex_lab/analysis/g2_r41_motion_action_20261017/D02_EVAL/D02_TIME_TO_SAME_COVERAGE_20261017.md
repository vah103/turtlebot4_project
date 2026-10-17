# D02 — Time-to-same-coverage comparison (2026-10-17)

Method: use exact D02 initial P1 room mask from archived G2-R3.1 evidence, compare time until original-room UNKNOWN count <=270 (which is 93.0573% of 3889 *initially UNKNOWN* cells resolved). Do **not** confuse this with 93% of all 6564 original room cells: 6294/6564 = 95.887% of whole room area has been observed when 270 remain.

Frame alignment verified **zero mismatches**: local D02 203x263 occupancy equals source global observed PNG at native tick 12 cropped [500:703,500:763] for both native Run04 and Run05. The D02 robot (147,111) corresponds to global (647,611). Exact source observed hash: de96490d2c94ca0b10ff7f359b2b8fe3b4bc9a8d5656194b9ceec8c13c539b23.

## Distance to identical room-coverage target

| Strategy | Data/provenance | Native tick/time-to-coverage | Distance starting D02 | Remaining D02 room unknown |
| --- | --- | --- | --- | --- |
| G2-R3.1, proxy for old R4 shadow decisions | causal D02 R3.1 summary, 4 legs and 39 scans | N/A (different offline driver) | 18.8 m | 270 |
| MapEx + Room Node V4-R4.1 Run04 | native full optimized Run04, initial tick 12, first reaching target at tick 102 | 90 native ticks after D02 | 24.861 m | 269 |
| Plain MapEx Run05 | native full optimized Run05, initial tick 12, first reaching target at tick 767 | 755 native ticks after D02 | 213.513 m | 268 |

Native Run04 was a **different room-scoped MapEx/WFD planner**, not the proposed G2-R4 implementation. Native Run05 was a global MapEx explorer; the long elapsed route includes wandering into other areas. This directly measures time-to-coverage *globally*, NOT MapEx constrained to remain inside the same room.

Distances computed as sum of Euclidean consecutive global odom displacement * 0.1 m P1 between native tick12 and threshold tick. G2 R3.1 distance from summary.json.

## Time estimates at a common virtual driving speed

Assume constant movement speed 0.3 m/s (not a measured physical speed), with no stops, acceleration or LiDAR dwell:

- G2-R3.1: 18.8 / 0.3 = 62.667 s movement; component compute timing measured on DELL = 70.55 LaMa +1.94 Room Node +11.13 structural +38.98 commitment = 122.60 s. Naive sum =185.27 s, **not a validated end-to-end runtime**. Earlier old R4 is a shadow policy, not a distinct executed trajectory and has no separately measured R4 time. Initial frozen plan generation may not be included; G2 simulated sensor overhead not included.
- Run04: 24.861 / 0.3 = 82.87 s movement; logged prediction+room scoring *only after native D02 tick12 and through tick102*: 6.798 s; naive subtotal >=89.668 s. Native RUN04 COM1 architecture optimizes 3-model LaMa.
- Run05: 213.513 / 0.3 = 711.71 s movement; logged prediction *only after tick12 through tick767*: 32.194 s; naive subtotal >=743.904 s.

**DO NOT claim the naive subtotals are fully measured comparable exploration times.** Runs04/05 are optimized COM1 native 2D, G2 R3.1 is DELL offline; the timing components, hidden sensor simulation, robot kinematics, and whether recomputation overlaps travel are not normalized. Run04/05 logs only known model/score timings, not full simulation/controller costs. R3.1 costs may exclude initial R3 goal planning and all missing physical overhead. The G2 path stops at a room goal, not a verified exit/rejoin handoff. A clean experiment must use one execution engine, identical velocity/sensor scan cadence, equivalent rejoin/coverage thresholds, and measure wallclock movement, scan, model, planning separately.

At matched target, geometric ratios only: plain MapEx requires 213.513/18.8 = 11.357× as much travel as G2 R3.1, and RoomNode MapEx requires 24.861/18.8 = 1.322×. These are descriptive single-run differences; not predictive algorithm improvements. The plain run delay is a global schedule effect, not proof its local MapEx goal selector is 11× worse.

## Reproduce

Source data are archived in /home/dell/mx072_native2d_pair_20261015/OPTIMIZED_FULL_PAIR_20261015/COM1_VERIFIED/native_plain_lama3small_run05 and native_opt_r41_lama3small_run04. Initial room mask from /home/dell/mx072_g2_r31_structure_commitment_20261016/D02_R31_FULL_CAUSAL_20261016/D02_G2_R31_CAUSAL_EVIDENCE.zip. Compute with Python script d02_compare_rooms.py (export from DELL .d02_compare_rooms.py). Both arm traces and milestones are retained in d02_mapex_room_coverage_comparison.json. Scan PNG grayscale (0 observed free; 128 unknown; 255 occupied), crop [500:703,500:763]; compute missing=(cropped_png==128)&initial_room_mask; first tick meeting <=270. Distances from odom.npy over consecutive points with 0.1 m native resolution.

No GT was used to select any planner action in these runs; the initial room mask here is R3 Memory from observed/Room Node. It is used **retrospectively** as a matched evaluation ROI, not input to MapEx native.
