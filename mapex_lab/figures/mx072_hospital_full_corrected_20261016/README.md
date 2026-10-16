# MX072 Hospital FULL corrected — RUN10 Room R4.1 vs RUN11 plain MapEx

- Source: COM1-verified native2D Hospital FULL corrected pair (2026-10-16), independently re-calculated on DELL but without independent science QA.
- RUN10 Room Node: 41 decisions, 2224 ticks, 642.2194 m, 415.83 s, free-space coverage 99.996659%.
- RUN11 plain MapEx: 39 decisions, 2883 ticks, 832.5093 m, 397.69 s, free-space coverage 99.994989%.
- Difference: RUN10 saves 190.2899 m (22.8574%), but takes 18.14 s longer (4.5613%).
- 41 ordinal pages + summary; each page includes native images, real first A* and retrospective recorded trajectory. RUN11 has no corresponding D40/41, so those panels remain explicitly blank.
- HTML is self-contained and usable on iPad. Attached render script and validation report show reproduction and caveats.
- Full input source remains at /home/dell/mx072_native2d_pair_20261016/HOSPITAL_FULL_CORRECTED_PAIR/COM1_VERIFIED; immutable COM1 to DELL transfer verified 17,838 files / 221,833,419 bytes.
- This is a 2D static native comparison, not ROS2/Gazebo, and SOURCE_TERMINAL is not an assertion of full GT completion. Inference timing fields between arms have non-equivalent timing contracts.

## 2026-10-16 — corrected visualization palette

The initial gallery renderer erroneously assigned ROOM/CORRIDOR colors to the wrong integer codes and also mislabeled WALL/OBSTACLE. The underlying COM1 RUN10 native computation and original semantic NPZ files were **not** modified. The source of truth in v4r41.py is ROOM=1, CORRIDOR=2, OBSTACLE=3, WALL=4. The native controller checks sem == 1 for room-scope navigation, so this issue was confined to the HTML renderer. The corrected renderer uses pink ROOM=1, yellow CORRIDOR=2, dark gray OBSTACLE=3, red WALL=4, and gray OTHER=0. Tentative UNKNOWN rooms now require baseline CORRIDOR=2 -> semantic ROOM=1. Renderer asserts all ten LOCAL robot poses fall in sem=1. These checks establish wiring/color consistency, **not** blind semantic room-label accuracy or science validation.
