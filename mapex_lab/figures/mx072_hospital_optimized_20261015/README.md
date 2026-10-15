# MX072 Hospital P1 — real Native2D 20-vs-21 decisions, optimized LaMa

Visual comparison: Run 09 = plain MapEx (21 decisions); Run 08 = MapEx + Room Node R4.1 (20 decisions).

- 20 aligned ordinal decision pairs showing original recorded observed map, robot, frontier candidates, first logged A*, retrospective actual trajectory, plus room R4.1 semantic / predictions.
- Extra D21 is a **plain-only decision**; Run 08 already finished at D20. No fabricated room D21.
- HTML embeds all three panels and detailed per-decision stats. PDF is 22 pages in landscape, iPad friendly.
- Hospital input: conservative 0.1m P1 downsample of existing HOSPITAL_FLAT structural map (source resolution 0.05m). **Native2D, not live ROS2/Gazebo mapping**.
- Ground truth was only used in offline evaluation. Native planner SOURCE_TERMINAL is not GT-complete; ~23% occupied GT recall in both runs.
- Distance: 388.38m plain vs 312.60m room (room 19.51% shorter); wall time: 138.50s vs 132.53s (room 4.31% faster). One paired experiment, not inferential evidence.
- Timing boundaries for LaMa median differ (plain includes preprocessing), so these figures are not like-for-like.
- Decision numbers are not aligned in time or space once trajectories diverge. Teal path = retrospective actual motion, purple A* = native plan from that decision.

Source backed up on DELL at /home/dell/mx072_native2d_pair_20261015/HOSPITAL_OPTIMIZED_PAIR_20261015/COM1_VERIFIED, EVALUATION/; archive transfer SHA-verified. This commit is visualization only, not a new experiment or scientific release.
