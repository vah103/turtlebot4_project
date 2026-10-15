# MX072 New Room — 25 decisions each, optimized LaMa

Visual comparison: native Run 05 (plain MapEx) vs Run 04 (MapEx + Room Node R4.1).

- HTML: 25 decisions, three panels for each decision (plain observed frontier decision, room-aware decision, room semantic/predicted structure), with native logged A* and post-decision odometry, plus final comparison.
- PDF: iPad-friendly 26-page landscape visualization.
- Important: decisions are aligned by ordinal number, **not** by time or spatial location. The plots use snapshots from the respective run. The teal line is retrospective recorded motion, not planner foresight.
- Native runs terminate when no selectable frontier remains; this does not certify ground-truth completion.
- Results from one pair of runs only, not inferential evidence; GT stats are evaluated offline.
- Production-side experiment artifacts and SHA-verified copies remain on DELL under /home/dell/mx072_native2d_pair_20261015/OPTIMIZED_FULL_PAIR_20261015/COM1_VERIFIED.

Run 05: 25 decisions, 370.00 m, 138.51 s. Run 04: 25 decisions, 199.41 m, 102.38 s; GLOBAL=6 / LOCAL=19.
