# Geometry-impact P1/P2 development results — 2026-10-08

P2 finished with a valid conservative task interval but failed the requested precision target: loss [0, 130.1] m, width 130.1 m versus the 1 m measurement target. No positive task-saving witness was found. This does not prove either prediction adequacy or harmful prediction. The practical harm threshold is deliberately unspecified.

P1: four preselected initial states, one per verified building. Action-set bounds were exact for three states: two singleton candidate sets, plus kth_50015848 where both native and GT-corrected choices are attained. kth_50010535_PLAN1 remained unresolved: lower {6}, upper all 12 candidates, 110410 relevant correction cells, 2394 nodes and 2396 open subcubes. Fresh uncached anchor inference is byte-identical. Real native parity: 144 checks. See results/P1_compact.json.

P2: kth_50052748, sealed start (181,282), R4 A1 physical controller, 0.1 m grid, 150 m mission budget. The terminal quality reference is coverage 55852/60019, occupied IoU 7870/8440 and 95/100 fixed GT-footprint-safe route successes. Native baseline first hits the componentwise reference at 146.8 m. Baseline has no collisions and matches the original R4 C_bar, Q, final coverage and distance within 1e-9.

Search began 43 nodes (42 fully logged, one interrupted and retained), after 16 baseline states. At the 900 s control checkpoint it retained 364 open branches: 4 witnessed prefixes and 360 unproved prefixes. Actual recorded phase time is 904.30 s, including an in-flight operation and queue serialization. The 120-node total baseline/search cap was not exhausted. 52 new model calls, 8 cache hits, 379.66 s inference. Of 42 completed branching nodes, 41 geometry action sets were unresolved; 17 nodes had multiple actions independently checked against the frozen native scorer. There were no proved earlier task target hits.

The upper 130.1 m is determined by a relaxed, unproved prefix at 16.7 m: 146.8 - 16.7. It is not an attained saving or an observed loss. Keeping that branch is conservative; no useful remaining-distance lower bound was available. Zero lower means no better witness found, not proof that every correction is useless.

The cache-only post-run audit passed in 21.04 s. It recomputed all 16 baseline metric/hash states, replayed every native baseline decision and physical path, verified frozen source/data parity, checked all 364 queue states and their distance envelopes, and recomputed the final frontier bound. It explicitly accounts for the one unlogged in-flight node. No new model inference or additional search was performed by the audit.

This run is 2D development only. No population frequency, held-out-building generalization, online repair method, training, robot/ROS/Gazebo run, or independent Company QA verdict is claimed. Secondary-area search was not performed (trivial bound only [0,41.67] m2). The four verified buildings have unverified train overlap. The adapter is A1 4-neighbour, whereas the paper uses an 8-neighbour controller. No 1370-mission confirmation batch was opened.

Provenance:

- R4 A1 reference: 348a2cda77b0a7542b973fdd07ab040514f8493b.
- P1 source: 17a1704fe9974e6bd7e81dad601167dc9f61108c.
- P2 source: da0cb66b9309646c4adc4cbc993b2ace541bbc21.
- Audit source SHA256: 28a85f3906e287413b273928970cb6d22358ef9a9459d27252feb212f4d20715.
- Runtime: Python3.11.16 / NumPy2.4.6 / SciPy1.17.1 / Shapely2.1.2.
- Before-run checks: 1587 reader/action checks (512 correction maps); 567 task-bound/quality checks, including predicted routes crossing GT walls.
- Evidence: results/P1_compact.json; results/P2_baseline_compact.json; results/P2_frontier_compact.json; results/run_20261008_dell/p2/{seal,predictor,tu_goals,summary,audit,audit_runtime}.json; process metadata.
- Raw local queue/search/baseline digests: results/P2_frontier_compact.json. Heavy raw states, maps, cache, correction subcubes and verbose logs are intentionally not committed.

Run command: `/home/dell/miniforge3/envs/mx040-py311/bin/python -u mapex_lab/pilots/geometry_impact/run_preflight.py --config mapex_lab/pilots/geometry_impact/protocol.json --phase p2`.

Audit command: same interpreter, `mapex_lab/pilots/geometry_impact/audit_completed.py --config mapex_lab/pilots/geometry_impact/protocol.json`. Audit requires the retained local inference cache and queue/state files.
