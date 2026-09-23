# R003 paper500 implementation evidence

Status: COMPLETE_PENDING_REVIEW
Date: 2026-09-23
Task: H031 / R003-PAPER500-IMPL
Branch: `r003-paper500-h031`
Canonical contract: `docs/R003_PAPER500_PROTOCOL_V1.md`

## Scope

- Explicit 500-step / 150.0 m odometry budget with common crossings 10..500.
- Official provenance IDs use `new_room_mapex_paper500_v1` and
  `odom_progress_0p30m_500step_v1`.
- NF and MapEx emit `paper500_snapshots.csv`, paper500 metadata keys, map names,
  cutoff reason `budget_500_reached`, and symmetric terminal behavior.
- Public runner mode is `./run --paper500` with optional non-interactive method,
  environment, record and run-ID arguments.
- Alltrain snapshot input, evaluator output and endpoint are paper500-labelled;
  natural completion holds to 500 and algorithmic failure does not hold.
- The historical `r003_paper1000.py` implementation filename is retained behind
  the public `r003_paper500.py` API to avoid a risky code move. It is not used as
  official output provenance.
- `run_r003_paper500_batch.py` owns the exact 20-run matrix, atomic per-run
  state, resume, post-processing, one bounded same-ID technical retry and stop
  behavior for repeated/ambiguous invalidity.
- `r003_deadlock_watchdog.py` applies the same objective action-plus-odometry
  detector to NF and MapEx.

## Frozen watchdog

- action status required active;
- 120 s startup grace;
- 0.05 m cumulative odometry progress resets the timer;
- 180 s continuous no-progress window declares technical deadlock;
- inactive action state resets the window;
- missing odometry cannot trigger;
- poor metrics and ordinary algorithmic/navigation outcomes never trigger a
  replacement run.

## Profile parity

The paper500 profile is generated under
`ground_truth/new_room/generated/r003_paper500/`. Counts and TU-goal identity
match the accepted prior profile:

- reduced occupied: 7,179;
- reduced free: 46,497;
- valid/evaluation cells: 52,924;
- TU goals: 100, seed 3001;
- TU goal array SHA-256:
  `b6e805e5f78d0fd95fd435b7b06edb693d638127fe6b1727d1a5898dda6f505e`.
- profile NPZ SHA-256:
  `3233d9bae62a7dfb52d3f9248c9932218d06bb5e2a2bea3516c57c5074481124`;
- TU-goal file SHA-256:
  `d6b587196e6c964e679be0e660b08b39c97398aca51b46f0e7e35a19fbb81471`;
- GT/valid overlay SHA-256:
  `73c0c3826c038bf052c742f3249e81e9861433fa69aeeaf27479cce0cb317132`.

Only horizon/provenance changed; GT, valid-space, E10/V10, threshold, TU and
prediction semantics did not.

## Validation

```bash
python3 -m py_compile mapex_lab/scripts/r003_paper1000.py \
  mapex_lab/scripts/r003_paper500.py \
  mapex_lab/scripts/evaluate_r003_paper500.py \
  mapex_lab/scripts/generate_r003_paper500_profile.py \
  mapex_lab/scripts/predict_alltrain_offline.py \
  mapex_lab/scripts/run_r003_paper500_batch.py \
  mapex_lab/scripts/r003_deadlock_watchdog.py

PYTHONPATH=mapex_lab/scripts python3 -m unittest \
  mapex_lab.scripts.test_r003_paper500 \
  mapex_lab.tests.test_r003_paper500_initialization \
  mapex_lab.tests.test_r003_paper500_batch

bash -n run .run_core
./run --help
python3 mapex_lab/scripts/run_r003_paper500_batch.py --dry-run
```

Official runtime remains blocked until H030 Stage-B independently accepts the
exact implementation SHA. No paper500 simulation was started by H031.
