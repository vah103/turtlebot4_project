# MX072 R5 — COM1 native decision audit

Engineering candidate for the accepted New Room R5 method. This is a private,
headless 2D MapEx adapter, immutable recorder, native replay, and offline fresh
shadow scorer. It does not modify the ROS baseline, source MapEx checkout,
LaMa checkout, weights, live robot, or other research jobs.

**Current result: `IMPLEMENTATION_BLOCKED`, resource gate P12 FAIL.** No READY
token, authorization receipt, or scientific trajectory was produced. Maker
fixture checks are not an independent runtime ACCEPT.

## Provenance and frozen contracts

- Design: `vah103/chat-gpt` PR #53 at
  `67ca9f74e6ff99c93d5548d4ecf7e6c82eb5459b`; design SHA-256
  `1cc5e58bb3e12433c5820c9448d9125c772ccf428b31d067002d5959d80dda5a`.
- Method-only IR1 ACCEPT: `0d4b43bde06ad5582d43f6827e53bad2e73cfd8d`.
- Technical base: `88fb673b7a19d2973cfb1c78b4d0a083fdf66b50`.
- MapEx: `53636bd1c79153acc3c74a532837d78c926bae5e`;
  LaMa: `b61dcb33e063fe9586b50e2dc7b70f97d5046c1d`.
- Direct SDF P1 slice at z=0.20 m; no physical inflation. Raw grid 203x263
  at 0.10 m, source mapper 1203x1263, LaMa tensor 1216x1264. P2 GT/ROI use
  the accepted 0.05 m generator, exact 2x2 mapping, and weighted ROI projection.
- Source physical 2500 rays / predicted 250 rays; probabilistic raycast bug,
  unshifted source candidate origins, effective unknown-as-occupied false,
  one 3x3 planning inflation, four-neighbor A*, goal locking/reselection,
  controller path index 3, and center-cell collision remain source behavior.
- G then G1/G2/G3, CPU float32, one model resident, original transform,
  NumPy mean and torch 1.10 sample variance. No crop, quantization, or model skip.
- Primary oracle: `V_GT_SENSORGRID_PREDONLY250`, write only common
  unknown/support; preserve known, padding and exterior. P2 ANYOCC remains
  `STRUCTURAL_P2_SENSITIVITY`. All candidate renderers are recomputed.
- 0.5 m snapshots follow native pose updates without controller feedback.
  `NATIVE_ACTUAL`, `NATIVE_REPLAY`, `SHADOW_FRESH` and
  `LOCKED_GOAL_REFERENCE` are separate. Corner diagnostics are not computed.
- k<=1000 is `NATIVE_BUDGET_PREFIX`; k>=1001 is `EXTENDED_BUDGET_TAIL`.
  The cap is 10000; terminal completion does not imply full exploration.

## Bounded verification completed on COM1

Final attempt: `/work/com1/mapex_single_audit/preflight_r5_20261010_attempt03`.
Compact, hash-bound evidence is in `evidence/20261010/`; original arrays,
resource trace, ledgers and plots remain on COM1 at the indexed paths.

| Check | Maker result and scope |
|---|---|
| P1-P3 | source/model/env/world hashes, pinned grid/frame and ABI import fixtures PASS |
| P4 | all four real models at full size; repeated CPU outputs bit-identical; validated reuse of earlier probe PASS |
| P5-P6 | source sensor/planning/controller/scorer fixtures; all candidates, empty pool and all-zero ties PASS |
| P7 | actual pinned explore.py versus adapter, recorder off/on: 4+4+4 fixture steps, same pose/map/goal/pool/cost/terminal PASS |
| P8-P10 | immutable distance snapshots, simultaneous boundary event, scan/state hashes, native replay, matched full-renderer oracles PASS |
| P11 | synthetic crossing-action split at 999/1000/1001/1002; not a real 1000-step trajectory PASS |
| P13-P14 | 2 real-model native control steps, final scan/map/pose, one fresh offline shadow unit and validated completed-unit resume PASS |
| P12 | FAIL: storage projection exceeds quota; transient combined-memory peak also exceeds target |

The final technical attempt contains **14 fixture/smoke control steps**, including
12 source-comparison fixture steps and only 2 real-model steps. Earlier attempt01
stopped on an import-generated source cache; attempt02 completed the same bounded
smoke but failed report encoding. Neither is scientific trajectory evidence.

Model-only kernel peak RSS was 3,357,257,728 bytes (3.127 GiB). Final combined
preflight kernel peak was 4,927,135,744 bytes (4.589 GiB), above the 4.5 GiB
target. Live 0.5 s samples never exceeded 4.5 GiB and no sustained five-second
abort fired; this does not erase the transient peak or certify peak-RAM PASS.
Minimum MemAvailable was 1,621,528,576 bytes (1.510 GiB).

Five mandatory compressed maps (G1/G2/G3/mean/variance) measured 27,972,822
bytes per initial input. Multiplying by at most 10001 distinct observed inputs
gives **279,756,192,822 bytes (260.543 GiB)**, before rays, snapshots or ledgers.
The complete model bundle projection is **447,432,878,814 bytes (416.704 GiB)**;
the contract permits only 64 GiB. One complete measured shadow unit is
47,268,944 bytes, including all three arms. The projection credits potential
native/shadow input overlap; current object dedup is per store, so cross-store
sharing is an optimistic projection credit, not an implemented global saving.

These are cap-times-measured-compression projections, not actual scientific
usage or proven worst-case entropy bounds. Candidate ceilings and total offline
job time are not certified by one state. The model-only cap envelopes are
437730 s native / 262682 s shadow; actual native prediction frequency may be
lower, but the 12 h acquisition budget is not certified by this smoke.

## Technical commands

COM1 checkout: `/work/com1/mapex_single_audit/implementation_r5_20261010`.
Environment: `/work/conda-envs/mapex-single-audit-com1` (Python 3.6.13,
torch 1.10.2, NumPy 1.19.5, pyastar2d 1.0.2, range_libc 0.1).
Geometry generation uses system Python 3.12 and the pinned repository generator;
model/native work uses the private Python 3.6 environment.

```bash
cd /work/com1/mapex_single_audit/implementation_r5_20261010
python3 -m mapex_lab.pilots.single_run_decision_audit.geometry \
  --repo "$PWD" --output /work/com1/mapex_single_audit/geometry_r5_NEW_ID
PYTHONDONTWRITEBYTECODE=1 OMP_NUM_THREADS=4 MKL_NUM_THREADS=4 MPLBACKEND=Agg \
  /work/conda-envs/mapex-single-audit-com1/bin/python -u \
  -m mapex_lab.pilots.single_run_decision_audit.preflight \
  --repo "$PWD" --geometry /work/com1/mapex_single_audit/geometry_r5_20261010 \
  --output /work/com1/mapex_single_audit/preflight_r5_NEW_ID --stage full \
  --reuse-model-probe /work/com1/mapex_single_audit/model_probe_r5_20261010
```

Directories must be new; failed output is preserved. Full preflight exits 2
when blocked, and an enforced resource abort exits 75. Reused probe arrays,
prediction implementation, full model/source/env/config/geometry bindings are
checked before reuse. Code changes invalidate the exact full-preflight binding.

`run_one.py` requires an explicit one-trajectory USER grant and an independent
technical ACCEPT receipt against the same stable binding and sealed timeout;
the present report fails that gate. There is no scientific acquisition resume.
`shadow.py --resume` only validates and skips committed offline units; an
interrupted uncommitted directory remains an explicit blocker requiring recovery.
The global output guard counts native, shadow and metadata artifacts; its total
bytes check runs on the five-second heartbeat, while object writes also have a
per-store quota check. This is bounded detection, not a filesystem reservation.

## Review and remaining work

Review the exact implementation/evidence before any merge or readiness claim.
P7 establishes four-step fixture equivalence, not universal equivalence across
all native failure/reselection branches. P11 uses a synthetic split fixture;
abort/collision edge cases, exact recorder overhead (direct object serialization
is not fully included in `recorder_io_s`), complete time budgets, and a lossless
global storage bound remain review items. The current renderer is an initial /
terminal diagnostic and availability matrix; scientific timelines require actual
trajectory/audit data. Optional full-GT/corner branches are not computed.

A bounded storage/memory engineering revision may use the already permitted
lossless deltas or global dedup. Changing quotas, cap, sampling, required maps,
or candidates requires a reviewed contract revision. Do not raise limits or
omit artifacts to turn this negative preflight into PASS.
