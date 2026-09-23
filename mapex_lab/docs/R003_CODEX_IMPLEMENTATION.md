# R003-CODEX implementation evidence for WORK review

Status: COMPLETE_PENDING_REVIEW (implementation/mask gate only)
Date: 2026-09-23
Branch: `r003-paper1000-implementation`
Canonical contract: `docs/R003_PAPER1000_PROTOCOL_V1.md`

## Outcome

Implemented the frozen New Room `new_room_mapex_paper1000_v1` construction
without changing NF or MapEx policy scoring. This delivery includes:

- versioned 0.10 m structural/valid/evaluation masks and deterministic 100-goal TU set;
- immutable raw-map common samples at initial, every crossed k=10..1000, cutoff/natural final, and post-cancellation;
- accumulated odometry budget with residual carried across goals/recovery;
- responsive cutoff subscriptions in a reentrant callback group under a four-thread executor;
- goal-dispatch suppression after cutoff, normal action cancellation, cutoff-map latch, separate post-cancellation sample and 15 s cancellation watchdog;
- snapshot-backed alltrain inference for both methods, with hash deduplication and checkpoint/config/preprocessing provenance;
- fixed R003 Coverage, occupied IoU and 4-neighbour A* TU evaluator with explicit error accounting;
- focused unit/integration tests and generated visual audit artifacts.

Smoke and bulk runs are intentionally not included. WORK mask/implementation
review is the next frozen gate.

## Generated profile evidence

Source SDF SHA-256:
`678c60172328183b42d2b26c15a8ed5a6faa8467a5b6e54f2564e2f7d974a735`

Source 0.05 m counts:

- occupied: 23,990
- free in footprint: 187,706
- valid: 211,696
- valid-and-occupied: 23,990

Reduced 0.10 m counts:

- occupied including mandated padded row: 7,179
- free in evaluation domain: 46,497
- valid: 52,924
- valid-and-occupied: 6,427
- evaluation domain: 52,924

Frozen start cell: `(row=601, col=256)`. It is inside E10 and GT-free.
The padded odd source row does not intersect source valid/evaluation support.

Artifact hashes:

- profile NPZ: `2925b93e68c435d292919303f54ca451fdebd4dc955d23e12874f9c2040298a9`
- TU goal file: `d6b587196e6c964e679be0e660b08b39c97398aca51b46f0e7e35a19fbb81471`
- TU goal array identity: `b6e805e5f78d0fd95fd435b7b06edb693d638127fe6b1727d1a5898dda6f505e`
- GT/valid overlay: `463ab9dda2659126c48b5722435c3b50bd7102ad13780745cfee3345f827d89e`
- observed alignment overlay: `62782c4e1e616224be2ed2d5212f6947975e574cf4755d2166cfe96eac8d2b6d`

Visual inspection: footprint, room topology and start pose are aligned. The
saved final `mpx_001` SLAM boundary follows the same structure, with expected
thin red/green one-cell boundary differences from SLAM/raster thickness. No
large translation, rotation or wrong-building footprint is visible. WORK must
accept or reject this construction before smoke.

## Validation

Commands:

```bash
python3 -m py_compile \
  mapex_lab/scripts/r003_paper1000.py \
  mapex_lab/scripts/generate_r003_paper1000_profile.py \
  mapex_lab/scripts/evaluate_r003_paper1000.py \
  mapex_lab/scripts/predict_alltrain_offline.py \
  mapex_lab/scripts/_nf_run_core.py \
  mapex_lab/scripts/nf_run.py \
  mapex_lab/scripts/mapex_run.py

PYTHONPATH=mapex_lab/scripts python3 -m unittest -v \
  mapex_lab/scripts/test_r003_paper1000.py

PYTHONPATH=mapex_lab/scripts python3 -m unittest -v \
  mapex_lab/tests/test_alltrain_recording.py

PYTHONPATH=mapex_lab/scripts:mapex_lab/analysis/d1 python3 -m unittest -v \
  mapex_lab/analysis/d1/test_r002_gt_semantics.py
```

Observed result at delivery: R003 13/13 PASS; existing alltrain 4/4 PASS;
existing R002 12/12 PASS. `git diff --check` passes.

The R003 suite covers odd-row reductions, occupied/free/unknown precedence,
strict `p>0.5`, axis-aligned projection, budget residual/recovery, odom
time/frame/gap/jump faults, cutoff concurrent with scoring, fixed TU goal/error
accounting, authoritative final selection, mask construction, snapshot alltrain
deduplication and an end-to-end synthetic evaluator pass.

## Proposed smoke commands after WORK acceptance

Prerequisites: clean R003 worktree; ROS 2 Jazzy stack built/sourced; simulation
launched with `launch/slam.launch.py world:=new_room`; Nav2/map/TF/odom ready;
MapEx reference checkout and alltrain checkpoint available; exact effective
launch, SLAM, merged Nav2, checkpoint and simulator versions retained by run
metadata.

Use separate run IDs/namespaces:

```bash
python3 mapex_lab/scripts/nf_run.py \
  --run-id nf_p1000_smoke_001 --environment new_room \
  --runtime-profile slam --paper1000

python3 mapex_lab/scripts/mapex_run.py \
  --run-id mpx_p1000_smoke_001 --environment new_room \
  --runtime-profile slam --paper1000 --save-predictions

python3 mapex_lab/scripts/predict_alltrain_offline.py \
  mapex_lab/experiments/nearest/nf_p1000_smoke_001 \
  --paper1000-snapshots --checkpoint <ALLTRAIN_MODEL_OR_CKPT> --device <DEVICE>
python3 mapex_lab/scripts/evaluate_r003_paper1000.py \
  mapex_lab/experiments/nearest/nf_p1000_smoke_001

python3 mapex_lab/scripts/predict_alltrain_offline.py \
  mapex_lab/experiments/mapex/mpx_p1000_smoke_001 \
  --paper1000-snapshots --checkpoint <SAME_ALLTRAIN_MODEL_OR_CKPT> --device <SAME_DEVICE>
python3 mapex_lab/scripts/evaluate_r003_paper1000.py \
  mapex_lab/experiments/mapex/mpx_p1000_smoke_001
```

## Review points / unresolved runtime evidence

- The dedicated callbacks and synthetic concurrency test demonstrate the
  intended non-blocking design; real callback scheduling and cancellation still
  require the gated 1+1 ROS smoke.
- Smoke must verify cutoff detection overshoot, post-cancel overshoot and map age
  against the approved limits. No such values are claimed yet.
- Integrity guards currently treat odom stamp gaps >1.0 s and single XY jumps
  >2.0 m as invalid evidence; these engineering guard thresholds are recorded in
  code and should be included in WORK review.
- The observed overlay shows thin boundary differences but no gross frame error;
  WORK owns the mask acceptance decision.

No existing result/workbook sheet was changed. No official attempt was deleted
or replaced. Bulk `10+10` remains blocked on accepted smoke evidence.
