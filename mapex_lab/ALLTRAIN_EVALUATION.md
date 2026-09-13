# Paper-style all-training IoU / TU re-evaluation

The primary NF-vs-MapEx IoU/TU comparison must use the same whole-training
LaMa predictor for both methods. The all-training prediction is an **offline
evaluation step**; it does not change NF frontier selection or MapEx online
policy execution.

## Pipeline

```text
saved observed raw map
        |
        v
same big_lama / all-training checkpoint
        |
        v
predicted occupancy map
        |
        +--> occupied IoU vs structural GT
        +--> TU vs structural GT
```

`mapex_lab/scripts/predict_alltrain_offline.py` now accepts both logging layouts:

- MapEx: `decisions.csv` + `decision_id`
- Nearest Frontier: `policy_decisions.csv` + `policy_decision_id`

The generated manifest records the decision-log hash, checkpoint hash, raw-map
path and raw-map hash so a prediction cannot silently be attached to a different
run or decision.

## One NF run

From the repository root, run prediction with the Python environment that can
load the original MapEx LaMa model:

```bash
python3 mapex_lab/scripts/predict_alltrain_offline.py \
  mapex_lab/experiments/nearest/nf_001 \
  --mapex-root ~/MapEx \
  --checkpoint pretrained_models/weights/big_lama \
  --device cuda:0
```

Then evaluate with the project Python environment:

```bash
python3 mapex_lab/scripts/evaluate_nf_profiled.py \
  mapex_lab/experiments/nearest/nf_001 \
  --ground-truth mapex_lab/ground_truth/new_room/generated/new_room_structural_gt_v1.npz \
  --roi mapex_lab/ground_truth/new_room/generated/new_room_connected_free_v1.npy
```

The primary columns in `evaluation.csv` are then `prediction_source=alltrain`.
For audit, the old observed-map values are retained separately in
`observed_occupied_iou` and `observed_tu`.

## All 10 NF runs

```bash
for i in $(seq -w 1 10); do
  run="mapex_lab/experiments/nearest/nf_0${i}"
  python3 mapex_lab/scripts/predict_alltrain_offline.py "$run" \
    --mapex-root ~/MapEx \
    --checkpoint pretrained_models/weights/big_lama \
    --device cuda:0

done
```

If the directories are named `nf_001` ... `nf_010`, an alternative that avoids
shell-width differences is:

```bash
for n in $(seq 1 10); do
  printf -v id 'nf_%03d' "$n"
  run="mapex_lab/experiments/nearest/$id"
  python3 mapex_lab/scripts/predict_alltrain_offline.py "$run" \
    --mapex-root ~/MapEx \
    --checkpoint pretrained_models/weights/big_lama \
    --device cuda:0

done
```

Then:

```bash
for n in $(seq 1 10); do
  printf -v id 'nf_%03d' "$n"
  run="mapex_lab/experiments/nearest/$id"
  python3 mapex_lab/scripts/evaluate_nf_profiled.py "$run" \
    --ground-truth mapex_lab/ground_truth/new_room/generated/new_room_structural_gt_v1.npz \
    --roi mapex_lab/ground_truth/new_room/generated/new_room_connected_free_v1.npy

done
```

## MapEx runs

Use the same prediction script on `mpx_001` ... `mpx_010`, then run
`evaluate_mapex_run.py`. When a valid `evaluation/alltrain/manifest.json` exists,
the MapEx evaluator already prefers `alltrain_map` over the online ensemble
`mean_map`.

```bash
for n in $(seq 1 10); do
  printf -v id 'mpx_%03d' "$n"
  run="mapex_lab/experiments/mapex/$id"
  python3 mapex_lab/scripts/predict_alltrain_offline.py "$run" \
    --mapex-root ~/MapEx \
    --checkpoint pretrained_models/weights/big_lama \
    --device cuda:0
  python3 mapex_lab/scripts/evaluate_mapex_run.py "$run" \
    --ground-truth mapex_lab/ground_truth/new_room/generated/new_room_structural_gt_v1.npz \
    --roi mapex_lab/ground_truth/new_room/generated/new_room_connected_free_v1.npy

done
```

## Existing alltrain directory

The prediction script refuses to overwrite `evaluation/alltrain` by default.
If regeneration is intentional, add:

```text
--overwrite-alltrain
```

This deletes only that run's generated `evaluation/alltrain` directory before
recreating it. It does not modify the recorded raw maps or decision logs.

## Important interpretation

Do not mix these two metric sources in the main NF-vs-MapEx result table:

```text
NF observed SLAM IoU           (legacy diagnostic)
MapEx ensemble-mean IoU        (online prediction diagnostic)
```

For the paper-style comparison, both methods must report:

```text
prediction_source = alltrain
```

using the same checkpoint, GT, ROI, threshold, TU goal count and TU seed.
