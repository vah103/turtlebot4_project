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

`mapex_lab/scripts/predict_alltrain_offline.py` supports both logging layouts:

- MapEx: `decisions.csv` + `decision_id`
- Nearest Frontier: `policy_decisions.csv` + `policy_decision_id`

The generated manifest records the decision-log hash, checkpoint hash, raw-map
path and raw-map hash so a prediction cannot silently be attached to a different
run or decision.

## Recommended: re-evaluate all 20 saved runs in one command

Run this with the Python environment that can load the original MapEx LaMa
checkpoint. The model is loaded once and reused for all supplied runs:

```bash
python3 mapex_lab/scripts/reevaluate_alltrain.py \
  mapex_lab/experiments/nearest/nf_* \
  mapex_lab/experiments/mapex/mpx_* \
  --ground-truth mapex_lab/ground_truth/new_room/generated/new_room_structural_gt_v1.npz \
  --roi mapex_lab/ground_truth/new_room/generated/new_room_connected_free_v1.npy \
  --mapex-root ~/MapEx \
  --checkpoint pretrained_models/weights/big_lama \
  --device cuda:0
```

The batch tool fails if an evaluator does not return `prediction_source=alltrain`,
so an observed-map NF result or online ensemble-mean MapEx result cannot silently
enter the paper-style batch.

If alltrain predictions already exist and only the metrics need to be refreshed:

```bash
python3 mapex_lab/scripts/reevaluate_alltrain.py \
  mapex_lab/experiments/nearest/nf_* \
  mapex_lab/experiments/mapex/mpx_* \
  --ground-truth mapex_lab/ground_truth/new_room/generated/new_room_structural_gt_v1.npz \
  --roi mapex_lab/ground_truth/new_room/generated/new_room_connected_free_v1.npy \
  --evaluate-only
```

## One NF run

Generate the all-training prediction:

```bash
python3 mapex_lab/scripts/predict_alltrain_offline.py \
  mapex_lab/experiments/nearest/nf_001 \
  --mapex-root ~/MapEx \
  --checkpoint pretrained_models/weights/big_lama \
  --device cuda:0
```

Then evaluate:

```bash
python3 mapex_lab/scripts/evaluate_nf_profiled.py \
  mapex_lab/experiments/nearest/nf_001 \
  --ground-truth mapex_lab/ground_truth/new_room/generated/new_room_structural_gt_v1.npz \
  --roi mapex_lab/ground_truth/new_room/generated/new_room_connected_free_v1.npy
```

The primary columns in `evaluation.csv` are then `prediction_source=alltrain`.
For audit, the old observed-map values are retained separately in
`observed_occupied_iou` and `observed_tu`.

## MapEx runs

Use the same prediction script on `mpx_001` ... `mpx_010`, then run
`evaluate_mapex_run.py`. When a valid `evaluation/alltrain/manifest.json` exists,
the MapEx evaluator already prefers `alltrain_map` over the online ensemble
`mean_map`.

## Existing alltrain directory

The prediction script refuses to overwrite `evaluation/alltrain` by default.
If regeneration is intentional, add:

```text
--overwrite-alltrain
```

The batch tool accepts the same flag. It deletes only that run's generated
`evaluation/alltrain` directory before recreating it. It does not modify the
recorded raw maps or decision logs.

## Immediate post-run NF diagnostic

`nf_run.py` still computes the old observed-map IoU/TU immediately after a new
NF run so the existing run workflow remains usable. Those rows are explicitly
labelled `prediction_source=observed_fallback` and are **legacy diagnostics**.
After the alltrain pass, `evaluate_nf_profiled.py` overwrites the primary
`evaluation.csv` / `evaluation.json` with `prediction_source=alltrain` while
retaining the observed values in separate diagnostic columns.

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
