# All-training prediction after a MapEx run

Online runs need only the existing three ensemble checkpoints and LaMa environment. No alltrain configuration or fourth checkpoint is required by the runner. The algorithm, online inference and recording remain as before this evaluation change.

After stopping/completing a run, use the LaMa environment's Python to generate predictions (replace RUN_DIR with the saved run directory):

```bash
python mapex_lab/scripts/predict_alltrain_offline.py RUN_DIR --mapex-root ~/MapEx --checkpoint pretrained_models/weights/big_lama --device cuda:0
```

This is a ROS-free offline command. It loads only the MapEx whole-training predictor. The default checkpoint layout comes from the original MapEx release; use verified KTH-fine-tuned weights. It reads every decision's raw map, retaining occupancy conversion, resolution, origin and padding. It writes `evaluation/alltrain/` without modifying decisions or metadata. Existing output directories are rejected to avoid overwriting prior evidence; after an interrupted attempt, preserve/rename that directory before retrying.

Then evaluate using the run's actual ground truth and connected-free ROI:

```bash
/usr/bin/python3 mapex_lab/scripts/evaluate_mapex_run.py RUN_DIR --ground-truth GT_NPZ --roi ROI_NPY
```

The evaluator verifies the offline manifest against the source decisions/maps. Alltrain becomes primary IoU/TU, while ensemble mean is evaluated separately on the same goals. It updates evaluation outputs and backfills primary metrics. Without an offline manifest, existing mean-based evaluation continues. Do not compare mean-based and alltrain-based scores as if their prediction source were identical.

Real inference has not yet been validated in this workspace because the default checkpoints and LaMa environment are absent. Tests cover the ROS-free recording/postprocessing data flow using synthetic predictions.
