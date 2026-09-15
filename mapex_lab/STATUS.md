# mapex_lab status

_Last synchronized with `main`: 2026-09-15._

This file records the current code/research state on `main`.

## Current focus

The active thesis direction is **Direction 2: prediction/uncertainty-aware early stopping**.

Research question: **When has MapEx learned enough that further physical exploration is no longer worth its time/distance cost while reconstructed map quality remains acceptable?**

The online MapEx policy has **not** yet been changed to stop early. Stopping remains an offline hypothetical analysis until cross-environment validation is completed.

## Experiment runner

Normal entry point:

```bash
./mapex_lab/run
```

Current interactive flow:

1. choose `NF` or `MapEx`;
2. choose `New Room` or `Hospital`;
3. choose whether to save a record;
4. enter a run ID when recording.

Current map/profile mapping:

- New Room -> `world:=new_room`, environment `new_room`, runtime profile `slam`;
- Hospital -> `world:=hospital`, environment `hospital`, runtime profile `hospital_slam`.

The runner launches `launch/slam.launch.py`, waits for `/map` and `/navigate_to_pose`, then dispatches the selected exploration method. Existing run directories are not overwritten.

## Core MapEx state

- `scripts/mapex.py` — MapEx exploration/frontier ranking.
- `scripts/mapex_run.py` — recording and evaluation wrapper.
- `scripts/mapex_lama_bridge.py` / `mapex_lama_worker.py` — LaMa inference path.
- Online MapEx uses the **3-model ensemble**.
- All-training prediction is an offline evaluation path, not a fourth online ensemble member.

## Early-stopping tooling

Implemented tools:

- `scripts/analyze_early_stopping.py`
- `scripts/sweep_early_stopping_thresholds.py`
- `scripts/validate_early_stopping_loocv.py`
- `scripts/compare_fixed_early_stopping_rules.py`

Main online-available signal under study:

```text
unknown_variance_p95
```

The stop rule form is:

```text
P95 <= threshold for K consecutive distinct decision states
```

Repeated identical terminal policy ticks are collapsed before persistence counting.

## Development/tuning set — New Room mpx_001...mpx_010

`mpx_001...mpx_010` are the **development/tuning dataset** because they were used to inspect P95 behavior and select thresholds. They are not an independent validation set.

Fixed-rule development results:

| Rule | Time saved | Distance saved | Mean IoU loss | Worst IoU loss |
|---|---:|---:|---:|---:|
| `P95 <= 0.23, K=1` | 23.64% | 24.03% | -0.00275 | 0.00938 |
| `P95 <= 0.25, K=2` | 22.23% | 22.67% | -0.00285 | 0.00977 |
| `P95 <= 0.20, K=1` | 17.28% | 16.90% | -0.00030 | 0.00540 |

Current labels:

- **Primary frozen candidate:** `P95 <= 0.23, K=1`
- **Persistence challenger:** `P95 <= 0.25, K=2`
- **Conservative baseline:** `P95 <= 0.20, K=1`

LOOCV showed that repeatedly selecting the maximum-savings rule on only nine training runs can become too aggressive. Therefore validation uses the fixed candidate rather than retuning the rule for each validation set.

## Prospective New Room validation — mpx_011...mpx_015

Five new runs were collected after freezing `P95 <= 0.23, K=1`.

| Run | Time saved | Distance saved | IoU loss vs analyzer final reference |
|---|---:|---:|---:|
| `mpx_011` | 18.69% | 20.72% | +0.000053 |
| `mpx_012` | 27.62% | 25.39% | -0.003336 |
| `mpx_013` | 21.67% | 23.28% | -0.001435 |
| `mpx_014` | 24.77% | 27.81% | -0.002251 |
| `mpx_015` | 29.10% | 29.58% | -0.003084 |

Aggregate:

- trigger rate: **5/5**;
- mean time saved: **24.37%**;
- mean distance saved: **25.36%**;
- mean IoU loss: **-0.00201**;
- worst IoU loss: **+0.000053**;
- IoU loss `> 0.01`: **0/5**;
- TU loss: **0 on all 5 runs**.

This is strong **within-environment prospective validation** on New Room, but it does not establish cross-environment generalization.

Development vs prospective validation are close:

```text
Development 001-010: time 23.64%, distance 24.03%, mean IoU loss -0.00275
Validation  011-015: time 24.37%, distance 25.36%, mean IoU loss -0.00201
```

Further New Room repetition is now lower priority than testing another environment.

## Hospital — canonical 1.0x state

Hospital is now configured at **scale 1.0x**.

Canonical runtime state:

- world: `map/hospital_aws_flat.sdf`;
- Hospital scale: `1.0`;
- TurtleBot4 is not scaled;
- spawn: `(0.0, 12.0, -1.57)`.

Hospital ground-truth tooling:

- `scripts/generate_hospital_ground_truth.py`
- `scripts/hospital_ground_truth_core.py`

The generator follows the historical canonical ROI-generation pipeline: COLLADA scene loaded with `trimesh`, wall slice at `z=0.30 m`, OpenCV rasterization, elevator blockers, crack closing and 8-connected free-space extraction from the SLAM start.

Verified locally on `com1` on 2026-09-15:

```text
status: ok_frozen_v1_match
hospital_scale: 1.0
ROI cells: 215435
ROI SHA-256: 05d45b7aba66cb6dbb71e0406005f4a3e21875901af3ae72164b17f1d3add8d1
frozen_v1_match: true
wall segments: 1040
elevator blockers: 2
structural evaluation-mask cells: 245623
structural occupied cells: 30188
```

Generated local artifacts:

```text
mapex_lab/ground_truth/hospital/generated/
├── hospital_connected_free_v1.npy
├── hospital_connected_free_v1_metadata.json
├── hospital_structural_gt_v1.npz
├── hospital_structural_gt_v1_preview.png
└── hospital_structural_gt_v1_summary.json
```

These heavy generated artifacts remain local and are not committed.

The active Hospital world uses 1.8 m elevator blockers, and the regenerated ROI still reproduces the exact frozen-v1 cell count and SHA.

## Current cross-environment validation plan

Next formal experiment: **Hospital**.

Protocol:

- use canonical Hospital **1.0x**;
- use the verified Hospital frozen ROI/structural GT;
- keep MapEx/LaMa and the established runtime/recording semantics unchanged;
- freeze the stopping rule at **`P95 <= 0.23, K=1`**;
- do **not** retune the threshold on Hospital before evaluating it;
- run normal full MapEx exploration; stopping remains offline/hypothetical;
- initially collect about **3–5 Hospital validation runs**.

The purpose is to test whether the absolute P95 threshold transfers beyond New Room. If it fails, that is evidence that an environment-normalized uncertainty criterion may be needed instead of a universal absolute threshold.

## Evaluation caveat

Current `iou_loss_vs_final` uses the reconstructed map at the **last analyzed policy decision** as its reference, not necessarily the actual final fully observed SLAM map.

Therefore a negative IoU loss means the earlier hypothetical-stop reconstruction outperformed the last-decision reconstruction under analyzer semantics; it does not automatically mean it outperformed the actual final observed map.

TU remains weakly discriminative in the current New Room dataset and should not be the main stopping signal.

## Next actions

1. Keep `mpx_001...010` strictly as development/tuning data.
2. Keep `mpx_011...015` as prospective New Room validation data; do not use them to tune the threshold.
3. Start Hospital cross-environment validation with the frozen rule `P95 <= 0.23, K=1`.
4. Collect about 3–5 full Hospital MapEx runs before deciding whether more repetitions are useful.
5. Keep early stopping offline until Hospital results are understood.
6. Verify the unusually large New Room accumulated trajectory distances before making final absolute-distance thesis claims.
7. Improve the final IoU reference before making strong final map-quality claims.
8. Integrate online stopping into `mapex.py` only after cross-environment validation is satisfactory.
