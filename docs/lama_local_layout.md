# Project-local LaMa layout

LaMa code, snapshots and results use paths inside this repository:

```text
turtlebot4_project/
  third_party/lama/       local LaMa runtime, model and environment (ignored)
  third_party/lama_upstream/ pinned upstream source (Git submodule)
  data/lama_runs/         full snapshot runs and generated PNG pairs (ignored)
  data/lama_samples/      small reviewed samples that may be committed
  models/lama/            optional extracted checkpoints (ignored)
  results/lama/           generated evaluation output (ignored)
  scripts/lama/           setup and migration scripts (tracked)
```

## One-time migration

From the repository root, first copy and checksum-verify both legacy folders:

```bash
bash scripts/lama/migrate_local_assets.sh
```

After confirming LaMa and the snapshot runs work from the new paths, repeat
with `--move` to remove the two verified legacy folders:

```bash
bash scripts/lama/migrate_local_assets.sh --move
```

`--move` deletes each legacy source only after `rsync` checksum verification.
It maps:

- `~/lama` -> `third_party/lama`
- `~/turtlebot4_lama_snapshots` -> `data/lama_runs`

The source paths can be overridden with `--lama-source` and
`--snapshot-source`.

## Runtime paths

Set the project root once in each terminal:

```bash
cd ~/turtlebot4_project
export TURTLEBOT4_PROJECT_ROOT="$(pwd)"
```

The snapshot recorder now defaults to the relative path `data/lama_runs`.
Relative paths are anchored at `TURTLEBOT4_PROJECT_ROOT`; if the variable is
unset, the recorder searches parent directories of the working directory and
installed module for the repository marker. Absolute `output_dir` overrides
remain supported.

Useful variables for manual LaMa commands:

```bash
export LAMA_ROOT="${TURTLEBOT4_PROJECT_ROOT}/third_party/lama"
export LAMA_RUNS_ROOT="${TURTLEBOT4_PROJECT_ROOT}/data/lama_runs"
export LAMA_RESULTS_ROOT="${TURTLEBOT4_PROJECT_ROOT}/results/lama"
```

Full runs, upstream source, checkpoints and generated results are deliberately
ignored. The upstream implementation is referenced as a pinned submodule at
`third_party/lama_upstream`; it is not duplicated in this repository. Git
contains integration code, configs, documentation and three reviewed samples,
not the multi-gigabyte full run.

## Chat-readable source and samples

Initialize the pinned LaMa source after cloning:

```bash
git submodule update --init --recursive
```

The three representative checkpoints from
`hospital_flat_lama_01_001` are stored under
`data/lama_samples/hospital_flat_lama_01_001/`:

- `000050`: early exploration
- `000708`: middle exploration
- `001415`: final exploration state

Each checkpoint contains the model input, binary unknown-region mask, and LaMa
prediction. `sample_manifest.json` records dimensions, geometry, completion
state and provenance so another reviewer can interpret the files without the
8.5 GiB local run.
