# Versioned LaMa samples

This directory contains three representative checkpoints from the completed
Hospital run `hospital_flat_lama_01_001`:

| Frame | Stage | Files |
| --- | --- | --- |
| `000050` | Early exploration | `input.png`, `mask.png`, `prediction.png` |
| `000708` | Middle exploration | `input.png`, `mask.png`, `prediction.png` |
| `001415` | Final state | `input.png`, `mask.png`, `prediction.png` |

The sample manifest documents the source run, fixed-canvas geometry, image
dimensions, file roles and checksums. Full raw snapshots remain local under
`data/lama_runs/` and are ignored by Git.

These files are evidence for qualitative LaMa evaluation. They are not a
replacement for the full 1,416-frame dataset or a quantitative benchmark.
