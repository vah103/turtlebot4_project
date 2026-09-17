# Way2 result archive

This directory is the durable, machine-readable archive for the evidence used to freeze the Way2 early-stopping rule.

## Frozen rule

```text
R_t = max_f information_gain(f) / distance_m(f)
U_t = max_f visible_unknown_cells(f) / distance_m(f)

base_valid := (R_t <= 0.30) AND (U_t <= 10.0)

confirmation:
- first consecutive valid state: continue
- second consecutive valid state:
  - if selectable candidate_count <= 1: stop
  - otherwise require one more valid state
- third consecutive valid state: stop
```

The frozen constants are:

- `R threshold = 0.30`
- `U threshold = 10.0 cells/m`
- `candidate_count cutoff = 1`
- `second confirmation = 2`
- `third confirmation = 3`

## What belongs here

- `verified_development_results.csv`: development/tuning facts that are already supported by repository notes/status and were used to select the frozen rule.
- `quality_failure_cases.csv`: concrete failure cases that motivated the visible-unknown guard and adaptive confirmation.
- `frozen_rule.json`: canonical machine-readable copy of the frozen Way2 rule and its data-use boundary.
- `reproduce_way2_analysis.sh`: reruns the historical Way2 analysis scripts and captures their complete terminal output under `analysis/way2_results/reproduced_logs/`.

Related files one level up:

- `../WAY2_THRESHOLD_SELECTION.md`: narrative decision trail.
- `../way2_threshold_neighborhood.csv`: selected robustness-neighborhood points already preserved from the threshold-selection work.
- `../way2_online_runs.csv`: post-freeze online runs. These must not be used to retune the frozen thresholds.

## Important provenance rule

Historical development data and post-freeze online runs are intentionally separated.

Development/tuning results may explain **why the threshold was selected**. Post-freeze online runs may test the frozen rule, but must not be used to change `0.30`, `10.0`, cutoff `1`, or the `2/3` confirmation logic.

Some older analysis scripts originally printed detailed sweep tables only to the terminal. Those exact historical terminal rows cannot be reconstructed from Git history when they were never committed. Therefore this archive does not invent missing rows. Instead, `reproduce_way2_analysis.sh` captures future reruns verbatim from the preserved experiment data and scripts so the complete output can then be committed.
