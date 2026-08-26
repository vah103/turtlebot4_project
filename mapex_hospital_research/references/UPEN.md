# UPEN — Uncertainty-driven Planner for Exploration and Navigation

## Citation
Georgios Georgakis, Bernadette Bucher, Anton Arapin, Karl Schmeckpeper, Nikolai Matni, Kostas Daniilidis. "Uncertainty-driven Planner for Exploration and Navigation." ICRA 2022, pp. 11295–11302.

## Links
- arXiv: https://arxiv.org/abs/2202.11907
- PDF: https://arxiv.org/pdf/2202.11907
- Project page: https://ggeorgak11.github.io/uncertainty-nav-project/
- DOI: https://doi.org/10.1109/ICRA46639.2022.9812423

## Main idea
UPEN learns occupancy priors beyond the robot's field of view and uses model/epistemic uncertainty from map prediction to guide planning. For exploration, candidate paths are evaluated using prediction uncertainty; for point-goal navigation, the method uses uncertainty-aware path objectives.

## Relation to MapEx
MapEx treats UPEN as a representative prediction-based exploration baseline. The important contrast is that UPEN emphasizes uncertainty, while MapEx additionally models what the sensor can actually observe through predicted visibility/probabilistic raycasting.

## What problem it already addresses
UPEN is important before claiming novelty around:
- ensemble/model uncertainty for exploration;
- using predicted unseen occupancy to guide exploration;
- uncertainty-driven path selection.

## Limitation relevant to MapEx
An uncertainty-only objective does not by itself guarantee that the uncertain region is actually visible/observable from the selected frontier. This is one motivation for MapEx's combination of uncertainty and sensor visibility.

## What it does not automatically settle for this workspace
UPEN does not answer whether MapEx uncertainty is calibrated on Hospital, whether probabilistic visibility is correct, or whether WFD candidates/scoring are the bottleneck. It is mainly a baseline and related-work reference unless diagnosis points toward uncertainty-driven exploration.

## Research-use rule
Use UPEN later for broader benchmarking or when the selected research direction directly concerns uncertainty-based exploration. It is not required to identify the initial Hospital bottleneck if Nearest + MapEx + oracle ablations already provide the needed causal diagnosis.
