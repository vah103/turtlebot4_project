# IG-Hector — Learned Map Prediction for Enhanced Mobile Robot Exploration

## Citation
Rakesh Shrestha, Fei-Peng Tian, Wei Feng, Ping Tan, Richard Vaughan. "Learned Map Prediction for Enhanced Mobile Robot Exploration." ICRA 2019, pp. 1197–1204.

## Links
- Preprint PDF: https://autonomy.cs.sfu.ca/doc/shrestha_icra19.pdf
- DOI: https://doi.org/10.1109/ICRA.2019.8793769
- Official code: https://github.com/rakeshshrestha31/map_prediction_enhanced_exploration

## Naming note
In MapEx's benchmark/code this baseline is commonly referred to as `IG-Hector` / `hectoraug`. The underlying paper is "Learned Map Prediction for Enhanced Mobile Robot Exploration."

## Main idea
The method detects frontier clusters, predicts unseen occupancy around frontier regions with a learned map-completion model (VAE), estimates information gain from the predicted map, and feeds that gain into a Hector-style exploration planner.

## Relation to MapEx
IG-Hector is an earlier prediction-guided frontier exploration method. MapEx uses it as a baseline representing information gain based mainly on predicted sensor coverage/structure, without the same explicit ensemble-uncertainty + probabilistic-visibility formulation used by MapEx.

## What problem it already addresses
This paper is important before claiming novelty around:
- learned map completion for frontier exploration;
- using predicted unseen structure to estimate frontier information gain;
- coupling map prediction with Hector/frontier-based exploration.

## Limitation relevant to MapEx
If predicted map structure is wrong, visibility/coverage-based utility can be wrong as well. This makes IG-Hector particularly relevant to the Hospital hypothesis that map-prediction errors can propagate into visibility and information-gain estimation.

## What it does not automatically settle for this workspace
It does not diagnose whether MapEx's failure on Hospital comes from prediction, uncertainty, visibility, score normalization, travel cost, or candidate generation. It is mainly a historical/direct baseline for prediction-based frontier scoring.

## Research-use rule
Implement/compare IG-Hector when a broader benchmark or close reproduction of MapEx baselines is needed. Initial bottleneck discovery can still prioritize Nearest + original MapEx + oracle ablations.
