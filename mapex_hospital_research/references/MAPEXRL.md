# MapExRL — Human-Inspired Indoor Exploration with Predicted Environment Context and Reinforcement Learning

## Citation
Narek Harutyunyan, Brady Moon, Seungchan Kim, Cherie Ho, Adam Hung, Sebastian Scherer. "MapExRL: Human-Inspired Indoor Exploration with Predicted Environment Context and Reinforcement Learning." ICAR 2025.

## Links
- arXiv: https://arxiv.org/abs/2503.01548
- PDF: https://arxiv.org/pdf/2503.01548
- Project page: https://mapexrl.github.io/

## Main idea
MapExRL extends prediction-guided indoor exploration toward longer-horizon decision making. A learned policy uses global map predictions together with prediction uncertainty, estimated sensor coverage, frontier distance, remaining distance budget, and other frontier-scoring context to estimate the strategic value of frontier choices.

## Relation to MapEx
MapEx uses a greedy frontier score at each decision step. MapExRL explicitly targets the limitation that locally attractive frontier choices may not be best over the remaining exploration budget.

## What problem it already addresses
This paper is important before claiming novelty around:
- long-horizon / non-greedy frontier selection;
- learned frontier scoring using predicted environment context;
- remaining exploration budget;
- strategic value beyond a single-step information-gain score.

## What it does not automatically settle for this workspace
It does not prove which component of MapEx fails on Hospital. It also does not remove the need to diagnose prediction, uncertainty calibration, visibility estimation, information gain, score normalization, or WFD candidate quality on our TurtleBot4/Hospital setup.

## Research-use rule
If the Hospital diagnosis points to a greedy/long-horizon failure, compare the proposed idea against MapExRL before calling it a research gap. If the diagnosed bottleneck is uncertainty calibration, visibility error, or WFD candidate limitation, MapExRL is related context but not necessarily a direct solution.
