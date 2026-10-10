# PIPE Planner — Pathwise Information Gain with Map Predictions for Indoor Robot Exploration

## Citation
Seungjae Baek, Brady Moon, Seungchan Kim, Muqing Cao, Cherie Ho, Sebastian Scherer, Jeong Hwan Jeon. "PIPE Planner: Pathwise Information Gain with Map Predictions for Indoor Robot Exploration." IROS 2025.

## Links
- arXiv: https://arxiv.org/abs/2503.07504
- PDF: https://arxiv.org/pdf/2503.07504
- Project page: https://pipe-planner.github.io/
- Official code: https://github.com/castacks/pipe-planner

## Main idea
PIPE evaluates expected information gain along a planned path rather than only at the final frontier/viewpoint. It estimates cumulative sensor coverage along trajectories and uses map prediction to reduce overestimation. It also emphasizes efficient computation of pathwise visibility/coverage.

## Relation to MapEx
MapEx uses pointwise information gain at a frontier and normalizes by Euclidean distance. PIPE moves toward pathwise sensor coverage and path-aware planning, making it directly relevant when MapEx selects a frontier that looks useful at the endpoint but is inefficient when actual travel is considered.

## What problem it already addresses
This paper is essential before claiming novelty around:
- pathwise information gain;
- using the whole trajectory instead of only the frontier endpoint;
- A*/path-distance-aware exploration utility;
- reducing information-gain overestimation along paths.

## What it does not automatically settle for this workspace
PIPE does not prove that Hospital failure is caused by path cost or pointwise IG. The roadmap still requires closed-loop comparison and component/oracle diagnosis. A Hospital bottleneck could instead be prediction, uncertainty, visibility, score calibration, or candidate generation.

## Research-use rule
If the diagnosis shows raw IG is good but `IG/d` or endpoint-only scoring is poor, PIPE is a high-priority related work reference and baseline/conceptual comparator before defining a new scoring/pathwise contribution.


## 2026-10-10 source/paper audit

Read paper v1 §IV-A–E and official code pin e5bcb5ec4a9a13cbe14aa9f7850bc8b5989a2bb0. Path visibility, overlap union, probabilistic occlusion handling and computational geometry are already addressed. Residual prediction error, cached-goal value, remaining-budget score support, alternative routes and downstream positioning are unverified task-level hypotheses, not omissions/novelty verdicts. Preserve source path[2::3] and sampled-pose-count denominator, and separate CODE_REFERENCE planning from ALIGNED_SHARED. See `../pilots/mx071_followup/PIPE_AUDIT_R0.md`.
