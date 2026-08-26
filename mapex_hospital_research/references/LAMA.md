# LaMa — Resolution-robust Large Mask Inpainting with Fourier Convolutions

## Citation
Roman Suvorov, Elizaveta Logacheva, Anton Mashikhin, Anastasia Remizova, Arsenii Ashukha, Aleksei Silvestrov, Naejin Kong, Harshith Goka, Kiwoong Park, Victor Lempitsky. "Resolution-robust Large Mask Inpainting with Fourier Convolutions." 2021.

## Links
- arXiv: https://arxiv.org/abs/2109.07161
- PDF: https://arxiv.org/pdf/2109.07161
- Official code: https://github.com/advimman/lama

## Main idea
LaMa is an image-inpainting method designed for large missing regions. It uses Fast Fourier Convolutions (FFC) to obtain a large effective receptive field, together with a high-receptive-field perceptual loss and large training masks.

## Relation to MapEx
MapEx adapts a LaMa-style inpainting network for global occupancy-map completion. In the MapEx pipeline, map predictions are not the final exploration decision by themselves: predictions feed the ensemble mean/variance, probabilistic visibility estimation, information gain, and frontier scoring.

## Why it matters for this workspace
This paper becomes especially important if diagnosis points to:
- map-prediction error / Hospital domain shift;
- hallucinated or missing structures;
- ensemble members sharing similar prediction bias;
- checkpoint/fine-tuning/provenance questions.

## What LaMa does NOT solve
LaMa itself does not solve frontier generation, exploration scoring, visibility raycasting, travel cost, Nav2 reachability, or closed-loop exploration policy design.

## Research caution
A failure in MapEx downstream ranking must not automatically be attributed to LaMa. The workspace roadmap requires measuring prediction error and then using component/oracle ablations before selecting prediction as the bottleneck.
