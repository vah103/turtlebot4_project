# mpx_w2_002

Method: MapEx Way2
Environment: New Room
Launcher: ./run

Result:
- Way2 early stopping did NOT trigger.
- Exploration terminated through the existing stable no-frontier completion logic.
- Reason: no frontier region > 10 cells remained stably.
- Stable sweeps: 5
- Idle confirmation: >= 10 s

Final recorded values:
- coverage: 0.838430391
- distance: 309.93 m
- policy decisions: 35
- offline IoU: 0.139368
- TU: 0.040000

This run uses the frozen Way2 rule without retuning.
