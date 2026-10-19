# G2-R3 — Global Conditional Frontier Sequence (precommitted)

**19-10-2026; native New Room 2D; build R3, run NR-G2SEQ-02**

## 1. Why this design

PVU-R1/R2 only changed LOCAL. PVU-R2 reduced LOCAL travel by 6.725 m (not an equal-tasks comparison) but increased GLOBAL by 73.373 m. At R2 D21, official GLOBAL WFD near goals were still available when MapEx selected a distant goal. Static current-map shadow on three runs revealed that always visiting near first is wrong: with a fixed third endpoint, sometimes far-first has lower native A* travel.

R3's **scope is deliberately GLOBAL only**. The original Run04 G1 V4-R4.1 room mask and LOCAL official WFD frontier/scorer are not edited. Entire LaMa 3-model pipeline, worker, sensor, predictor, native clearance-weighted 4-neighbor A*, goal arrival threshold and global fallback are unchanged.

## 2. Inputs & outputs

At GLOBAL decision t:
- observed P1 grid at that instant; same-epoch LaMa mean and variance (existing three-model ensemble);
- robot pose, native MapEx planner's FULL original weighted planning grid; official GLOBAL WFD candidate list and original MapEx costs;
- LiDAR range 20m (same config), 0.1m P1, native goal arrival radius.

Returns: index of FIRST official WFD frontier to execute (or original first), and full per-candidate audit. R3 does not dispatch a tour; after LiDAR, MapEx recomputes G1 and R3 from a new epoch.

No future snapshot, GT, manually chosen New Room coordinates, hindsight room masks or offline route is allowed into runtime policy.

## 3. Precommitted decision rule

Definitions:
- S = current robot; F = original best GLOBAL frontier (argmin original MapEx cost); N = another official frontier whose current-native-A* geometrical route from S is **strictly shorter** than S→F; E = continuation frontier.
- For performance, examine at most six N candidates in ascending route length. Ignore N within native goal-arrival radius.
- Use exactly original 3×3 inflation + distance-transform clearance weights and 4-connected pyastar2d; native costmap treats UNKNOWN as passable. As extra **robustness guard for the newly introduced detour**, accept N only if S→N and N→F run entirely through currently **observed FREE**. Original S→F may still pass UNKNOWN, as native MapEx allows.
- E must be two **distinct** official WFD candidates currently reachable from F and N: (i) candidate geographically nearest F by native A*, (ii) candidate with lowest current original MapEx score among other WFD. If these are the same endpoint (or fewer than two) → original F.
- Predicted visibility is a coarse 180-ray simulation of same-epoch LaMa mean, observed walls, 20m range and path sample every 50 native cells. Expected var gain counts UNKNOWN at most once per combined path:
  V_NF = visible_UNKNOWN(S→N ∪ N→F); V_F = visible_UNKNOWN(S→F).
  Unique = Σ variance((V_NF \ V_F) ∩ observed_UNKNOWN)
  Fraction r = Unique / Σ variance(V_NF), capped to [0,1].
- Important: **r is NOT a calibrated probability** that N will remain an unresolved frontier. It is a heuristic normalized *visibility residual*. The 180-ray surrogate overpredicted at D21 (56 cells predicted vs 11 actually from exact native LiDAR).
- Distances d(X,Y) are physical 0.1m × native path steps along that fixed weighted-A* route, NOT plain shortest Euclidean path.

Compare for **each** E:
C_near = d(S,N)+d(N,F)+d(F,E).
C_far_proxy = d(S,F)+d(F,E)+r·max(0, d(F,N)+d(N,E)-d(F,E)).
M_E = C_far_proxy−C_near.

R3 changes selected FIRST goal from F to N **only if** Unique>0, both E are distinct, every M_E>0 and detour paths are observed FREE. Among accepted N, choose the greatest **minimum** M_E; tie breaks by extra predicted variance, original MapEx cost and index. Otherwise **return original F**. No externally hand-tuned New Room gain cutoff: comparison is positive-vs-nonpositive.

Subsequently only that goal is dispatched; native MapEx's A* navigation, LiDAR and G1 continue unchanged.

## 4. Verification

- Unit sanity for shapes, insufficient WFD fallback, and an **artificial off-route UNKNOWN pocket**: forced near-before-far should actually change first goal in a geometry where both E agree. Synthetic PASS before full native.
- Read-only shadow across 3 archived native runs: 19 GLOBAL decisions after initial t0 were examined; **0/19** met all guards. Shadow is a falsifiable **negative**, not evidence of superiority. No parameters changed to make R3 pass.
- Stage immutable NR-G2SEQ-02 profile with hash-sealed policy/helper. Model smoke PASS (3 LaMa models), D01–D02 numeric and room/pose parity PASS in guarded native run.
- Full-run metrics/result: enter only after runtime receipt; do not assume same traversal based merely on shadow. All freezes reside under /home/dell/DELL_NATIVE2D/runs/NR-G2SEQ-02/.

## 5. Interpretation, anticipated limitations

This is intentionally a **conservative feasibility test**. Existing data indicate many potential N goals will not pass both endpoint tests: the current-map visibility proxy predicts a low residual fraction and candidate continuation endpoints can disagree about preferable order. Thus R3 can have zero interventions on New Room even when a hindsight itinerary is shorter. If native result matches Run04, declare **NO_ACTION**, not a G2 victory or evidence MapEx+G1 optimal.

Scientific next options require **independent calibration of residual need**, accounting for a frontier disappearing after planned F scans, and allowing alternative continuation anchor strategies that are fixed BEFORE benchmark runs. Hospital snapshots/2D native will be needed for generalizability. Do not tune merely to D21.

## 6. Stored artifacts

- Algorithm: \`/home/dell/DELL_NATIVE2D/build/g2_global_order_r3.py\`
- Stage/execute: \`/home/dell/DELL_NATIVE2D/build/g2_r3_deploy.py\`
- Shadow: \`/home/dell/DELL_NATIVE2D/analysis/g2_autonomous_research_20261018/NR_G2_R3_GLOBAL_CONDITIONAL_SHADOW.json\`
- Native frozen run: \`/home/dell/DELL_NATIVE2D/runs/NR-G2SEQ-02/\`
- Reference frozen Run04: \`/home/dell/DELL_NATIVE2D/runs/NR-RN-04/\`.
