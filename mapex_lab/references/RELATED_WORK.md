# Related Work — index

File này là index để kiểm tra literature sau khi bottleneck đã được xác định. Không dùng các paper này để quyết định trước research gap.

## MapEx — phương pháp gốc

### `MAPEX_PAPER.md`
**MapEx: Indoor Structure Exploration with Probabilistic Information Gain from Global Map Predictions** — ICRA 2025.

Vai trò trong workspace:
- phương pháp gốc cần reproduce/diagnose;
- WFD candidates;
- LaMa-style global prediction ensemble;
- mean + variance;
- probabilistic visibility/raycasting;
- Information Gain;
- frontier score dựa trên gain và distance.

---

## Các nghiên cứu trực tiếp liên quan sau/trước MapEx

### `MAPEXRL.md`
**MapExRL: Human-Inspired Indoor Exploration with Predicted Environment Context and Reinforcement Learning** — ICAR 2025.

Kiểm tra paper này nếu bottleneck/hướng đề xuất liên quan:
- greedy frontier selection;
- long-horizon reasoning;
- remaining distance budget;
- learned/context-aware frontier value.

Không nên gọi long-horizon/stage-aware learned decision là research gap mới trước khi đối chiếu MapExRL.

### `PIPE_PLANNER.md`
**PIPE Planner: Pathwise Information Gain with Map Predictions for Indoor Robot Exploration** — IROS 2025.

Kiểm tra paper này nếu bottleneck/hướng đề xuất liên quan:
- pointwise vs pathwise information gain;
- Euclidean distance vs actual path cost;
- sensor coverage accumulated along a trajectory;
- path-aware frontier evaluation.

Không nên gọi pathwise IG/path-aware scoring là research gap mới trước khi đối chiếu PIPE.

### `UPEN.md`
**Uncertainty-driven Planner for Exploration and Navigation** — ICRA 2022.

Kiểm tra paper này nếu hướng đề xuất liên quan:
- prediction ensemble;
- epistemic/model uncertainty;
- uncertainty-driven exploration;
- uncertainty along candidate paths.

UPEN là baseline prediction/uncertainty quan trọng của MapEx, nhưng không thay thế diagnosis về calibration của variance trên Hospital.

### `IG_HECTOR.md`
**Learned Map Prediction for Enhanced Mobile Robot Exploration** — ICRA 2019.

Trong MapEx benchmark/code thường được gọi `IG-Hector` / `hectoraug`.

Kiểm tra paper này nếu hướng đề xuất liên quan:
- map prediction for exploration;
- predicted sensor coverage / information gain;
- learned completion feeding frontier/Hector exploration.

Đặc biệt liên quan khi nghiên cứu propagation:
prediction error → visibility/coverage error → information-gain error.

### `LAMA.md`
**Resolution-robust Large Mask Inpainting with Fourier Convolutions** — 2021.

Kiểm tra sâu nếu diagnosis chỉ ra:
- prediction/domain shift;
- hallucinated/missing structures;
- fine-tuning/checkpoint problem;
- ensemble predictors sharing systematic bias.

LaMa là map-completion architecture, không phải exploration policy.

---

## Baseline priority trong workspace

### Giai đoạn tìm bottleneck
Ưu tiên:
1. Nearest Frontier
2. MapEx original
3. structural GT / evaluation oracle
4. component/oracle ablations

UPEN và IG-Hector chưa bắt buộc ở giai đoạn đầu nếu mục tiêu chỉ là xác định bottleneck của MapEx trên Hospital.

### Giai đoạn final benchmark / publication-style comparison
Tối thiểu:
- Nearest Frontier
- MapEx original
- Proposed method

Sau đó cân nhắc thêm:
- UPEN
- IG-Hector
- MapExRL hoặc PIPE nếu chúng trực tiếp trùng với research question/proposed method.

---

## Literature decision table

| Nếu diagnosis cho thấy... | Paper phải đọc/đối chiếu trước | Lý do |
|---|---|---|
| prediction/domain shift | LaMa, MapEx | xác định vấn đề predictor/fine-tuning |
| uncertainty không phản ánh true error | UPEN, MapEx | uncertainty-driven exploration và ensemble uncertainty |
| predicted visibility/raycasting sai | MapEx, IG-Hector, PIPE | prediction-to-visibility/coverage propagation |
| endpoint IG tốt nhưng travel efficiency kém | PIPE | pathwise IG và path cost |
| greedy selection / long-horizon kém | MapExRL | learned longer-horizon frontier selection |
| WFD candidate set hạn chế | MapEx + broader viewpoint literature | vấn đề candidate generation nằm upstream ranking |

## Quy tắc cập nhật

Khi đã xác định bottleneck:
1. đọc full paper liên quan, không chỉ abstract;
2. ghi rõ paper giải quyết thành phần nào trong pipeline;
3. ghi experimental setting và metric có so được với Hospital hay không;
4. ghi phần paper chưa giải quyết;
5. chỉ sau đó mới kết luận research gap;
6. nếu thêm paper mới quan trọng, tạo file riêng trong `references/` thay vì nhồi toàn bộ vào file này.


## 2026-10-07 — active-verification exploratory pilot

The new pilot treats verification as a hypothesis to test, not an established research gap. MapEx global map prediction remains the backend. Closest references include:

- MapEx: https://arxiv.org/abs/2409.15590 — prediction/uncertainty/visibility-based exploration.
- Action-Aware Pro-Active Safe Exploration for Mobile Robot Mapping: https://arxiv.org/abs/2503.09515 — actionable information, motion costs and termination; cost-aware gating alone is not sufficient novelty.
- Hypothesis Graph Refinement: https://chenpppx.github.io/Hypothesis_Graph_Refinement/ — revisable semantic hypotheses and verification/cascade correction for embodied navigation. Hypothesis verification itself is not claimed as new here.

The pilot asks whether local structural consequence can improve action selection for predicted 2D map completion, including ensemble consensus errors. Current results do not establish superiority or novelty; see the branch-only [pilot report](../pilots/active_verification_2d/REPORT_VI.md).

### V2 design check — 2026-10-07

Read MapEx https://arxiv.org/html/2409.15590v1 and PIPE https://arxiv.org/html/2503.07504v1, including their method/experimental sections. PIPE already integrates path sensor coverage and variance and normalizes by path length. V2 therefore includes a PIPE-inspired route uncertainty control; our hard-map ray scans and candidate pool differ from its probabilistic polygon-union implementation. Pathwise scoring alone is not novel. The extra candidate contribution under test is cell-error-weighted partial local structural correction; neither its superiority nor novelty is established by implementing it.
