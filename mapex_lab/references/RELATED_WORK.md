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
