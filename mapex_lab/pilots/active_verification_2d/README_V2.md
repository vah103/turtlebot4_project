# V2: lỗi dự đoán dọc đường và kiểm chứng cấu trúc cục bộ

V2 tiếp tục bài toán của V1 và giữ nguyên `core.py`, `policy.py`, `predictor.py`,
`run.py`, LaMa, sensor thực thi và metric. Bản sửa nằm trong các module V2.
Kết quả V1 và budget ablation không bị ghi đè. Đây là nghiên cứu phát triển,
không tự động thay phương pháp chính hoặc trạng thái QA của Company Hub.

## Bản sửa giải quyết điều gì?

V1 chấm consequence khi đảo toàn bộ một patch và dùng bảy tia thưa ở đích để
ước lượng visibility. V2 đánh giá phần đường đi robot thực sự đi được trong
budget; ước lượng phép đo dọc đường và phần patch có thể được đo. Các goal
trùng prefix được gộp. Ưu tiên route ít nhất 1 m nếu có, tránh lấy một lợi ích
giả định rất nhỏ chia cho chi phí một cell.

Một logistic error estimator dùng sáu feature: nhãn predicted occupied, độ
mơ hồ mean, ensemble variance, vote disagreement, khoảng cách tới known cells
và mật độ cạnh cục bộ. Fit offline trên unknown cells của sáu snapshot V1,
trọng số bằng nhau cho mỗi warm state; tối đa 20.000 cells/state, sampling seed
cố định, L2=0.01. Leave-layout-out là diagnostic, không dùng để chọn hyperparameter.
Ground truth của layout mới không tham gia fit; runtime chỉ dùng `PolicyInput`
và coefficients đã lưu. Không fine-tune LaMa.

## Năm phương pháp và phép ablation

| Method | Thông tin được chấm | Tập route |
|---|---|---|
| `mapex` | Variance trong visibility tại frontier / Euclidean distance | MapEx adapter V1 |
| `uncertainty` | Variance tại viewpoint / shortest path | Viewpoint rộng V1 |
| `route_uncertainty` | Variance trong union visibility dọc route / chi phí | Chung cho ba method V2 |
| `route_error` | Diện tích lỗi ước lượng có thể nhìn dọc route / chi phí | Chung cho ba method V2 |
| `structural_v2` | Route error + tác động của sửa một phần cấu trúc / chi phí | Chung cho ba method V2 |

`route_uncertainty` lấy cảm hứng từ PIPE; hard-map first-hit scans, sampling
và candidate pool khác PIPE, nên không nhận là exact reproduction. Pathwise
information gain đã có trong [PIPE](https://arxiv.org/abs/2503.07504); riêng phần
này không được gọi là đóng góp mới.

Với route a, gọi A(a) là tổng estimated error × diện tích cell trong visibility
mask, C(a) là chiều dài prefix khả thi. Điểm `route_error` là A(a)/C(a).

Mỗi hypothesis h tạo một alternative map bằng cách mở/đóng unknown patch.
R_base, R_alt là reachable component có xét footprint, từ pose hiện tại.
R_partial là reachable component sau khi chỉ sửa các patch cells dự kiến đo
được trong alternative. Direct correction proxy là:

`ΔL_h(a) = max(0, distance(R_base,R_alt) - distance(R_partial,R_alt))`

Distance là diện tích symmetric difference trên unknown cells. Chưa quan sát
ngoài patch và các dự đoán còn lại được giữ cố định. Điểm Structural V2:

`[A(a) + max_h(q_h × ΔL_h(a))] / C(a)`

q_h là **mean estimated cell error** trên phần patch thay đổi. Nó chỉ là trọng
số heuristic, không phải xác suất cả giả thuyết đúng. Thêm hai diện tích vào
score cũng là lựa chọn heuristic (weight=1), không phải Bayes risk guarantee.
Đây là thành phần phải chứng minh có lợi ích riêng so với `route_error`.

Route scans dùng 360 tia, half-cell sampling trên hard mean map, checkpoint
1 m và endpoint. Patch dùng target-directed first-hit rays trong alternative.
Đó là approximation; simulator thực thi vẫn dùng 2.500 tia và đo mỗi 0.3 m như
V1. Hypothesis không sửa known cells. Không có STOP.

## Cohort và chống lẫn dữ liệu

- Development: sáu trạng thái V1 của New Room/KTH 50052751/752. Fit model và
  chạy ba method V2. Tái dùng 12 baseline V1 có hash, không gọi chúng là run mới.
- New-layout check: KTH 50015847/753/754, hai warm states 5 m/15 m, năm methods.
  Tất cả 30 nhánh chạy mới với budget thêm 8 m. IDs được cố định trước inference;
  không chọn theo prediction error hoặc outcome.
- `protocol_v2.json`, model coefficients, source và assets được commit/seal
  trước inference confirmation. Không retune sau khi xem kết quả.
- "New" là mới với policy/error estimator trong pilot này; chưa xác nhận là
  building holdout hoặc LaMa-training holdout. Hai warm states một layout phụ
  thuộc nhau. Số layout nhỏ, không suy diễn thống kê tổng quát.

Preprocessing giống V1 và kiểm tra tái tạo asset V1 đúng tuyệt đối. Layout có
kích thước lẻ được pad occupied/invalid trước reduce 2×2. Không nhận là cùng
physical scale hay exact benchmark setting với paper gốc.

## Chạy lại

Từ root repo, driver Python có NumPy/SciPy/Matplotlib/Pillow/Shapely và worker
LaMa riêng. Các lệnh tạo assets/model từ chối overwrite dữ liệu đã freeze;
dùng output mới hoặc checkout mới khi tái lập.

```bash
python -m unittest mapex_lab.pilots.active_verification_2d.test_core \
  mapex_lab.pilots.active_verification_2d.test_budget_ablation \
  mapex_lab.pilots.active_verification_2d.test_v2 \
  mapex_lab.pilots.active_verification_2d.test_navigation_audit -v
V2_OUTPUT="mapex_lab/pilots/active_verification_2d/results/pilot_v2_repeat"
mkdir -p "$V2_OUTPUT"
cp mapex_lab/pilots/active_verification_2d/results/pilot_v2/error_model.json "$V2_OUTPUT/"
OMP_NUM_THREADS=4 MKL_NUM_THREADS=4 OPENBLAS_NUM_THREADS=4 \
python -m mapex_lab.pilots.active_verification_2d.run_v2 \
  --mapex-root "$MAPEX_ROOT" --worker-python "$MAPEX_LAMA_PYTHON" --output "$V2_OUTPUT"
python -m mapex_lab.pilots.active_verification_2d.validate_v2 --output "$V2_OUTPUT"
python -m mapex_lab.pilots.active_verification_2d.analyze_v2 --output "$V2_OUTPUT"
python -m mapex_lab.pilots.active_verification_2d.render_v2 --output "$V2_OUTPUT"
```

Assets/model nhẹ đã có trên branch. Cần raw V1 snapshots trên DELL để replay
development và xác minh model provenance; checkpoint weights không commit.
`prepare_assets_v2.py` giữ recipe tạo assets, `calibrate_v2.py --output <new_dir>`
cho phép fit lại từ V1 inputs; cả hai từ chối overwrite dữ liệu đã freeze.

## Evidence

`results/pilot_v2/` chứa model, seal, cross-layout diagnostics và validation.
Hai thư mục `development/` và `confirmation/` giữ CSV/JSON, rows/action logs,
hình; raw arrays và cache giữ trên DELL. Cache chung làm thời gian suy luận theo
nhánh phụ thuộc thứ tự chạy; không dùng chúng để so tốc độ thuật toán/robot.

`analyze_v2.py` thêm audit tĩnh 40 goal/state từ vùng truth-reachable bằng seed
cố định, dùng cùng goals cho mọi method, và kiểm tra đường dự đoán có đụng
vật cản thật không. Đây là **diagnostic bổ sung trong vòng phát triển**, không
thay endpoint chính đã chốt, không phải thực thi navigation, và không phải
100-goal TU paper reproduction. Metric risk trên layout mới chỉ được tính
offline sau quyết định; không cập nhật estimator hoặc policy.
