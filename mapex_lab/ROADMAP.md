# Research Roadmap

## Mục tiêu chung

Tìm chính xác thành phần nào trong pipeline MapEx là bottleneck khi chuyển sang Hospital, sau đó mới chọn bài toán nghiên cứu và đề xuất phương pháp cải tiến.

## Nguyên tắc

Không giả định trước prediction, uncertainty, visibility, scoring hay candidate generation là vấn đề. Xác nhận failure trong closed-loop trước, sau đó dùng phân tích từng tầng và oracle/ablation để tìm nguyên nhân.

## 1. Chuẩn hóa Hospital

Việc cần làm:
- Chạy ổn định simulation + SLAM + Nav2.
- Cố định world, spawn, sensor, resolution, SLAM/Nav2 params, timeout, stopping condition.
- Chốt rõ simulator seed policy.
- Chốt fixed logging canvas và canonical evaluation ROI/mask dùng chung.
- Chốt benchmark clock: `t=0` tại first policy decision before computation.
- Kiểm tra khả năng tái lập bằng repeated runs.

Kết quả cần có:
- Một cấu hình chuẩn dùng cho mọi phương pháp.
- Simulation/SLAM/Nav2 ổn định.
- Denominator của `known_fraction` và `coverage` cố định.
- Provenance đủ để biết chính xác code/config/world nào đã chạy.

## 2. Nearest Frontier baseline

- Dùng **MapEx frontier generation** chung cho Nearest/MapEx.
- Nearest score = Euclidean distance tới frontier representative.
- Hospital adaptation: candidate `<1m` vẫn được gửi Nav2; không dùng MapEx 1 m rejection vì pilot_005 chứng minh startup deadlock.
- Chạy closed-loop tối thiểu 5 run, mục tiêu 10 run.
- Chưa dùng LaMa.
- Ghi coverage theo time/distance, total distance/time, frontier goals, success/failure, computation timing, termination.
- Lưu exact decision maps/candidates và periodic/final maps để không phải rerun vì thiếu dữ liệu.
- Báo cáo từng run và mean ± std.

## 3. MapEx closed-loop

Pipeline Hospital:

`Observed map → MapEx frontier candidates → LaMa ensemble → mean + variance → visibility → IG → score/rank → Nav2 → map update → repeat`

- Cùng frontier-generation semantics, below-1m adaptation, Nav2 stack, benchmark clock và logging protocol như Nearest.
- Chạy cùng điều kiện và cùng số run như Nearest.
- Đo Coverage, occupied IoU, TU nếu dùng, distance, time, failed goals, success rate và computation time.
- Trả lời: MapEx thực sự tốt/kém Nearest ở đâu trên Hospital?

## 4. Phân tích theo exploration stage

### 4.1 Trục stage chính: absolute exploration state

Không normalize final state từng run thành 100%.

Ưu tiên `coverage` trên canonical ROI. Nếu dùng `known_fraction` thì fixed canvas phải giống hệt giữa mọi run.

Các ngưỡng stage được chốt một lần sau pilot dựa trên vùng coverage mà các methods có thể so sánh công bằng.

### 4.2 So sánh tại cùng coverage stage

Tại cùng stage so:
- time-to-stage;
- distance-to-stage;
- goals attempted/succeeded/failed;
- success rate;
- computation cost;
- với MapEx: prediction error, uncertainty calibration, visibility error, IG/ranking quality.

Không so “coverage cao hơn” tại chính cùng một coverage stage.

Hiệu quả tổng thể báo riêng bằng:
- Coverage vs time;
- Coverage vs distance;
- Coverage AUC time/distance;
- final coverage dưới fixed common budget;
- total time/distance nếu stopping condition cho phép so trực tiếp.

### 4.3 Resource-budget progress

Có thể phân tích phụ:

```text
time_progress = time_s / fixed_time_budget_s
distance_progress = distance_m / fixed_distance_budget_m
```

Budget phải giống nhau giữa methods.

## 5. Log toàn bộ quyết định MapEx

Mỗi policy decision lưu:
- policy decision ID;
- benchmark time + source map timestamp;
- exact observed map raw + fixed canvas;
- map-frame robot pose;
- full MapEx frontier candidate set + stable candidate IDs;
- G1/G2/G3;
- ensemble mean, variance;
- mỗi candidate: distance, predicted visibility, IG, score, rank;
- selected candidate + Nav2 path/outcome;
- computation timing từng stage nếu khả thi.

Ngay cả `candidate_count=0` cũng phải lưu exact exhausted decision.

Nếu có structural GT, thêm prediction error, GT visibility, GT gain, GT gain/m và GT-best candidate.

## 6. Đánh giá từng tầng

### 6.1 Prediction
So LaMa prediction với GT theo stage: IoU, precision, recall, prediction error.

### 6.2 Uncertainty
Đo quan hệ ensemble variance ↔ prediction error. Tìm low-variance/high-error cells và calibration theo stage.

### 6.3 Visibility
So predicted visibility ↔ GT visibility: visible IoU, false-visible, missed-visible.

### 6.4 Information Gain
So estimated IG ↔ GT gain trên toàn bộ MapEx frontier candidates; dùng Spearman/ranking metrics.

### 6.5 Ranking/scoring
Tách:
- `rho(raw IG, GT gain)`;
- `rho(IG/d, GT gain/m)`.

Nếu raw IG tốt nhưng IG/d kém → nghi scoring/distance. Nếu raw IG đã kém → lỗi upstream.

### 6.6 Candidate-set quality
So ba mức:

`MapEx top-1 → GT-best trong MapEx frontier candidate set → GT-best broader observation pose`

Nếu MapEx top-1 << candidate-set oracle → ranking/scoring. Nếu candidate-set oracle << broader-pose oracle → candidate generation/viewpoint space.

## 7. Oracle / ablation

Phân biệt:
- Offline oracle: ranking/Spearman/headroom trên snapshot.
- Closed-loop oracle: Coverage, distance, time thực tế.

Test chính:
- Oracle uncertainty: variance → GT prediction error.
- Oracle visibility: predicted visibility → GT visibility.
- Oracle ranking: chọn GT-best trong cùng MapEx frontier candidate set.
- Oracle candidate: so candidate-set oracle với broader-pose oracle.

Một bottleneck có bằng chứng mạnh khi offline cải thiện rõ và, nếu khả thi, được xác nhận closed-loop.

## 8. Chọn bottleneck

Chỉ so trực tiếp mức cải thiện của các oracle khi cùng metric. Ưu tiên thành phần có oracle headroom lớn nhất; không chọn đề tài trước bằng chứng.

## 9. Literature

Sau khi biết bottleneck mới tìm nghiên cứu liên quan. Phân biệt:
- đã giải quyết trực tiếp;
- gần liên quan;
- phần còn trống.

Nếu bottleneck đã có lời giải mạnh, đổi hướng sớm.

## 10. Proposed method

Ví dụ tùy bottleneck:
- uncertainty → calibrated/stage-aware/error-aware uncertainty, ensemble diversity;
- visibility → uncertainty-aware raycasting, robust/multi-hypothesis visibility;
- scoring → stage-adaptive score, normalized IG, adaptive IG-distance weighting;
- candidate generation → broader observation poses, frontier + viewpoint sampling.

## 11. Benchmark cuối

Tối thiểu:
- Nearest Frontier;
- MapEx original Hospital adaptation;
- Proposed method.

Nếu cần benchmark sát paper hơn mới thêm UPEN và IG-Hector.

Metric cuối nên có:
- Coverage AUC;
- occupied IoU AUC;
- Topological Understanding;
- total distance/time;
- failed goals/success rate;
- ranking correlation;
- computation time.

## 12. Bốn câu hỏi trước khi chốt đề tài

1. MapEx thất bại ở đâu trên Hospital?
2. Thành phần nào gây failure đó?
3. Có bằng chứng nhân quả từ oracle/ablation không?
4. Đây có phải research gap sau khi đối chiếu literature không?
