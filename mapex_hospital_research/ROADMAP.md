# Research Roadmap

## Mục tiêu chung

Tìm chính xác thành phần nào trong pipeline MapEx là bottleneck khi chuyển sang Hospital, sau đó mới chọn bài toán nghiên cứu và đề xuất phương pháp cải tiến.

## Nguyên tắc

Không giả định trước ranking, uncertainty, visibility hay WFD là vấn đề. Xác nhận failure trong closed-loop trước, sau đó dùng phân tích từng tầng và oracle/ablation để tìm nguyên nhân.

## 1. Chuẩn hóa Hospital

Việc cần làm:
- Chạy ổn định simulation + SLAM + Nav2.
- Cố định world, spawn, sensor, resolution, SLAM/Nav2 params, timeout, stopping condition, seed nếu có.
- Chốt một fixed logging canvas và một canonical evaluation ROI/mask dùng chung cho mọi run/phương pháp.
- Kiểm tra khả năng tái lập.

Kết quả cần có:
- Một cấu hình chuẩn dùng cho mọi phương pháp.
- Simulation/SLAM/Nav2 ổn định.
- Denominator của `known_fraction` và `coverage` được cố định trước khi chạy benchmark.

## 2. Nearest Frontier baseline

- Chạy closed-loop tối thiểu 5 run, mục tiêu 10 run.
- Chưa dùng LaMa.
- Ghi coverage theo time/distance, total distance/time, frontier goals, success/failure, thời điểm kết thúc.
- Báo cáo từng run và mean ± std.

## 3. MapEx closed-loop

Pipeline:

`Observed map → WFD → LaMa ensemble → mean + variance → visibility → IG → score → frontier → Nav2 → map update → repeat`

- Chạy cùng điều kiện và cùng số run như Nearest.
- Đo Coverage, occupied IoU, TU nếu dùng, distance, time, failed goals, success rate.
- Trả lời: MapEx thực sự tốt/kém Nearest ở đâu trên Hospital?

## 4. Phân tích theo exploration stage

### 4.1 Trục stage chính: absolute exploration state

Không chuẩn hóa final known fraction của từng run thành 100%.

Stage chính phải dựa trên một đại lượng tuyệt đối có cùng denominator cho mọi run, ưu tiên `coverage` trên canonical evaluation ROI. Nếu dùng `known_fraction` thì fixed logging canvas phải giống hệt giữa mọi run.

Ví dụ, nếu các run thực sự đi qua đủ các mốc, có thể chia:
- 0–20%: very early
- 20–40%: early
- 40–60%: mid
- 60–80%: late
- 80–100%: very late

Các ngưỡng thực tế được chốt một lần sau pilot dựa trên vùng giá trị mà các phương pháp có thể so sánh công bằng. Nếu một run kết thúc trước một stage thì run đó không có sample ở stage đó; không kéo giãn final state của run thành 100%.

### 4.2 Trục phụ: normalized resource-budget progress

Có thể phân tích thêm theo resource budget để biết failure xuất hiện sớm/muộn theo chi phí đã dùng:
- `time_progress = time_s / fixed_time_budget_s`
- `distance_progress = distance_m / fixed_distance_budget_m`

Time/distance budget phải cố định và giống nhau giữa các phương pháp. Đây là phân tích phụ, không thay thế absolute exploration state.

### 4.3 Định nghĩa dùng trong workspace

- `known_fraction`: tỷ lệ cell đã biết (`occupancy != unknown`) trên fixed logging canvas. Đây chủ yếu là progress/debugging proxy và chỉ so được khi canvas/resolution/origin cố định.
- `coverage`: tỷ lệ cell trong canonical evaluation ROI đã được quan sát/biết ở thời điểm hiện tại. Đây là metric exploration chính; ROI và denominator phải cố định giữa mọi run.
- `occupied IoU`: metric riêng để đánh giá correctness của occupied mapping; không dùng coverage để suy ra map correctness.

Kết quả: biết failure xuất hiện ở absolute exploration state nào, có lặp lại giữa nhiều run hay không, và có phụ thuộc resource budget hay không.

## 5. Log toàn bộ quyết định MapEx

Mỗi decision lưu:
- snapshot/decision ID
- known fraction, robot pose
- observed map
- G1/G2/G3
- mean, variance
- WFD candidates
- mỗi frontier: distance, predicted visibility, IG, score, ranking
- frontier được chọn

Nếu có structural GT, thêm prediction error, GT visibility, GT gain, GT gain/m, GT-best frontier.

## 6. Đánh giá từng tầng

### 6.1 Prediction
So LaMa prediction với GT theo stage: IoU, precision, recall, prediction error.

### 6.2 Uncertainty
Đo quan hệ ensemble variance ↔ prediction error. Tìm low-variance/high-error cells và kiểm tra calibration theo stage.

### 6.3 Visibility
So predicted visibility ↔ GT visibility; đo visible IoU, false-visible, missed-visible.

### 6.4 Information Gain
So estimated IG ↔ GT gain; dùng Spearman trên toàn bộ frontier candidates.

### 6.5 Ranking/scoring
Tách:
- `rho(raw IG, GT gain)`
- `rho(IG/d, GT gain/m)`

Nếu raw IG tốt nhưng IG/d kém → nghi scoring/distance. Nếu raw IG đã kém → lỗi upstream.

### 6.6 WFD candidate set
So:
`MapEx top-1 → GT-best WFD → GT-best broader observation pose`

Nếu MapEx << WFD oracle → ranking/scoring. Nếu WFD oracle << broader oracle → candidate generation.

## 7. Oracle / ablation

Phân biệt hai mức:
- Offline oracle: ranking/Spearman.
- Closed-loop oracle: Coverage, distance, time thực tế.

Test chính:
- Oracle uncertainty: variance → GT prediction error.
- Oracle visibility: predicted visibility → GT visibility.
- Oracle ranking: chọn GT-best trong WFD candidates.
- Oracle candidate: so WFD oracle với broader-pose oracle.

Một bottleneck có bằng chứng mạnh khi offline cải thiện rõ và, nếu khả thi, được xác nhận thêm bằng closed-loop.

## 8. Chọn bottleneck

Chỉ so trực tiếp mức cải thiện của các oracle khi cùng metric, ví dụ cùng Coverage AUC hoặc cùng Spearman.

Ưu tiên thành phần có oracle headroom lớn nhất; không chọn đề tài trước khi có bằng chứng.

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
- WFD → broader observation poses, frontier + viewpoint sampling.

## 11. Benchmark cuối

Tối thiểu:
- Nearest Frontier
- MapEx original
- Proposed method

Sau đó nếu cần benchmark sát paper hơn mới thêm UPEN và IG-Hector.

Metric cuối nên có:
- Coverage AUC
- occupied IoU AUC
- Topological Understanding
- total distance/time
- failed goals/success rate
- ranking correlation
- computation time

## 12. Bốn câu hỏi trước khi chốt đề tài

1. MapEx thất bại ở đâu trên Hospital?
2. Thành phần nào gây failure đó?
3. Có bằng chứng nhân quả từ oracle/ablation không?
4. Đây có phải research gap sau khi đối chiếu literature không?
