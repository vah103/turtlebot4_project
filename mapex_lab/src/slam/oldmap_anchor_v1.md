# Old-Map-First Temporal Anchor V1

## Mục tiêu

Giữ phần map được tạo sớm ổn định hơn phần map mới, với scan/node đầu tiên là mốc tuyệt đối. Khi loop closure xuất hiện, correction nên được hấp thụ chủ yếu bởi phần trajectory mới hơn thay vì kéo mạnh phần map đầu.

Đây là một diagnostic SLAM ablation cho New Room, chưa phải thay đổi protocol `hospital_v2`.

## Ý tưởng chính

Pose đầu tiên được Ceres giữ constant như upstream SLAM Toolbox. Các local pose-graph constraints được tăng thông tin theo tuổi trajectory:

```text
S1 = hard fixed

S1 ---- S2 ---- S3 ---- S4 ---- ... ---- Sn
      mạnh       mạnh hơn mới dần        gần 1x
```

Không ép scan mới trực tiếp match với S1 nếu hai scan không có vùng quan sát chung. Độ tin cậy của S1 truyền dần qua các local constraints trong pose graph.

## Runtime profile

Launch:

```text
mapex_lab/launch/oldmap_toolbox.launch.py
```

Các tham số chính:

```yaml
scan_buffer_size: 30
adaptive_anchor_enabled: true
adaptive_anchor_min_weight: 1.0
adaptive_anchor_max_weight: 5.0
adaptive_anchor_decay_nodes: 70.0
adaptive_anchor_local_edge_max_gap: 5
adaptive_anchor_loop_min_node_gap: 30
adaptive_anchor_release_stage1_factor: 1.0
adaptive_anchor_release_stage2_factor: 1.0
```

Loop-closure thresholds, scan cadence, map resolution và Nav2 giữ giống profile `new_toolbox` hiện tại.

## Temporal weight

Với local edge có older node index `n` tính từ node đầu:

```text
w(n) = 1 + 4 * exp(-n / 70)
```

Ví dụ xấp xỉ:

```text
n = 0    -> 5.00x
n = 20   -> 4.01x
n = 50   -> 2.96x
n = 100  -> 1.96x
n = 150  -> 1.47x
n -> inf -> 1.00x
```

Ceres dùng đúng quy tắc information weighting hiện có:

```text
sqrt_information *= sqrt(w)
```

nên cost/information multiplier là `w`, không phải `w^2`.

## Local edge classification

V1 coi edge có:

```text
node_gap <= 5
```

là local temporal edge. Điều này mở rộng so với Adaptive V1 cũ chỉ boost `node_gap <= 1`, để các near-chain links cục bộ cũng nhận được thứ tự độ tin cậy theo thời gian.

Các edge có gap lớn, gồm loop-like constraints, không được tăng trọng số và giữ ở `1x`.

## First-node anchor

Node đầu tiên vẫn được upstream Ceres đặt constant ở cả x, y và yaw. Không có node lịch sử nào khác bị hard-freeze.

Điều này tạo thứ tự:

```text
S1 = bất biến
S2 = strong-but-finite
S3 = strong-but-finite nhưng yếu hơn S2
...
Sn = linh hoạt nhất
```

## Loop closure

Karto loop closure vẫn hoạt động bình thường và loop constraints vẫn đi vào whole-graph Ceres solve ở `1x`.

Trong V1 này, adaptive release bị vô hiệu hóa bằng:

```text
release_stage1_factor = 1.0
release_stage2_factor = 1.0
```

Do đó loop evidence không thể hạ temporal weight của phần map cũ. Correction được phân bố bởi Ceres dựa trên graph, trong đó local edges đầu trajectory có cost cao hơn khi bị phá vỡ, nên phần mới có xu hướng chịu correction nhiều hơn.

## Những gì V1 không làm

- Không hard-freeze S2, S3, ...
- Không trực tiếp match mọi scan với S1.
- Không tăng trọng số loop constraint.
- Không thay Karto scan matcher.
- Không thay loop-closure detector.
- Không thay Nav2 hoặc frontier policy.
- Không thay official `hospital_v2` protocol.

## Rủi ro cần kiểm tra

1. Nếu một local constraint rất sớm đã sai, weight lớn có thể giữ sai số đó lâu hơn.
2. `node_gap <= 5` là heuristic cho local edge vì `LinkInfo` không expose edge origin/type qua `ScanSolver` API.
3. Nếu early weights quá mạnh, loop closure đúng có thể không sửa đủ phần trajectory giữa.
4. `scan_buffer_size=30` tăng local context nhưng cũng có thể tăng ambiguity trong môi trường lặp.

## Runtime validation tối thiểu

- Xác nhận startup log cho thấy `weight=5.00->1.00`, `decay=70`, `local_gap<=5`, `release=1.00->1.00`.
- Xác nhận first node vẫn được set constant.
- Xác nhận local edge gap 2..5 có log temporal weight >1 ở giai đoạn đầu.
- Xác nhận long-gap edge vẫn weight 1x.
- Khi quay lại vùng cũ, quan sát phần map đầu có ổn định hơn `new_toolbox` hay không.
- Nếu loop đúng nhưng phần mới không thể co về map cũ, giảm `max_weight` hoặc `decay_nodes` trước khi thêm cơ chế release mới.
