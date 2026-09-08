# Old-Map-First Temporal Anchor V2

## Mục tiêu

Thực hiện nguyên tắc độ tin cậy giảm dần theo thời gian ở cả hai tầng của SLAM:

```text
S1 > S2 > S3 > ... > Sn
```

`S1` là pose đầu tiên và vẫn được Ceres giữ constant tuyệt đối. Các scan/constraint sớm hơn có ảnh hưởng lớn hơn scan/constraint muộn hơn. Khi robot quay lại vùng cũ, phần trajectory mới được ưu tiên điều chỉnh trước thay vì kéo mạnh phần map đầu.

Đây là diagnostic SLAM ablation cho New Room, chưa thay đổi protocol chính thức `hospital_v2`.

## Kiến trúc V2

V2 áp dụng cùng một thứ tự độ tin cậy ở hai tầng:

```text
Laser scan mới
    |
    v
Weighted sequential scan matching
recent buffer + nearby trusted history
S1 > S2 > S3 > ...
    |
    v
Pose mới + graph constraints
    |
    v
Temporal Ceres weighting
S1 hard fixed; early local edges > late local edges
    |
    v
Corrected trajectory + map
```

Loop-closure matcher của Karto vẫn giữ nguyên, không dùng temporal weighting. Loop constraint đi vào Ceres ở `1x`, để loop closure vẫn là một nguồn bằng chứng độc lập.

## 1. Weighted sequential scan matching

Implementation:

```text
slam_toolbox/include/slam_toolbox/oldmap_mapper.hpp
slam_toolbox/src/slam_mapper.cpp
```

`OldMapMapper` kế thừa `karto::Mapper`. Khi:

```yaml
oldmap_scan_weighting_enabled: false
```

nó gọi thẳng upstream `karto::Mapper::Process()`, nên các profile khác giữ hành vi cũ.

Khi V2 bật, chỉ bước sequential scan matching trong `Mapper::Process()` được thay. Graph construction, near-chain links, loop closure và phần còn lại của Karto vẫn giữ nguyên.

### Reference set

Mỗi scan mới dùng hai nguồn reference:

```text
1. Recent reference
   - running buffer bình thường
   - scan_buffer_size = 30

2. Trusted historical reference
   - scan đầu được giữ làm trusted history
   - keyframe lịch sử cách nhau khoảng 0.5 m
   - chỉ lấy keyframe trong bán kính 3.0 m quanh pose dự đoán hiện tại
   - tối đa 40 historical keyframes mỗi match
```

Việc giới hạn theo khoảng cách có nghĩa là `S1` không bị ép tham gia trực tiếp vào mọi phép match. Khi robot ở xa `S1`, nó không đóng góp vào local correlation grid. Khi robot quay lại gần vùng `S1`, scan đầu lại trở thành reference lịch sử mạnh nhất.

### Raw temporal confidence

Với scan có state index `i`:

```text
c_raw(i) = c_min + (1 - c_min) * exp(-i / tau)
```

V2 dùng:

```yaml
c_min: 0.25
tau: 70 nodes
```

Do đó:

```text
c_raw(S1) = 1.00
c_raw(S2) < 1.00
c_raw(S3) < c_raw(S2)
...
late scan -> 0.25
```

### Active normalization

Để không làm response của scan matcher giảm quá thấp khi robot đang ở vùng hoàn toàn mới, confidence trong tập reference đang hoạt động được chuẩn hóa:

```text
c_active(i) = c_raw(i) / max(c_raw(active references))
```

Nhờ vậy scan cũ nhất đang có mặt luôn có weight `1.0`, nhưng thứ tự vẫn giữ:

```text
older active reference > newer active reference
```

Nếu `S1` đang ở gần và được đưa vào tập active thì `S1 = 1.0` và mọi scan sau đều nhỏ hơn.

### Weighted correlation grid

Karto gốc đánh dấu mỗi reference hit với cùng mức occupied rồi smear bằng Gaussian. V2 giữ hình dạng Gaussian đó nhưng nhân biên độ kernel với `c_active` của từng reference scan:

```text
weighted_cell = Gaussian(distance) * GridStates_Occupied * c_active
```

Các scan được fusion bằng `max`, giống tinh thần correlation grid gốc. Vì vậy nếu hai reference đưa ra thông tin mâu thuẫn ở vị trí khác nhau, vùng được hỗ trợ bởi scan cũ hơn có correlation strength lớn hơn.

Coarse/fine correlation search, odometry penalty và covariance calculation vẫn dùng `ScanMatcher::CorrelateScan()` của Karto.

## 2. Temporal pose-graph weighting

V2 giữ cơ chế Ceres đã có từ Old-Map-First V1:

```yaml
adaptive_anchor_enabled: true
adaptive_anchor_min_weight: 1.0
adaptive_anchor_max_weight: 5.0
adaptive_anchor_decay_nodes: 70.0
adaptive_anchor_local_edge_max_gap: 5
adaptive_anchor_release_stage1_factor: 1.0
adaptive_anchor_release_stage2_factor: 1.0
```

Local edge dùng:

```text
w(n) = 1 + 4 * exp(-n / 70)
```

với `n` là tuổi của older node tính từ node đầu.

Ví dụ xấp xỉ:

```text
n=0   -> 5.00x
n=20  -> 4.01x
n=50  -> 2.96x
n=100 -> 1.96x
n=150 -> 1.47x
late  -> 1.00x
```

Ceres áp:

```text
sqrt_information *= sqrt(w)
```

nên information/cost multiplier là `w`.

## 3. First scan / first pose

Pose đầu tiên vẫn được Ceres đặt constant ở `x`, `y`, `yaw`.

```text
S1 = hard fixed
S2 = strong but finite
S3 = weaker than S2
...
Sn = most flexible
```

Điều này khác với việc hard-freeze cả chuỗi. Chỉ `S1` là bất biến tuyệt đối; các scan sau vẫn có thể sửa nếu dữ liệu yêu cầu.

## 4. Khi robot quay lại vùng cũ

Ví dụ:

```text
S1 ---- S2 ---- ... ---- S100
 \_________________________/
          revisit
```

Khi `S100` quay lại gần vùng map đầu:

1. local matcher lấy recent scans và nearby historical keyframes;
2. nếu `S1`/các keyframe sớm ở gần, chúng có confidence cao hơn scan mới;
3. pose ban đầu của `S100` được kéo theo cấu trúc lịch sử đáng tin hơn;
4. Karto vẫn kiểm tra loop closure theo cơ chế gốc;
5. loop constraint được thêm ở `1x` nếu được chấp nhận;
6. Ceres solve toàn graph, trong đó early local edges cứng hơn late local edges;
7. correction vì vậy có xu hướng được hấp thụ nhiều hơn ở phần trajectory mới.

## 5. Runtime profile

Launch:

```text
mapex_lab/launch/oldmap_toolbox.launch.py
```

Các tham số V2 chính:

```yaml
scan_buffer_size: 30

oldmap_scan_weighting_enabled: true
oldmap_scan_min_confidence: 0.25
oldmap_scan_decay_nodes: 70.0
oldmap_keep_first_scan: true
oldmap_keyframe_distance: 0.5
oldmap_history_search_radius: 3.0
oldmap_history_max_keyframes: 40

adaptive_anchor_enabled: true
adaptive_anchor_min_weight: 1.0
adaptive_anchor_max_weight: 5.0
adaptive_anchor_decay_nodes: 70.0
adaptive_anchor_local_edge_max_gap: 5
adaptive_anchor_release_stage1_factor: 1.0
adaptive_anchor_release_stage2_factor: 1.0
```

Recorder provenance:

```text
new_room_oldmap_toolbox_v2
```

Không trộn run V1 với V2.

## 6. Những gì V2 không thay

- Không hard-freeze `S2`, `S3`, ...
- Không bắt mọi scan trực tiếp match với `S1`.
- Không temporal-weight Karto loop-closure matcher.
- Không tăng weight loop constraint.
- Không thay frontier selection / Nearest / MapEx policy.
- Không thay Nav2.
- Không thay official `hospital_v2` protocol.

## 7. Rủi ro cần kiểm tra

1. Nếu scan/constraint đầu rất sai, ưu tiên lịch sử có thể bảo vệ sai số đó lâu hơn.
2. Historical radius quá lớn có thể đưa vào các cấu trúc lặp và làm local match mơ hồ.
3. Keyframe quá dày làm tăng chi phí scan matching; quá thưa làm old-map evidence yếu.
4. `max` fusion khiến một scan cũ mạnh có thể chi phối vùng mâu thuẫn; đây là hành vi chủ ý của V2 nhưng phải đánh giá bằng run thực tế.
5. `node_gap<=5` vẫn là heuristic cho local pose-graph edge vì `LinkInfo` không expose edge origin/type qua `ScanSolver` API.
6. Loop closure đúng có thể sửa ít hơn Toolbox nếu early weights quá mạnh; trước tiên giảm `5x` hoặc decay `70` nếu thấy graph quá cứng.

## 8. Runtime validation tối thiểu

Sau khi rebuild `slam_toolbox`:

1. startup phải có:

```text
Old-map sequential matcher: enabled=true, min_conf=0.25, decay=70.0,
keep_first=true, keyframe_dist=0.50, history_radius=3.00, history_max=40
```

2. Ceres phải có:

```text
weight=5.00->1.00, decay=70.0, local_gap<=5, release=1.00->1.00
```

3. xác nhận first pose được set constant;
4. chạy `nf_basic.py` trước, chưa ghi benchmark;
5. quan sát map khi robot quay lại vùng đầu: phần map đầu phải ổn định hơn, còn phần mới có thể co/chỉnh về lịch sử;
6. nếu smoke test ổn mới ghi NF/MapEx run bằng provenance V2.
