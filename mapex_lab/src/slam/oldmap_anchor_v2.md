# Old-Map-First Temporal Anchor V2

## Mục tiêu

Thực hiện nguyên tắc độ tin cậy giảm dần theo thời gian ở cả frontend local matching và backend pose graph:

```text
S1 > S2 > S3 > ... > Sn
```

`S1` là pose đầu tiên và vẫn được Ceres giữ constant tuyệt đối. Các scan/constraint sớm hơn có ảnh hưởng lớn hơn scan/constraint muộn hơn. Khi robot quay lại vùng cũ, phần trajectory mới được ưu tiên điều chỉnh trước thay vì kéo mạnh phần map đầu.

Đây là diagnostic SLAM ablation cho New Room, chưa thay đổi protocol chính thức `hospital_v2`.

## Kiến trúc V2 hiện tại

```text
Laser scan mới Sn
    |
    v
Weighted initial local scan matching
recent buffer + nearby trusted history
older reference > newer reference
    |
    v
Pose ban đầu của Sn
    |
    v
OldMap AddEdges (topology giống Karto)
    |
    +--> previous-scan edge
    |
    +--> running-chain link
    |
    `--> weighted near-chain local matching
         older chain scan > newer chain scan
    |
    v
Temporal Ceres weighting
S1 hard fixed; early local edges > late local edges
    |
    v
Karto loop closure (upstream/unweighted)
loop edge = 1x
    |
    v
Corrected trajectory + map
```

Điểm quan trọng: **cả hai phép local scan matching có thể thay đổi pose của scan mới đều dùng cùng temporal confidence**:

1. initial sequential/local match trong `OldMapMapper::Process()`;
2. near-chain local match trong graph construction.

Loop-closure matcher của Karto vẫn giữ nguyên, không dùng temporal weighting. Loop constraint đi vào Ceres ở `1x`, để loop closure vẫn là một nguồn bằng chứng độc lập.

## 1. OldMapMapper và phạm vi thay đổi

Implementation chính:

```text
slam_toolbox/include/slam_toolbox/oldmap_mapper.hpp
slam_toolbox/src/slam_mapper.cpp
```

`OldMapMapper` kế thừa `karto::Mapper`. Khi:

```yaml
oldmap_scan_weighting_enabled: false
```

nó gọi thẳng upstream `karto::Mapper::Process()`, nên các profile khác giữ hành vi cũ.

Khi V2 bật, `OldMapMapper` sở hữu đường xử lý mapping riêng và mirror topology của `MapperGraph::AddEdges()` để có thể thay đúng **near-chain MatchScan** bằng weighted matcher. Previous-scan edge, running-chain linking, near-chain discovery, covariance-weighted pose fusion và các threshold vẫn theo logic Karto. Loop closure tiếp tục gọi `MapperGraph::TryCloseLoop()` gốc.

## 2. Initial weighted local scan matching

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

Các scan được fusion bằng `max`. Vì vậy nếu hai reference đưa ra thông tin mâu thuẫn ở vị trí khác nhau, vùng được hỗ trợ bởi scan cũ hơn có correlation strength lớn hơn.

Coarse/fine correlation search và covariance calculation vẫn dùng `ScanMatcher::CorrelateScan()` của Karto. Initial match giữ `do_penalize=true`, giống ordinary sequential matching.

## 3. Weighted near-chain local matching

Đây là thay đổi hoàn thiện V2 so với bản đầu tiên.

Karto gốc trong `LinkNearChains()` làm:

```text
near chain
    |
    v
unweighted MatchScan(..., doPenalize=false)
    |
    v
mean + covariance
    |
    v
LinkChainToScan
```

Old-Map V2 hiện làm:

```text
near chain
    |
    v
c_raw(scan) theo state index
    |
    v
normalize trong chính chain
    |
    v
WeightedMatchScan(..., do_penalize=false)
    |
    v
mean + covariance
    |
    v
LinkChainToScan
```

Nghĩa là trong một near chain:

```text
older scan > newer scan
```

và near-chain match không còn là một nhánh local có khả năng ghi đè pose theo cơ chế equal-confidence.

`OldMapMapper` mirror các bước graph construction của Karto:

- link scan mới với previous scan;
- link với closest scan trong running chain;
- tìm near chains theo cùng graph/distance logic;
- chỉ xét chain đủ `loop_match_minimum_chain_size`;
- weighted match chain;
- kiểm tra `link_match_minimum_response_fine`;
- link tới closest scan của chain nếu trong `link_scan_maximum_distance`;
- fuse các pose mean bằng covariance như Karto.

Do đó topology/threshold được giữ, chỉ thay **cách tính near-chain local match**.

## 4. Temporal pose-graph weighting

V2 giữ cơ chế Ceres của Old-Map-First:

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

Edge classification hiện tại:

```text
node_gap <= 5      -> temporal local edge, w(n)
6 <= gap < 30      -> ordinary 1x edge
gap >= 30          -> loop-like candidate, 1x
```

Đây vẫn là heuristic vì `LinkInfo` không expose explicit edge origin/type qua `ScanSolver` API.

## 5. First scan / first pose

Pose đầu tiên vẫn được Ceres đặt constant ở `x`, `y`, `yaw`.

```text
S1 = hard fixed
S2 = strong but finite
S3 = weaker than S2
...
Sn = most flexible
```

Chỉ `S1` là bất biến tuyệt đối; các scan sau vẫn có thể sửa nếu dữ liệu yêu cầu.

## 6. Loop closure

Loop closure cố ý **không temporal-weight**.

```text
candidate old chain
    |
    v
Karto coarse loop match (upstream)
    |
    v
Karto fine loop match (upstream)
    |
    v
accepted loop edge = 1x
    |
    v
whole-graph Ceres solve
```

Như vậy frontend local mapping ưu tiên lịch sử, nhưng loop detector không bị ép phải đồng ý với map cũ. Nếu một loop thật được phát hiện, nó tạo evidence độc lập; Ceres sau đó phân bố correction trên graph có `S1` fixed và early local edges cứng hơn late local edges.

Adaptive release bị vô hiệu hóa trong profile này:

```text
release_stage1_factor = 1.0
release_stage2_factor = 1.0
```

nên loop evidence không hạ temporal local weights.

## 7. Khi robot quay lại vùng cũ

Ví dụ:

```text
S1 ---- S2 ---- ... ---- S100
 \_________________________/
          revisit
```

Khi `S100` quay lại gần vùng map đầu:

1. initial matcher lấy recent scans và nearby historical keyframes;
2. nếu `S1`/các keyframe sớm ở gần, chúng có confidence cao hơn scan mới;
3. pose ban đầu của `S100` được kéo theo cấu trúc lịch sử đáng tin hơn;
4. graph construction kiểm tra near chains; nếu có, near-chain match cũng dùng old>new confidence;
5. Karto vẫn kiểm tra loop closure theo cơ chế gốc;
6. loop constraint được thêm ở `1x` nếu được chấp nhận;
7. Ceres solve toàn graph, trong đó early local edges cứng hơn late local edges;
8. correction vì vậy có xu hướng được hấp thụ nhiều hơn ở phần trajectory mới.

## 8. Runtime profile

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

`oldmap_mapper.hpp` nằm trong provenance hashes, nên run trước/sau thay đổi near-chain này vẫn phân biệt được bằng exact source hash. Không trộn các run có implementation hash khác nhau trong cùng một batch chính thức.

## 9. Những gì V2 không thay

- Không hard-freeze `S2`, `S3`, ...
- Không bắt mọi scan trực tiếp match với `S1`.
- Không temporal-weight Karto loop-closure matcher.
- Không tăng weight loop constraint.
- Không thay frontier selection / Nearest / MapEx policy.
- Không thay Nav2.
- Không thay official `hospital_v2` protocol.

## 10. Rủi ro cần kiểm tra

1. Nếu scan/constraint đầu rất sai, ưu tiên lịch sử có thể bảo vệ sai số đó lâu hơn.
2. Historical radius quá lớn có thể đưa vào các cấu trúc lặp và làm local match mơ hồ.
3. Keyframe quá dày làm tăng chi phí scan matching; quá thưa làm old-map evidence yếu.
4. `max` fusion khiến một scan cũ mạnh có thể chi phối vùng mâu thuẫn; đây là hành vi chủ ý nhưng phải đánh giá bằng run thực tế.
5. Weighted near-chain match có thể làm local graph cứng hơn trước; cần theo dõi response, covariance và map continuity.
6. `node_gap<=5` vẫn là heuristic cho local pose-graph edge.
7. Loop closure đúng có thể sửa ít hơn Toolbox nếu early weights quá mạnh; trước tiên giảm `5x` hoặc decay `70` nếu thấy graph quá cứng.

## 11. Runtime validation tối thiểu

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
5. xác nhận không có assertion/error trong custom AddEdges / weighted near-chain matching;
6. quan sát map khi robot quay lại vùng đầu: phần map đầu phải ổn định hơn, còn phần mới có thể co/chỉnh về lịch sử;
7. nếu smoke test ổn mới ghi NF/MapEx run bằng provenance V2.