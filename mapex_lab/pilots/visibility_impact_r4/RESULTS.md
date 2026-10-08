# R4 Stage A A1 — kết quả development

**Đánh giá hiện tại: INCONCLUSIVE về tần suất/mức mất mát đủ lớn và khả năng khắc phục online.** Đã chạy và audit xong sáu layout hiện có; chưa hoàn thành toàn bộ R4. Code ở nhánh [mapex-visibility-impact-r4-20261008](https://github.com/vah103/turtlebot4_project/tree/mapex-visibility-impact-r4-20261008/mapex_lab/pilots/visibility_impact_r4). Kết quả chi tiết và JSON audit được giữ cùng code. Lượt chạy kết thúc lúc 16:49:38 ngày 08/10/2026 (UTC+07); audit cuối lúc 2026-10-08 17:42:41 (UTC+07).

**10.1 Đã triển khai và thực sự chạy gì?**

- Census 91 quyết định hợp lệ; 75 state có ít nhất một cặp trong tám arm đổi action đầy đủ. Các quyết định không đổi vẫn thuộc mẫu số.
- Tám arm P/GT × native/first-hit sensor × U/uniform; candidate, cost, tie-breaking và đường đi dùng observation. GT chỉ dùng cho chẩn đoán và evaluator.
- SRS không hoàn lại K=6/layout, seed 2026100804, tổng 36 state; danh sách được chọn trước outcome. M ở bảng dưới là union đổi action của cả tám arm, không phải riêng cặp chính.
- Mỗi nhánh chỉ đổi action đầu, sau đó cùng native continuation tới tổng B=150 m từ start. P/U được cập nhật ở mọi quyết định tương lai. Không cutoff 8 m, không đóng băng future U.
- 100 outcome action riêng sau dedup: 36 baseline suffix được replay để kiểm tra parity và 64 suffix khác baseline. Đây là các nhánh từ reference state, không phải 100 episode độc lập. Cả sáu reference và 100 outcome đều đạt 150 m, không va chạm; reference có 21 lần kết thúc action vì measurement mới làm footprint kế tiếp không an toàn.
- C_bar là observed-free coverage AUC/150 m; Q là strict occupied IoU trên cùng ROI fixed valid_space với cùng ensemble reporting. Không dùng predicted coverage làm observed coverage.
- A1 thêm 511 lần suy luận ensemble thật, 380 cache hits; 4.590,37 s suy luận (76,51 phút), 5.467,55 s wall time (91,13 phút). Không tính lượt debug A0/preflight vào các con số này.

**10.2 Kết quả đối chiếu chính**

Thay P bằng GT trong **native renderer**, giữ U và mọi thành phần còn lại, đổi action đầu rồi trở về native continuation. Đây là can thiệp map-input trong scorer, **không phải action oracle tối ưu hay trần của mọi lợi ích sửa visibility**.

| Layout | Building | T/M/k | GT đổi action/T | Ca qua cả hai gate/k | Mean delta C_bar ước lượng (pp) | Khoảng tỷ lệ trong episode |
|---|---|---:|---:|---:|---:|---:|
| kth_50052748 | A0043022 | 15/13/6 | 7/15 | 0/6 | -0.128 | 0.00–40.00% |
| kth_50037764_PLAN1 | A0043040 | 21/16/6 | 12/21 | 0/6 | -2.095 | 0.00–42.86% |
| kth_50037765_PLAN3 | A0043040 | 13/12/6 | 8/13 | 0/6 | -0.684 | 0.00–46.15% |
| kth_50010535_PLAN1 | A0043034 | 9/7/6 | 3/9 | 0/6 | 0.328 | 0.00–11.11% |
| kth_50010536_PLAN3 | A0043034 | 18/15/6 | 10/18 | 0/6 | -0.860 | 0.00–44.44% |
| kth_50015848 | A0043035 | 15/12/6 | 11/15 | 0/6 | -1.104 | 0.00–40.00% |

Gate của một state: delta C_bar >= 0,02 và delta Q >= -0,005. Trong sample, cặp chính có 0/36 ca qua gate; thực tế không ca nào tăng C_bar >= 0,02. Đổi action xuất hiện chính xác ở 51/91 quyết định (56,04%), nhưng đổi action không tự chứng minh thiệt hại nhiệm vụ.

Primary tổng hợp: lấy mean các floor/start cố định trong mỗi building, rồi bốn building ngang trọng số. Tỷ lệ ước lượng **0.00%**, khoảng sampling cho cohort đã quan sát **0.00–38.07%**. Mean delta C_bar/decision ước lượng **-0.722 pp**; khoảng bounded-mean đã khóa là **[-82.868; 82.868] pp**, quá rộng để kết luận mức ảnh hưởng nhỏ. Mean này là effect của can thiệp một quyết định, **không phải tổng regret của episode hay hiệu quả policy sửa liên tục**.

**10.3 Các đối chứng đã khóa**

| Đối chiếu | Ca qua gate trong sample | Tỷ lệ ước lượng theo building | Khoảng tỷ lệ cohort | Mean delta C_bar/decision (pp) |
|---|---:|---:|---:|---:|
| Native: P → GT, giữ U (chính) | 0/36 | 0.00% | 0.00–38.07% | -0.722 |
| First-hit: P → GT, giữ U | 3/36 | 10.28% | 5.00–44.74% | -0.311 |
| Native: P → GT, uniform | 2/36 | 6.94% | 3.33–43.07% | 0.010 |
| First-hit: P → GT, uniform | 4/36 | 11.87% | 5.60–45.93% | 0.505 |
| Native → first-hit, giữ P/U | 0/36 | 0.00% | 0.00–38.07% | -0.628 |

Khoảng hypergeometric và bounded-mean dùng alpha=0,05/(6×5). Có coverage đồng thời 95% trong từng family tỷ lệ hoặc mean-effect riêng; không nhận joint 95% cho hợp cả hai family. Các mean interval của năm đối chiếu đều rất rộng, [-82.868; 82.868] pp. Các bounds chỉ phản ánh sampling trên cohort cố định; **không phải uncertainty khi suy rộng sang building mới**.

Những ca dương ở sensitivity controls cho thấy một số thay đổi visibility có thể giúp dưới scorer/weighting tương ứng. Chúng được tính so với arm P cùng renderer/weighting, không mặc định là cải thiện tương ứng so với native_P_U. Ví dụ đã replay đúng: tại kth_50037764_PLAN1/state 3, sensor_GT_uniform hơn sensor_P_uniform 7,481 pp C_bar và 37,777 pp Q, nhưng chỉ hơn native reference 1,725 pp C_bar. Không dùng ca này để gọi bottleneck của native đã được xác nhận.

**10.4 Kiểm tra tính đúng và lượt bị loại**

Audit cuối pass trên cả sáu reference: sealed source/config/data hashes; provenance prediction cache; bảo toàn ô đã biết; chọn sample đúng seed; physical sensor/state replay; C_bar và Q tính độc lập; branch context; same-action zeros và estimators tính lại. Các branch cũng khớp bounds của phần coverage quá khứ chung và cùng tổng budget.

Thêm ba actual suffix replay bằng cache-only cho một ca giảm lớn và một sensitivity contrast dương: final observation hash, C_bar, Q, distance, collisions và terminal đều khớp; không được phép suy luận mới. Đây là kiểm tra integrity được chọn sau outcome, không phải sample bổ sung hay replication độc lập.

A0 dùng controller tiếp tục đường cũ dù scan mới đã đánh dấu footprint tiếp theo không an toàn. Audit bốn reference đầu đều phát hiện collision dạng này. **Toàn bộ A0 là debug-only**, không dùng cho kết luận nghiên cứu. A1 có seal/output riêng và regression fixture; action bị chặn bởi measurement kết thúc rồi cùng native controller chọn lại. Không dùng GT để tránh chướng ngại. Giữ nguyên layout/start, B, K, seed, tám arm, scorer, reporter, ROI và gate; không tái sử dụng A0 sample hoặc branch outcome. Chỉ chia sẻ prediction cache có cùng provenance và observed-map hash.

Bộ nạp MapEx gốc cũng pool 2×2 với occupied priority và valid_space any; tỷ lệ 0,1 m/ô khớp original loader. Nhiệm vụ này vẫn là adapter 2D xác định, pose/sensor không nhiễu và biên world không pad; không phải reproduction đầy đủ Nav2/Gazebo hay kết quả trên robot.

**10.5 Chưa hoàn thành những gì trong R4?**

| Yêu cầu | Trạng thái sau pilot |
|---|---|
| Census, sampling có xác suất và paired suffix toàn budget | Đã triển khai, chạy và audit trên sáu layout hiện có. |
| Sáu building development được xác minh | **Chưa đạt:** sáu layout thuộc bốn building. |
| Audit training overlap của predictor G | Bài gốc V-A báo split 80:20 theo building; chưa có train-building list của exact checkpoints để kiểm chứng độc lập. |
| Pathwise labels/scorer/control của f/g | Chưa freeze/preflight đầy đủ; Stage A truy vấn cùng frontier endpoint để giữ native scorer. Physical labels phải giữ scan timing và measured-path abort, không render từ pose ảo không an toàn. |
| f học Brier visibility và g học direct gain với cùng input/compute | Chưa huấn luyện hay test. |
| Edits/cơ chế, placebo và weight khi tổng hợp | Chưa chạy. |
| Bốn policy full-episode, ba start/building, sáu simultaneous C/Q contrasts | Chưa chạy. |
| Power từ variance paired full-policy episodes | Chưa có dữ liệu phù hợp. |
| Xác nhận tần suất đủ thường xuyên với độ chính xác đã chọn | Chưa đạt; pilot không khóa một minimum-frequency gate/target interval width, interval còn rộng. |
| Contribution/novelty của phương pháp cuối | Chưa xác nhận. |

Building IDs được join chính xác từ XML KTH gốc: A0043022, A0043040, A0043034, A0043035. Toàn bộ 14 raw floor đi kèm repo cũng chỉ thuộc bốn building này, nên thêm floor từ pool hiện có không tăng số building độc lập. [Nguồn XML](https://bsg.lericson.se/kth_floorplans_xml.zip), [trang tác giả liên kết dữ liệu](https://lericson.se/floorgent/); archive SHA256 và entry hashes ở building_metadata.json. Tra cứu 64 commit hiện có của repo gốc cũng chưa tìm lại được split manifest; điều này không phải evidence tác giả đã train trên test.

**10.6 Tự đánh giá thẳng câu hỏi “đã đo được mức ảnh hưởng chúng ta muốn chưa?”**

**Đo được effect của các can thiệp cụ thể trong reference benchmark này: có. Đã đủ chính xác, đại diện và xác nhận recoverable online impact để quyết định contribution mạnh: chưa.** Code chạy đúng không bù được sample nhỏ, chỉ bốn building và thiếu online confirmation. Không tìm thấy ca qua gate ở cặp chính là kết quả thực, nhưng upper bound 38,07% và mean interval rộng không cho phép kết luận “không đáng kể”.

Kết quả hiện tại không cung cấp bằng chứng dương mạnh cho việc ưu tiên fine-tune G theo visibility. Nó cũng không chứng minh MapEx hết hướng nghiên cứu, hay phủ định mọi learner/can thiệp. Việc huấn luyện f/g, đánh giá cơ chế và xác nhận toàn policy vẫn là phần còn lại của thiết kế; không ghi các bước đó đã hoàn thành bằng oracle hoặc một trajectory census.
