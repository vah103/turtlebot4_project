# Active verification — pilot 2D

Đây là pilot phát triển để kiểm tra một ý tưởng nghiên cứu, chưa phải thuật toán
STOP đã được xác nhận và chưa phải kết quả độc lập QA ACCEPT của Company Hub.
USER cho phép thực hiện trực tiếp trong phiên 2026-10-07 và yêu cầu không dùng
Gazebo. Pilot này là phạm vi mới, không sửa MX067 hay các kết quả D1 lịch sử.

## Bài toán và cơ chế đang thử

Từ bản đồ quan sát và ba dự đoán LaMa, tạo các giả thuyết mở/đóng cục bộ tại
những cấu trúc có căn cứ hình học: lối hẹp được dự đoán hoặc vật cản mỏng có
vùng trống ở hai phía. Chỉ vùng chưa quan sát được thay đổi trong giả thuyết.

Tính diện tích thay đổi trong thành phần vùng có thể tiếp cận nếu giả thuyết
được đảo. Chọn vị trí quan sát theo:

`diện tích thay đổi × phần cấu trúc có thể quan sát / độ dài đường đi`

Đây là **proxy độ nhạy cấu trúc**, không phải xác suất dự đoán sai, giá trị kỳ
vọng đã được hiệu chỉnh, hay chứng nhận bản đồ an toàn. Nếu không có giả thuyết
có ích, phương pháp quay lại chọn frontier theo MapEx; không tự suy ra STOP.

## Protocol được chốt trước kết quả

- `protocol.json`: ba layout (New Room, KTH 50052751, KTH 50052752).
- Hai trạng thái tại quãng đường 5 m và 15 m của cùng policy nearest-frontier.
  Chọn theo quãng đường, không chọn theo lỗi LaMa hoặc kết quả phương pháp.
- Mỗi trạng thái được rẽ thành ba nhánh có ngân sách thêm tối đa 8 m.
- Mô hình dự đoán là ensemble LaMa thật gồm ba checkpoint hiện có, chạy CPU;
  không dùng dự đoán tự tạo cho số liệu nghiên cứu.
- World lưới 0.10 m/cell; pose lý tưởng; sensor không nhiễu, 360°, 2.500 tia,
  tầm 20 m. Tia dừng ở vật cản đầu tiên; lấy mẫu mỗi nửa cell. Quan sát sau
  mỗi 0.30 m và ở cuối hành động/ngân sách.
- Robot bán kính 0.15 m, đi từng cell 4-neighbour. Shortest-path BFS là A*
  với heuristic bằng 0. Chỉ tâm cell đã quan sát trống được đưa vào planner;
  vật cản đã đo được inflate. World kiểm tra footprint thật lúc thực thi,
  báo va chạm thay vì che lỗi hoặc cho robot xuyên tường.

Ba phương pháp:

1. **MapEx 2D adapter:** frontier generation, `IG / Euclidean distance`, luật
   1 m/fallback; tái dùng chính hàm visibility/IG hiện có bằng AST allowlist,
   không import ROS. Giới hạn canvas và planner 2D được ghi rõ; không nhận là
   byte-for-byte reproduction của simulator MapEx gốc hay baseline Gazebo.
2. **Uncertainty:** cùng tập vị trí quan sát rộng với phương pháp cấu trúc,
   chọn `IG / shortest-path distance`. Đây là đối chứng bỏ điểm consequence;
   tập vị trí vẫn có những điểm được tạo từ giả thuyết cấu trúc chung.
3. **Structural:** chọn kiểm chứng theo consequence/visibility/path-cost;
   nếu không có giả thuyết khả dụng, fallback MapEx.

## Tách ground truth

`PolicyInput` chỉ chứa observed/mean/variance/ba dự đoán/pose/resolution/budget.
World giữ geometry thật để sinh phép đo và kiểm tra va chạm. Evaluator dùng
ground truth sau quyết định; domain và metric không quay lại policy. Không
chọn checkpoint, gate hoặc vị trí dựa vào ground truth.

Assets là raster nhỏ từ geometry New Room đã có và hai floorplan KTH trong
MapEx. `assets/sources.json` giữ nguồn, hash và preprocessing. Pose xuất phát
KTH là điểm clearance lớn nhất trong thành phần trống lớn nhất, do simulator
khởi tạo; agent không nhận mask/geometry để ra quyết định. KTH block-reduce 2×2
giữ vật cản theo cách bảo thủ; không thêm 500-cell padding của simulator gốc.

Snapshot SLAM lịch sử chỉ được audit để kiểm tra độ phù hợp làm input vật lý.
Không ghép occupancy SLAM có sai khác với structural GT vào ideal sensor
world, không sửa log cũ, không thay benchmark/metric lịch sử.

## Chạy lại

Yêu cầu Python 3.11, NumPy, SciPy, Matplotlib, Pillow, Shapely. LaMa dùng môi
trường riêng đã có các dependency và checkpoint của MapEx. Không cần ROS.

```bash
cd mapex_lab/pilots
python -m unittest active_verification_2d.test_core -v
OMP_NUM_THREADS=4 MKL_NUM_THREADS=4 OPENBLAS_NUM_THREADS=4 \
python -m active_verification_2d.run \
  --mapex-root "$MAPEX_ROOT" \
  --worker-python "$MAPEX_LAMA_PYTHON" \
  --output active_verification_2d/results/pilot_v1
python -m active_verification_2d.validate
python -m active_verification_2d.render
```

Trên DELL, MapEx root hiện là `/home/dell/MapEx`, worker Python là
`/home/dell/miniforge3/envs/lama/bin/python`. Cache prediction có hash model,
preprocessing và observed map; có thể tái dùng giữa các nhánh. Không dùng
cache prediction lịch sử cho observed map mới nếu hash không khớp.

## Đọc kết quả

- `results/pilot_v1/summary.json`: so sánh paired và layout macro.
- `branches.csv`, `paired.csv`, `initial_diagnostics.csv`: số liệu đầy đủ.
- `provenance.json`: hash code, protocol, assets, checkpoints.
- `validation.json`: tự kiểm tra artifact và recompute endpoint, không thay QA độc lập.
- `comparison.png`, hình từng case, `pilot_demo.gif`, `index.html`: minh họa.
- `raw/`: arrays, actions, đường đi và metric theo hành động; giữ local để repo nhẹ.
- `cache/`: inference cache; giữ local.

Endpoint chính là mismatch diện tích reachable giữa bản đồ hoàn thiện và GT,
có xét footprint, trên miền evaluation cố định. Report thêm missed reachable
free, false reachable, retention và macro IoU. Seed topology được cố định tại
pose warm-start cho mọi nhánh. Miền đánh giá gồm domain và lớp vật cản sát
domain; không normalize từng final map thành 100%.

Đây là metric connectivity/area proxy, chưa phải đánh giá mọi đường đi hoặc
chứng nhận downstream navigation. Nhánh không dùng hết budget được giữ lại
với lý do và tách khỏi so sánh equal-budget; không gọi đó là tiết kiệm do STOP.

Hai warm states cùng layout không phải hai môi trường độc lập. Mọi con số
trong pilot là development evidence; không tính kiểm định suy luận từ n=6,
không chọn threshold STOP và không khẳng định tính mới/chất lượng đồ án đã
được chứng minh. Muốn đi tiếp phải có đối chứng thêm, QA độc lập và layout mới.

Cache LaMa được chia sẻ để pilot chạy nhanh. Thời gian wall-clock/inference
theo nhánh bị ảnh hưởng bởi thứ tự chạy và cache hit; không dùng chúng để
khẳng định phương pháp tiết kiệm thời gian robot. Chi phí được so ở pilot
này là quãng đường mô phỏng, không phải thời gian Nav2 hay năng lượng.

## Ablation sau khi pilot V1 hoàn tất

V1 được giữ nguyên. `budget_ablation.py` chạy thêm sáu nhánh structural, chỉ
cho phép mục tiêu VERIFY có shortest path nằm trong ngân sách còn lại. Khi
không có mục tiêu như vậy, fallback exploration; không phát sinh STOP.
Mười hai nhánh đối chứng V1 được tái dùng có ghi provenance, không tính là
mười hai run mới. Đây là chẩn đoán phát triển sau kết quả, không phải tập
confirmation độc lập. Chạy và kiểm tra bằng:

```bash
python -m active_verification_2d.budget_ablation \
  --mapex-root "$MAPEX_ROOT" --worker-python "$MAPEX_LAMA_PYTHON"
python -m active_verification_2d.validate_ablation
python -m active_verification_2d.headroom
```

`headroom.py` chỉ chạy phía evaluator: enumerate một hành động tới các view
đã sinh, trong budget, giữ prediction phần chưa quan sát cố định rồi đo tác
dụng trực tiếp của sensor correction. Đây không phải oracle toàn cục, không
phản ánh mọi cập nhật LaMa về sau, và không được dùng để chọn hành động online.
