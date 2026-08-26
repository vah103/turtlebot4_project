# Teacher-facing report workflow

Tài liệu này quy định cách chuyển kết quả kỹ thuật trong `mapex_hospital_research/` thành nội dung báo cáo dễ đọc cho giảng viên hướng dẫn.

## Nơi lưu báo cáo chính

Báo cáo chính nằm trong Google Docs **`TurtleBot4`**, tab:

**`Báo cáo nghiên cứu MapEx Hospital`**

Repo là nguồn bằng chứng kỹ thuật; Google Docs là bản trình bày teacher-facing.

## Hai lớp thông tin phải tách biệt

### Repo — source of evidence
Lưu đầy đủ:
- code/config;
- dữ liệu từng run;
- CSV/JSON/log;
- figure/table;
- metadata và commit;
- kết quả pilot, failure và debugging cần thiết.

### Google Docs — source of presentation
Chỉ đưa vào:
- mục tiêu thí nghiệm;
- thiết lập đủ để hiểu phép so sánh;
- kết quả đã xác minh;
- bảng/figure quan trọng;
- diễn giải kết quả;
- kết luận và bước nghiên cứu tiếp theo.

Không dump terminal log, stack trace, đường dẫn local hoặc chi tiết debugging vào phần báo cáo chính.

## Khi nào phải cập nhật báo cáo

Cập nhật Google Docs khi một milestone tạo ra **kết quả có ý nghĩa và đã được xác minh**, ví dụ:
- Hospital protocol đã được chốt và chạy ổn định;
- Nearest baseline đã có đủ số run để báo cáo sơ bộ/chính thức;
- MapEx closed-loop đã có đủ số run;
- có kết quả comparison/stage analysis đáng tin;
- hoàn thành một component diagnosis;
- hoàn thành oracle/ablation;
- xác định bottleneck;
- literature check làm thay đổi research direction;
- proposed method hoặc benchmark mới có kết quả.

Không đưa một pilot run đơn lẻ, số liệu chưa kiểm tra hoặc giả thuyết chưa được chứng minh vào phần kết quả chính. Có thể ghi chúng trong `STATUS.md` hoặc `docs/experiment_notes.md`.

## Cấu trúc viết cho mỗi milestone

Mỗi phần báo cáo nên ưu tiên cấu trúc:

1. **Mục tiêu** — câu hỏi cần trả lời.
2. **Thiết lập thí nghiệm** — môi trường, phương pháp, số run, điều kiện so sánh và metric chính.
3. **Kết quả** — bảng/figure và số liệu tổng hợp.
4. **Phân tích** — kết quả có ý nghĩa gì; tránh suy diễn vượt quá dữ liệu.
5. **Kết luận** — câu hỏi ban đầu đã được trả lời tới đâu.
6. **Bước tiếp theo** — bước nào trong `ROADMAP.md` được kích hoạt bởi kết quả này.

## Cấu trúc tổng của tab báo cáo

1. Mục tiêu và phương pháp nghiên cứu
2. Chuẩn hóa môi trường Hospital
3. Nearest Frontier baseline
4. MapEx closed-loop
5. So sánh MapEx và Nearest
6. Phân tích theo exploration stage
7. Chẩn đoán từng tầng pipeline MapEx
8. Oracle / ablation
9. Xác định bottleneck
10. Đối chiếu literature và research gap
11. Phương pháp đề xuất
12. Benchmark và kết luận

Các mục chưa có kết quả được giữ như placeholder; không điền số giả định.

## Quy tắc tính nhất quán

- Mọi con số trong Google Docs phải truy ngược được về `experiments/` hoặc `results/`.
- Nếu một số liệu thay đổi sau khi rerun/fix bug, sửa cả summary trong repo và Google Docs.
- Không dùng kết quả từ report cũ làm bằng chứng cho lộ trình mới nếu chưa được tái xác minh theo protocol mới.
- Phân biệt rõ `pilot`, `preliminary` và `final`.
- Khi mô tả research gap, chỉ dùng wording mạnh sau oracle/ablation và literature check theo `ROADMAP.md`.

## Handoff cho ChatGPT/agent

Sau một milestone có kết quả đủ tin cậy:
1. lưu dữ liệu và summary trong repo;
2. cập nhật `STATUS.md`;
3. cập nhật `docs/experiment_notes.md` nếu có quyết định/lỗi quan trọng;
4. viết/cập nhật phần tương ứng trong Google Docs `TurtleBot4` → `Báo cáo nghiên cứu MapEx Hospital` theo phong cách teacher-facing;
5. kiểm tra các số liệu trong báo cáo khớp với file kết quả trong repo.
