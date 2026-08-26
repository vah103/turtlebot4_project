# Agent / ChatGPT Instructions

Khi được yêu cầu làm việc trong `mapex_hospital_research/`, hãy coi folder này là nguồn context chính.

## Đọc trước khi làm

1. `STATUS.md`
2. `ROADMAP.md`
3. `references/README.md`
4. `references/MAPEX_PAPER.md`
5. `EXPERIMENT_PROTOCOL.md`
6. `README.md`
7. `docs/DATA_SCHEMA.md` nếu công việc liên quan dữ liệu/experiment.
8. `docs/TEACHER_REPORT.md` nếu công việc tạo ra hoặc diễn giải kết quả nghiên cứu.

Nếu cần đối chiếu phương pháp gốc MapEx, ưu tiên paper gốc được dẫn trong `references/MAPEX_PAPER.md` thay vì suy từ ghi chú cũ.

## Quy tắc nghiên cứu

- Không mặc định ranking, uncertainty, visibility hoặc WFD là bottleneck.
- Xác nhận closed-loop failure trước khi chọn nguyên nhân.
- Ưu tiên nhiều run và dữ liệu có thể tái lập.
- Dùng oracle/ablation để xác định bottleneck.
- Chỉ gọi một vấn đề là research gap sau khi kiểm tra literature.
- Khi so oracle, không so trực tiếp % cải thiện giữa các metric khác loại.

## Quy tắc code/data

- Code/config mới cho lộ trình này nên đặt trong folder này.
- Có thể gọi lại dependency trong `ros2_ws/src/frontier_exploration/`, nhưng tránh copy code cũ nếu không cần.
- Không commit dữ liệu thô nặng.
- Không thay protocol giữa Nearest và MapEx mà không ghi lại.

## Quy tắc báo cáo cho giảng viên

- Repo là source of evidence; Google Docs `TurtleBot4`, tab `Báo cáo nghiên cứu MapEx Hospital`, là bản teacher-facing.
- Chỉ cập nhật báo cáo chính khi một milestone có kết quả có ý nghĩa và đã được xác minh.
- Pilot run, debug log hoặc số liệu chưa kiểm tra chỉ ghi trong repo/notes, không trình bày như kết quả chính thức.
- Mỗi phần báo cáo ưu tiên cấu trúc: Mục tiêu → Thiết lập → Kết quả → Phân tích → Kết luận → Bước tiếp theo.
- Mọi con số đưa vào báo cáo phải truy ngược được về `experiments/` hoặc `results/`.
- Xem đầy đủ quy tắc trong `docs/TEACHER_REPORT.md`.

## Trước khi kết thúc một phiên làm việc

- Cập nhật `STATUS.md` với DONE / IN PROGRESS / NEXT ACTION / LATEST RESULT.
- Nếu thay protocol, cập nhật `EXPERIMENT_PROTOCOL.md`.
- Nếu phát hiện lỗi hoặc quyết định quan trọng, ghi `docs/experiment_notes.md`.
- Nếu thêm hoặc dùng paper mới để đưa ra quyết định nghiên cứu, cập nhật `references/RELATED_WORK.md` hoặc file reference tương ứng.
- Nếu phiên này hoàn thành một milestone có kết quả đủ tin cậy, cập nhật phần tương ứng trong Google Docs `TurtleBot4` → `Báo cáo nghiên cứu MapEx Hospital` theo `docs/TEACHER_REPORT.md`.
