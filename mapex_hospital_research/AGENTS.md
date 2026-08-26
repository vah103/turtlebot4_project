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

## Trước khi kết thúc một phiên làm việc

- Cập nhật `STATUS.md` với DONE / IN PROGRESS / NEXT ACTION / LATEST RESULT.
- Nếu thay protocol, cập nhật `EXPERIMENT_PROTOCOL.md`.
- Nếu phát hiện lỗi hoặc quyết định quan trọng, ghi `docs/experiment_notes.md`.
- Nếu thêm hoặc dùng paper mới để đưa ra quyết định nghiên cứu, cập nhật `references/RELATED_WORK.md` hoặc file reference tương ứng.
