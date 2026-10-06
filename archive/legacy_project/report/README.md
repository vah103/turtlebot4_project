# Báo cáo tiến độ hằng ngày

Thư mục này lưu các báo cáo ngắn theo ngày để theo dõi tiến trình project TurtleBot4 và chuẩn bị công việc cho ngày tiếp theo.

## Quy tắc báo cáo

- Chỉ ghi những việc có bằng chứng đã thực hiện hoặc đã xác nhận.
- Không suy đoán, không bịa thêm kết quả và không biến kế hoạch thành việc đã hoàn thành.
- Việc chưa kiểm tra phải ghi rõ là **chưa xác nhận**.
- Lỗi chưa tìm được nguyên nhân phải ghi rõ là **chưa xác định nguyên nhân**.
- Nội dung ngắn gọn, ưu tiên kết quả thực tế, lỗi gặp phải và trạng thái cuối ngày.
- Khi có dữ liệu từ GitHub, ưu tiên commit, file và trạng thái trên nhánh `main`.
- Khi có dữ liệu chỉ tồn tại trên laptop hoặc robot nhưng chưa push lên GitHub, phải ghi rõ nguồn thông tin.
- Không đưa mật khẩu, token, credential hoặc thông tin nhạy cảm vào báo cáo.

## Quy ước tên file

Mỗi ngày tạo một file:

```text
YYYY-MM-DD.md
```

Ví dụ:

```text
2026-07-14.md
```

Nếu một ngày có nhiều phiên báo cáo, dùng:

```text
YYYY-MM-DD-session-2.md
```

## Mẫu báo cáo

```markdown
# Báo cáo ngày YYYY-MM-DD

## 1. Đã thực hiện

- Chỉ liệt kê việc thực sự đã làm.

## 2. Kết quả đã xác nhận

- Ghi kết quả kiểm tra, file tạo ra, commit hoặc trạng thái hệ thống đã xác nhận.

## 3. Sự cố và cách xử lý

- Sự cố:
- Cách xử lý:
- Trạng thái cuối cùng:

## 4. File hoặc cấu hình thay đổi

- `đường/dẫn/file`: mô tả ngắn.
- Nếu không có thay đổi, ghi: `Không có`.

## 5. Trạng thái cuối ngày

- Những phần đang hoạt động.
- Những phần chưa xác nhận hoặc còn dang dở.

## 6. Việc nên làm ngày tiếp theo

1. Việc ưu tiên nhất.
2. Việc tiếp theo.
3. Kiểm tra an toàn cần thực hiện trước khi bắt đầu.
```

## Yêu cầu dành cho ChatGPT

Khi người dùng yêu cầu tạo báo cáo trong ngày:

1. Đọc `AGENTS.md` trước.
2. Đọc `PROJECT_STATUS.md` và các commit mới nhất trên nhánh `main`.
3. Chỉ sử dụng nội dung đã được người dùng cung cấp, đã được xác nhận trong cuộc trò chuyện hoặc có bằng chứng trong repository.
4. Phân biệt rõ:
   - Đã hoàn thành.
   - Đã thử nhưng chưa thành công.
   - Chưa kiểm tra.
   - Kế hoạch cho ngày tiếp theo.
5. Tạo file trong `report/` theo định dạng ngày.
6. Không sửa source code, map hoặc cấu hình khác khi chỉ được yêu cầu tạo báo cáo.
7. Trước khi ghi lên GitHub, phải trình bày nội dung báo cáo để người dùng xác nhận nếu yêu cầu chưa cho phép ghi trực tiếp.
