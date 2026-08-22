# Experimental frontier utilities

Các node trong thư mục này không thuộc baseline frontier mặc định. Chúng được giữ để phục vụ ablation/thí nghiệm mà không làm lẫn pipeline runtime chính.

- `frontier_map_preprocessor.py`: tạo `/frontier_map` đã lọc từ raw SLAM map.
- `frontier_map_preprocessor_resilient.py`: wrapper TF-resilient cho preprocessor.

Executable `frontier_map_preprocessor` vẫn được giữ để các thí nghiệm cũ tiếp tục chạy.
