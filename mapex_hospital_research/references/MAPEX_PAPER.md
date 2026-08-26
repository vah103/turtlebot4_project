# MapEx — tài liệu gốc cần bám theo

## Paper

**MapEx: Indoor Structure Exploration with Probabilistic Information Gain from Global Map Predictions**

- arXiv: https://arxiv.org/abs/2409.15590
- PDF: https://arxiv.org/pdf/2409.15590
- Publication context: ICRA 2025.

## Vai trò của tài liệu này trong project

Đây là reference chính để kiểm tra:

- pipeline MapEx gốc;
- cách tạo frontier candidates;
- cách dùng map prediction;
- ensemble mean / variance;
- probabilistic visibility / raycasting;
- Information Gain;
- frontier score;
- baseline và metric của paper.

Khi code hoặc phân tích có mâu thuẫn với ghi chú nội bộ, phải quay lại paper gốc để xác minh.

## Pipeline khái quát

```text
Observed occupancy map
→ WFD frontier candidates
→ LaMa-based global map prediction
→ ensemble predictions
→ mean prediction + variance map
→ probabilistic raycasting / predicted visibility
→ Information Gain
→ score frontier theo utility và distance
→ chọn frontier
```

Trong lộ trình nghiên cứu Hospital, pipeline này phải được log theo từng decision để có thể kiểm tra từng tầng độc lập.

## Baseline / metric cần nhớ

Paper MapEx so sánh với các phương pháp gồm:

- Nearest Frontier
- IG-Hector
- UPEN

Các metric chính của paper gồm:

- Coverage
- occupied-class IoU
- Topological Understanding (TU)

Nếu muốn tái hiện số liệu paper hoặc dùng exact parameter/equation, luôn đọc trực tiếp paper thay vì suy từ file ghi chú này.

## Nguyên tắc sử dụng

File này chỉ là bản định hướng nhanh cho agent/ChatGPT. Nó không thay thế paper gốc và không được dùng để tự suy ra các con số hoặc hyperparameter chưa được xác minh.
