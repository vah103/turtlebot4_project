# Research notes — so sánh 7 nghiên cứu về cơ chế dừng khám phá

File này ghi lại các tiêu chí người dùng đã yêu cầu so sánh giữa 7 bài liên quan đến dừng khám phá sớm.

## Quy tắc trình bày

- Luôn giữ nguyên thứ tự 1 → 7 như bên dưới để tránh nhầm bài.
- Nếu cần xếp hạng, ghi thêm một mục xếp hạng riêng; không đổi thứ tự 1 → 7.
- Các mức như thấp / trung bình / cao là đánh giá định tính để hỗ trợ chọn hướng nghiên cứu, không phải số đo tuyệt đối giữa các bài.
- Khi đưa số liệu chính thức vào luận văn, phải quay lại bài gốc để xác minh con số và điều kiện thí nghiệm.

## 7 bài

1. **Enough is Enough: Towards Autonomous Uncertainty-driven Stopping Criteria** — *Đủ là đủ: hướng tới tiêu chí dừng tự động dựa trên độ bất định* (2022).
2. **Estimating Map Completeness in Robot Exploration** — *Ước lượng độ hoàn chỉnh bản đồ trong khám phá bằng robot* (2026).
3. **Optimizing Exploration with a New Uncertainty Framework for Active SLAM Systems** — *Tối ưu khám phá bằng khung độ bất định mới cho hệ SLAM chủ động* (2025).
4. **PUL-SLAM: Path-Uncertainty Co-Optimization with Lightweight Stagnation Detection for Efficient Robotic Exploration** — *Đồng tối ưu đường đi–độ bất định với phát hiện đình trệ nhẹ cho khám phá robot hiệu quả* (2026).
5. **A Novel Stop Criterion to Support Efficient Multi-Robot Mapping** — *Tiêu chí dừng mới hỗ trợ lập bản đồ đa robot hiệu quả* (2019).
6. **Exploration of Indoor Environments through Predicting the Layout of Partially Observed Rooms** — *Khám phá môi trường trong nhà bằng dự đoán bố cục các phòng được quan sát một phần* (2021).
7. **Sampling-based Incremental Information Gathering with Applications to Robotic Exploration and Environmental Monitoring** — *Thu thập thông tin gia tăng dựa trên lấy mẫu cho khám phá robot và giám sát môi trường* (2019).

---

## Bảng so sánh chính

| # | Robot dừng khi nào? | Mức phù hợp với mục tiêu dừng sớm nhưng vẫn giữ bản đồ tốt | Độ phức tạp xây dựng cơ chế | Độ kỹ của thí nghiệm | Cần huấn luyện? | Lượng tính toán khi chạy |
|---|---|---|---|---|---|---|
| **1** | Mức cải thiện tổng hợp của thông tin SLAM (Simultaneous Localization and Mapping – định vị và lập bản đồ đồng thời) và diện tích bản đồ xuống dưới khoảng **2% trong 3 lần liên tiếp**. | **Khá tốt** — logic bão hòa rõ, nhưng bằng chứng thực nghiệm còn ít. | **Trung bình** — phải lấy và xử lý thông tin từ đồ thị tư thế của SLAM, nhưng không cần học máy. | **Thấp** — chỉ ít môi trường/trajectory (quỹ đạo), gần như không có lặp nhiều lần hay thống kê mạnh. | **Không** | **Thấp** |
| **2** | Mạng CNN (Convolutional Neural Network – mạng nơ-ron tích chập) nhìn bản đồ hiện tại và dự đoán rằng bản đồ đã đủ hoàn chỉnh để kết thúc. | **Rất mạnh** — stopping (cơ chế dừng) là mục tiêu trực tiếp; kiểm chứng rộng và cho thấy tiết kiệm thời gian rõ. | **Cao** — phải tạo dữ liệu, nhãn, huấn luyện mô hình rồi tích hợp vào hệ khám phá. | **Rất cao** — dữ liệu và số lần chạy lớn, nhiều môi trường, có kiểm tra khả năng tổng quát hóa. | **Có** | **Trung bình** khi chạy; phần huấn luyện nặng hơn nhiều so với suy luận trực tuyến. |
| **3** | Không còn **Uncertainty Frontier (biên độ bất định)** đáng hoặc có thể tiếp cận để xử lý. | **Thấp hơn cho mục tiêu early stopping (dừng sớm)** — mục tiêu chính là làm bản đồ đáng tin hơn, robot có thể phải đi kỹ hơn. | **Cao** — phải xây bản đồ độ bất định, phát hiện biên độ bất định và lập kế hoạch xử lý chúng. | **Trung bình** — có nhiều cấu hình và lặp lại, nhưng chủ yếu mô phỏng trong môi trường khá đơn giản. | **Không** | **Trung bình–cao** |
| **4** | Phát hiện **stagnation (đình trệ)**: robot vẫn cố khám phá nhưng bản đồ gần như không mở rộng nữa. | **Mạnh về hiệu quả tổng thể** — vừa cải thiện đường đi vừa có cơ chế dừng; khó tách riêng lợi ích do stopping. | **Rất cao** — có DRL (Deep Reinforcement Learning – học tăng cường sâu), PPO (Proximal Policy Optimization – tối ưu chính sách gần), tối ưu đường đi/độ bất định và phát hiện đình trệ. | **Khá cao** — nhiều môi trường mô phỏng, lặp nhiều lần, nhiều phương pháp đối chứng và có thử robot thật. | **Có** | **Cao** |
| **5** | So sánh lượng thông tin kỳ vọng và lượng thông tin thực tế; một robot dừng khi sai khác đủ nhỏ trong nhiều mục tiêu liên tiếp, và nhiệm vụ kết thúc khi các robot đều dừng. | **Khá** — có thể giảm quãng đường trong khi sai số lập bản đồ vẫn thấp, nhưng thiên về hệ đa robot. | **Trung bình–cao** cho toàn hệ; riêng luật dừng khá đơn giản. | **Trung bình** — có khảo sát theo số robot/tham số nhưng chủ yếu mô phỏng synthetic maze (mê cung tổng hợp). | **Không** | **Trung bình** |
| **6** | Tổng diện tích chưa khám phá mà bố cục dự đoán cho rằng còn có thể quan sát từ các frontier (biên giữa vùng đã biết và chưa biết) còn lại nhỏ hơn ngưỡng, trong bài là khoảng **1 m²**. | **Mạnh** — rất sát mục tiêu “dùng dự đoán để biết phần còn lại có đáng đi không”. | **Cao** về toàn hệ vì phải dự đoán bố cục và tính lượng thông tin; riêng phần dừng tương đối nhẹ. | **Khá cao** — khoảng 10 tòa nhà, 10 lần chạy mỗi môi trường; so rõ baseline (phương pháp nền), prediction (dự đoán) và prediction + early stopping (dự đoán + dừng sớm). Điểm yếu là chủ yếu mô phỏng và đánh giá chất lượng bản đồ sau dừng chưa sâu. | **Không cần huấn luyện mô hình học máy trong cơ chế của bài này** | **Trung bình** |
| **7** | **Entropy (độ bất định thông tin)** trung bình của bản đồ đạt trạng thái bão hòa, nghĩa là đi thêm hầu như không thu được thông tin đáng kể. | **Trung bình nếu chỉ xét early stopping (dừng sớm)** — cơ sở lý thuyết mạnh nhưng stopping chỉ là một phần của hệ lớn hơn. | **Rất cao** — có lập kế hoạch lấy mẫu, không gian niềm tin, lượng thông tin, độ bất định tư thế và bản đồ xác suất. | **Cao về toàn framework (khung phương pháp), trung bình nếu chỉ xét bằng chứng cho stopping** — nhiều thí nghiệm nhưng không tập trung tách riêng phần tiết kiệm do dừng. | **Không** | **Rất cao** |

---

## Nhận xét riêng theo từng tiêu chí

### 1. Cơ chế dừng

- **Bài 1:** dừng khi mức cải thiện đã bão hòa.
- **Bài 2:** dừng khi mô hình đánh giá bản đồ đã đủ hoàn chỉnh.
- **Bài 3:** dừng khi không còn vùng bất định đáng xử lý.
- **Bài 4:** dừng khi exploration (quá trình khám phá) bị đình trệ.
- **Bài 5:** dừng khi thông tin thực tế thu được không còn khác đáng kể so với kỳ vọng.
- **Bài 6:** dừng khi phần diện tích chưa biết được dự đoán còn đáng khám phá là quá nhỏ.
- **Bài 7:** dừng khi entropy (độ bất định thông tin) của bản đồ đã bão hòa.

### 2. Độ mạnh cho mục tiêu “dừng sớm, khám phá ít hơn nhưng vẫn hiệu quả”

- **Bài 1:** khá tốt về ý tưởng, nhưng kiểm chứng còn mỏng.
- **Bài 2:** rất mạnh và trực tiếp nhất.
- **Bài 3:** không tập trung vào dừng sớm; ưu tiên giảm độ bất định của bản đồ.
- **Bài 4:** mạnh về hiệu quả tổng thể, nhưng lợi ích đến từ cả chính sách di chuyển và stopping (cơ chế dừng).
- **Bài 5:** khá, nhưng bối cảnh đa robot khác với MapEx một robot.
- **Bài 6:** mạnh và rất gần hướng MapEx vì dùng prediction (dự đoán) để quyết định phần còn lại có đáng đi không.
- **Bài 7:** có cơ sở thông tin tốt, nhưng dừng sớm không phải đóng góp duy nhất hay trọng tâm duy nhất.

### 3. Độ phức tạp xây dựng

- **Bài 1:** trung bình.
- **Bài 2:** cao.
- **Bài 3:** cao.
- **Bài 4:** rất cao.
- **Bài 5:** trung bình–cao cho toàn hệ; luật dừng riêng nhẹ hơn.
- **Bài 6:** cao cho toàn hệ; luật dừng riêng nhẹ.
- **Bài 7:** rất cao.

### 4. Độ kỹ của thí nghiệm

- **Bài 1:** thấp.
- **Bài 2:** rất cao.
- **Bài 3:** trung bình.
- **Bài 4:** khá cao.
- **Bài 5:** trung bình.
- **Bài 6:** khá cao.
- **Bài 7:** cao cho toàn phương pháp, nhưng trung bình nếu chỉ hỏi “stopping giúp tiết kiệm bao nhiêu”.

### 5. Bài nào cần huấn luyện

- **Bài 1:** không.
- **Bài 2:** có — huấn luyện CNN (Convolutional Neural Network – mạng nơ-ron tích chập).
- **Bài 3:** không.
- **Bài 4:** có — huấn luyện DRL (Deep Reinforcement Learning – học tăng cường sâu) với PPO (Proximal Policy Optimization – tối ưu chính sách gần).
- **Bài 5:** không.
- **Bài 6:** không cần huấn luyện mô hình học máy trong cơ chế được trình bày.
- **Bài 7:** không.

=> Trong 7 bài, **bài 2 và bài 4 là hai bài cần giai đoạn huấn luyện mô hình rõ ràng**.

### 6. Độ nặng tính toán trực tuyến

Đánh giá tương đối đã dùng trong trao đổi:

**7 > 4 > 3 > 2 ≈ 6 > 5 > 1**

Giải thích ngắn:
- **Bài 7:** nặng nhất vì liên tục lấy mẫu quỹ đạo, tính lượng thông tin và độ bất định trong không gian niềm tin.
- **Bài 4:** nặng do chính sách học tăng cường + đánh giá độ bất định + phát hiện đình trệ.
- **Bài 3:** phải duy trì và xử lý bản đồ độ bất định liên tục.
- **Bài 2:** khi đã huấn luyện xong chủ yếu là chạy suy luận CNN (Convolutional Neural Network – mạng nơ-ron tích chập).
- **Bài 6:** dự đoán bố cục và lượng thông tin nhưng không có mạng học sâu trong cơ chế này.
- **Bài 5:** luật dừng khá nhẹ; chi phí tăng do đa robot và tính lượng thông tin.
- **Bài 1:** cơ chế dừng nhẹ nhất trong nhóm này.

---

## So sánh nhanh bài 4 và bài 7

### Bài 4 — PUL-SLAM

- Cải thiện cả cách robot di chuyển và cách phát hiện lúc nên dừng.
- Dùng DRL (Deep Reinforcement Learning – học tăng cường sâu) và PPO (Proximal Policy Optimization – tối ưu chính sách gần).
- Dừng khi quá trình khám phá bị **stagnation (đình trệ)**: robot vẫn cố hoạt động nhưng bản đồ gần như không mở rộng.

### Bài 7 — IIG (Incremental Information Gathering – thu thập thông tin gia tăng)

- Dùng lập kế hoạch lấy mẫu dựa trên lý thuyết thông tin để chọn đường đi giàu thông tin hơn.
- Xét cả độ bất định vị trí robot và độ bất định bản đồ trong việc đánh giá đường đi.
- Dừng khi entropy (độ bất định thông tin) của bản đồ đã bão hòa.

### Khác nhau cốt lõi

- **Bài 4:** “robot không còn tiến triển thực tế” → dừng.
- **Bài 7:** “đi tiếp gần như không còn mang lại thông tin mới” → dừng.

Cả hai đều không phải chỉ thêm một luật dừng vào planner (bộ lập kế hoạch) cũ; chúng còn cải thiện chính chiến lược exploration (khám phá), vì vậy lợi ích về thời gian/quãng đường không thể quy hoàn toàn cho stopping (cơ chế dừng).

---

## Liên hệ với hướng MapEx hiện tại

Đối với mục tiêu **giảm thời gian/quãng đường khám phá nhưng giữ chất lượng bản đồ chấp nhận được**:

- **Bài 2** là tham chiếu mạnh cho ý tưởng đánh giá trực tiếp “đã khám phá đủ chưa?”.
- **Bài 6** gần MapEx nhất về tư duy: dùng phần môi trường được dự đoán để quyết định phần còn lại có còn đáng khám phá hay không.
- **Bài 1** hữu ích cho nguyên tắc “dừng khi mức cải thiện bão hòa”.
- **Bài 4 và 7** cần cẩn thận khi so phần trăm tiết kiệm vì chúng đồng thời thay đổi cách robot chọn đường/khám phá, không chỉ thay đổi điều kiện dừng.
- **Bài 3** hữu ích để chứng minh uncertainty (độ bất định) có thể là tín hiệu dừng, nhưng mục tiêu chính thiên về chất lượng/độ tin cậy bản đồ hơn là dừng càng sớm càng tốt.

## Nguồn bài gốc

1. https://arxiv.org/pdf/2204.10631
2. https://link.springer.com/content/pdf/10.1007/s10514-025-10221-8.pdf
3. https://arxiv.org/pdf/2506.17775
4. https://arxiv.org/pdf/2511.04180
5. https://ieeexplore.ieee.org/document/9018592
6. https://aamas.csc.liv.ac.uk/Proceedings/aamas2021/pdfs/p836.pdf
7. https://arxiv.org/pdf/1607.01883
