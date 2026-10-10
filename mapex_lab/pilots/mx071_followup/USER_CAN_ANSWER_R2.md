# R2 trả lời được gì — bảng cho USER / PM / IR1

**Hiện tại:** đã có bản thiết kế ứng viên, chưa có independent ACCEPT R2, implementation/preflight hoặc dữ liệu mới. R1 ACCEPT chỉ đóng proposal semantics. Bảng này mô tả khả năng kiểm chứng **sau khi** các gate và dữ liệu đủ; không phải kết quả thí nghiệm.

IR1 đã full-review R2 tại36027f2 và REVISE hai điểm (review0038e276). Bản sửa này thêm matching exposure câu3 và hạ claim câu8; chờ focused closure, không tự tuyên bố finding đã đóng.

| Câu | Phép kiểm chứng R2 | Nếu đạt gate, có thể trả lời | Điều chưa thể kết luận |
| --- | --- | --- | --- |
| 1 Mean map | Cùng input/U/V/pool, raycast(mean) so với mean(member raycasts), thực thi hai lựa chọn rồi cùng native continuation | Gộp prediction có đổi hành động và gây tổn thất Q/đường đi/chi phí ở các state được thử không; sửa một lần có lợi online không | Mean map luôn tốt/xấu ở mọi cấu trúc; tổng quát sang New Room; hiệu quả một policy lặp lại |
| 2 Observation update | Pre/post scan trên phần còn unknown chung, cùng decision opportunity; dùng prediction trước/sau để chọn hành động thật | Prediction ngoài tầm vừa quan sát thay đổi/tốt/xấu ra sao; riêng việc dùng prediction cập nhật có làm thay đổi utility không | Model đã học online; cập nhật luôn tốt; lợi ích giữ goal gốc (câu5 đo riêng); một policy dự đoán giá trị observation tương lai |
| 3 Topology | Local GT repair so với neutral/random repairs cùng size/error/direction/distance/dose **và exposure native trước can thiệp**, cộng route-denominator control riêng | Lỗi cửa/tường/kết nối trên P1, có P2 kiểm tra raster, có ảnh hưởng đặc thù lên hành động/task **khi cả hai neutral controls match exposure hợp lệ** không | Thiếu exposure match mà vẫn cô lập topology; robot biết GT để sửa; phát hiện “cùng tự tin sai” online; đã khảo sát mọi lỗi cấu trúc |
| 4 Visibility/gain | Chỉ thay prediction ở unknown bằng GT đúng renderer/frame, giữ U/V/d/pool; chạy paired goals | Phần visibility do prediction sai có headroom Q/route bao nhiêu trong code contract; phần sensor/frame khác được tách ra | IG dự đoán là m² đo thật; oracle uplift là cải tiến online; toàn bộ sai lệch do prediction |
| 5 Goal lock | KEEP, fresh rescore đúng một lần, fresh computation nhưng giữ nguyên control/cache | Việc giữ goal valid khi có observation mới có gây tổn thất không; reconsider một lần có đủ bù inference/replan/revisit không | Thường xuyên đổi goal luôn tốt; đã tránh được mọi oscillation; hành trình ngoài100m đã được thử |
| 6 Viewpoint | Representative, best trong cùng cluster, expanded pool và cheap viewpoint control | Mất mát do representative, so với đổi cluster/pool hoặc chỉ chọn geometry đơn giản; hiệu quả/cost của các view được thử | Đã tìm viewpoint tối ưu toàn cục; mọi occlusion đều được xét; lỗi source rep khi không có alternate hợp lệ |
| 8 Short/future | Tối đa4 action định trước, chạy thật đến hết R; đo G10/Q10 và Q_R trên cùng trace, không thêm scorer intervention | Unique/tied horizon disagreement; native-action tested long-horizon regret; native tình cờ là unique G10 winner nhưng một action đã thử tốt hơn về dài hạn không | Chính short-horizon optimization của MapEx gây lỗi; alignment là nguyên nhân chọn goal; best-tested là global optimum; hindsight winner là online policy; PIPE bỏ sót budget gốc |

**PASS / FAIL / INCONCLUSIVE áp dụng thế nào:**
- PASS ở tầng mechanism hoặc action chỉ chứng minh tầng đó. Gate task yêu cầu đủ4 map means, >=6 run contrasts non-equivalent trên >=3 maps, macro deltaQ>=0.01, >=3 map dương, không tăng behavioral failure và không mất final coverage quá0.5pp/map.
- Online recovery còn phải dùng thông tin robot có, tính đủ compute/movement cost và đạt gate giảm thời gian/chi phí. GT/future oracle không thể PASS tầng online.
- FAIL/NOT_SUPPORTED là không đạt tiêu chuẩn material trên cohort hợp lệ này; không được diễn đạt thành “không có điểm yếu ở mọi map”.
- INCONCLUSIVE khi replay/frame/control/số mẫu/resource/censoring chưa đủ. Không có alternate, thiếu patch topology hoặc thiếu action distinct là thiếu support, không phải kết quả âm.
- NO_EXECUTABLE_EFFECT_ON_TESTED_SUPPORT chỉ khi toàn bộ contract được kiểm tra có cùng hành vi và zero utility; chi phí tính thêm vẫn phải báo.
- Oracle mạnh nhưng chưa có detector/selector hợp lệ online: MECHANISM_PRESENT_BUT_NON_ACTIONABLE.
- Câu3 không có neutral controls match đủ exposure/dose: native→GT topology repair chỉ là oracle headroom, topology specificity INCONCLUSIVE; không đổi bins/patch/redraw để cứu support.
- Câu8 dùng các gate support/materiality cho **NATIVE_ACTION_HAS_TESTED_LONG_HORIZON_REGRET / TESTED_HORIZON_DISAGREEMENT_INVOLVING_NATIVE_ACTION**. Không gán SUPPORTED_CAUSAL_TASK_HARM cho cơ chế short-horizon của source: MapEx tối ưu predicted variance/distance, không tối ưu realized G10. Cần scorer intervention riêng mới cô lập được nguyên nhân ấy.

**Đề tài mới:** R2 có thể loại hướng không đạt hoặc xác định cơ chế/đối chứng đáng theo tiếp. Để gọi thành đề tài đủ mạnh còn cần costed online recovery, đối chiếu literature và validation prospective độc lập; Stage C chưa được mở.

**Tài nguyên và lựa chọn scope:**
- FULL_7:176 scientific requests +16 full KEEP replays; cả7 câu có hợp đồng causal. Quota48GiB, cần50GiB trống dự phòng.
- STAGED_CORE:64 scientific requests cho1/4/8 +16 KEEP; cả7 vẫn có Stage A diagnostic nhưng2/3/5/6 **chưa có kết luận causal task loss/cost**. Quota32GiB, cần34GiB trống.
- Chỉ thiết kế FULL_7 đang được yêu cầu. Không tự chọn staged hoặc chạy batch; PM/USER quyết định nếu preflight không đủ.
- DELL hiện khoảng6.6GiB trống; chưa đạt admission của hai lựa chọn. ETA phải lấy từ full-size real-model/resource preflight, không lấy các scenario minh họa trong RESOURCE_AND_PREFLIGHT_R2.md làm thời gian đã đo.

PIPE vẫn là nhánh audit-first riêng,0 scientific requests trong R2 này. MX072/COM1 và #7 dropped giữ nguyên.

