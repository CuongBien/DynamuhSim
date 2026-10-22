# Tầng bằng chứng bề mặt depth — 23/10/2026

**Đã chặn8lượt gán hỗ trợ người sai trên hộp phẳng, giữ toàn bộ cụm đo và lượt hỗ trợ đúng. Cảnh mới vẫn còn1lượt nhận nhầm vật thể cong. Chưa nghiệm thu detector người tổng thể hoặc train.** [Acceptance](acceptance.json), [audit](audit.json), [code/lệnh chạy](../README.md), [ảnh lỗi còn lại5s](preview/new_remaining_failure/semantic_review.png).

Đã làm theo lựa chọn của người dùng: chỉ LiDAR+depth, không thêm RGB. Lượt DR-SPAAM trước đã commit51bc106 và giữ FAIL; tầng mới dùng chính các nguồn cảm biến đó, không sửa ROS/Nav2/tracker/default hoặc dữ liệu cũ.

## Cách làm và phạm vi

Từ depth/intrinsics/extrinsic gốc, dựng cloud và cụm theo projector/shape filter cũ. Không đọc actor/GT/pixel owner trong nhánh suy luận. Với cụm≥60điểm, lấy192cặp điểm deterministic cho mỗi vòng để tìm tối đa2mặt phẳng thẳng đứng; tolerance5mm, mỗi mặt cần≥20điểm và span cao≥25cm, hai normal cách80–100°. Cần giải thích≥98%điểm mới có planar_surface_evidence. Điểm thưa/patch ngắn/curved hoặc chưa giải thích đủ vẫnunknown.

Bằng chứng phẳng chỉ được dùng rút hỗ trợ person_supported_experimental từ LiDAR. **Không xoá proposal, không gọi mọi cụm còn lại là người, không gọi planar evidence là semantic object truth.** Confidence/learned index gốc giữ để truy vết nhưng status mới unknown và semantic_valid=false. Một phần cơ thể/quần áo có thể nhìn phẳng ở pose khác; chưa dùng gate này như chứng nhận con người thật.

Tính mặt phẳng bằng CUDA double precision trên RTX4060; không cài thêm package sau venvCUDA của lượt trước. 7behavior tests (plane,corner,curve,partial/sparse,shortpatch,rotation,CPU/CUDA) chạy trước regression. frozen.json chốt mã/params trước khi xem output; scene_frozen.json chốt fixture mới trước capture. Không thay ngưỡng/model hoặc chọn lại geometry theo kết quả.

## Kết quả cũ và fixture mới

| Điều kiện | Proposal rows | Hỗ trợ sai trước→sau | Hỗ trợ đúng trước→sau | Cụm người bị gắn planar |
|---|---:|---:|---:|---:|
|Regression full|30|4→0|1→1|0|
|Regression null, không người|10|3→0|0→0|0|
|Fixture mới|35|2→1|0→0|0|

Các con số là quan sát lặp trên5timestamp/điều kiện, không phải75người hoặc episode độc lập. Tổng40person-owned candidate rows được giữ, gồm các người chỉ lộ ít điểm; toàn bộ75proposal và mọi learned detection gốc được giữ. Tầng mới chỉ rút8false-support rows, không tạo thêm true support. Do đó chưa giải quyết recall.

Mỗi fixture full có4người đủ pixel/điểm để đánh giá; t2s không có scan hợp lệ theo availability contract, t3–6s có scan. Không dùng sự thiếu input này để gọi model bỏ sót. Dù xét frame cóinput, full semantic gate vẫn FAIL vì thiếu người được hỗ trợ; fixture mới cònfalsepositive.

Fixture mới giữ camera thấp,LiDAR360tia/3.5m/5Hz/noise1cm và nhà xưởng; đổi vị trí/hướng người, hai hộp xoay góc0.4/−0.5rad, thêm trụ cong không phải người. Có75depth/20scan mới trên actual Ogre NVIDIA, mọi callback balanced, pause/stable/stop exit0 và cleanup rỗng. Không có Nav2 hoặc human policy động. Đây là một fixture mới sau khi chốt thuật toán, chưa phải đánh giá thống kê across maps/simulators.

Trên cả hai bố trí, hai hộp có surface explained fraction100% và không có person-owned cluster nào bị đánh planar. Người có16/19/30điểm thiếu ngưỡng60được đểunknown, không bị loại. Trụ cong mới vẫn có448điểm, chỉ giải thích được~28.57% bằng các mặt đang xét, nên không có planar veto. DR-SPAAM gắn hỗ trợ sai vào nó tại5s; **failure này giữ nguyên**, không thêm luật bán kính hay hạ/nângconfidence để giấu lỗi.

[Ảnh cũ trước](preview/old_before/semantic_review.png) / [sau](preview/old_after/semantic_review.png) và ảnh mới midpoint dùng frame2/t4s cố định. [Ảnh failure5s](preview/new_remaining_failure/semantic_review.png) chọn theo quy tắc first remaining false support, ghi rõ trong source.json; không chọn ảnh đẹp để trình bày thành công. Màu xanh trên plot prediction chỉ có nghĩa model đang hỗ trợ nhãn người, không bảo đảm đúng người. Nhãn pixel ở khung giữa là LABEL ONLY từ counterfactual renderer, không inputmodel. Tất cả hình là dữ liệu mô phỏng ghi thật/plot khoa học, không AI sinh.

## Audit và quyết định

[Audit](audit.json) độc lập tính lại75proposal trên CPU soCUDA, kiểm18270point residual/height từ normal/offset đã lưu, trạng thái trước→sau, không mất raw positions/point_count hoặc true support. Source/scene hashes không đổi sau freeze/capture;15221file có sẵn giữ nguyên. Capture/provenance/integrity và surface-only gate PASS theo phạm vi đã đo.

**Giữ gate tổng:** regression full FAIL; regression null PASS sauveto; fresh full FAIL. Không đổi surface-only PASS thành semantic-person PASS. Semantic/association/personaH1–H5/train masks đềufalse, HuNavSim chưa dùng; World Model chưa train, navigationTIMEOUT chưa sửa. Camera vẫn thấp và LiDAR vẫn nhìn proxy người theo profile gốc; chưa áp dụng tự độngvào ROS.

Git lưu code, manifests, capture metadata/native scans, full-depth5frame nén, representative counterfactual LABEL ONLY, predictions/evaluations/ảnh và audit. Raw đủ15view×5frame còn trong generated/runs/newscene01, Gitignore, không backup toàn bộ bằng Git. Để audit đầy đủ cần local arrays/checkpoint.

Bước kế có thể bắt đầu xây **bộ mẫu perception depth có nhãn nhỏ**: người/vật thể cong, chuyển động và che khuất, dùng để chọn/huấn luyện rồi kiểm định person-vs-object classifier. Đây khác bộ train World Model hàng loạt; raw/GT có thể thu trước, nhưng không bật semantic supervision hoặc train-eligibility chỉ nhờ gate phẳng này.
