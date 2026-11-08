# Giao thức kiểm định mô phỏng

Giao thức này quy định các cổng kiểm định khi có runtime. Trước khi chạy, lưu hồ sơ kiểm định với scenario/seed, phiên bản, fingerprint, số lần lặp, metric, sai số cho phép, chính sách lỗi và điều kiện pass/fail. Chọn dung sai theo độ phân giải đồng hồ, cảm biến và sai số số học của profile. Thiếu ngưỡng cho một tiêu chí bắt buộc thì kết quả tiêu chí đó chưa đủ điều kiện nghiệm thu.

| Cổng | Cách kiểm và điều kiện đạt |
| --- | --- |
| G0 · Cấu hình và scene | Resolve mọi tham chiếu và include. So fingerprint với manifest. Kiểm ID, frame, pose spawn, vật cản, mục tiêu và bounds. Thử lỗi tham chiếu, ID trùng, bước thời gian không chia hết và spawn xuyên vật cản. Cấu hình sai phải dừng trước khi episode được công bố. |
| G1 · Hợp đồng và đồng hồ | Test reset mới, zero tick, số substep mỗi bước, timestamp đơn điệu và xử lý terminal/truncated/failed. Gây lỗi bằng NaN, sai shape, sai frame, thiếu ID và lỗi adapter giữa step. Đối chiếu đồng hồ engine với mốc logic theo dung sai profile. Kiểm prefix hợp lệ và reset sau lỗi. |
| G2 · Động học và tiếp xúc | Dùng fixture robot đứng yên, đi thẳng, quay tại chỗ, qua biên yaw, người đi theo vận tốc cố định và vật cản cứng. So hướng, biên độ, độ trượt, giới hạn lệnh và executed với kỳ vọng của từng fixture. Kiểm bảo toàn sự kiện tiếp xúc ngắn giữa control tick và quan hệ ID trong registry. |
| G3 · Cảm biến và cách ly | Dùng vật chuẩn có khoảng cách/hướng biết trước. Kiểm frame, kích thước, đơn vị, tần số, độ trễ, missing/stale, quy ước nhiễu và mặt nạ của sensor có trong profile. Theo dõi đường dữ liệu để bảo đảm policy chỉ thấy mẫu đã sẵn sàng và không nhận trạng thái thật. |
| G4 · Tái lập | Lặp lại cùng seed, cấu hình, phần mềm và nền tảng. So spawn, ID entity, tick, trạng thái kết thúc và chuỗi mẫu. Dùng so sánh chính xác cho ID/tick và dung sai đã khai báo cho đại lượng liên tục. Đổi một seed trong fixture có ngẫu nhiên để kiểm ngẫu nhiên thực sự đi qua đúng luồng. |
| G5 · Sẵn sàng đối chứng | Kiểm manifest, hình học, profile chuyển động, snapshot ban đầu và robot policy giữa các cặp. Chạy pilot trên các nhóm scenario. Chốt rule tính metric và điều kiện lỗi trước khi mở tập đánh giá. |

## Áp dụng theo mốc

- TASK 04A kiểm cấu hình, đồng hồ, vận tốc thực thi, khả năng tái lập và các loại tiếp xúc hoặc phép đo khoảng hở mà profile hành vi công bố. Dùng lệnh robot cố định và danh sách sensor rỗng cho fixture hành vi ban đầu. Giữ cấu hình và số đo để tái hiện lần chạy. [Giao thức hành vi](HUMAN_BEHAVIOR_PROTOCOL.md)
- TASK 04B kiểm thêm G3 và đối chiếu episode đã ghi với kết quả runtime, gồm timestamp, prefix hợp lệ, trạng thái kết thúc và quyền truy cập từng kênh. TASK 04C dùng manifest hoàn chỉnh và các cặp nhánh để kiểm QA và split trước khi công bố tập huấn luyện. [Lộ trình](../ROADMAP.md) · [Schema episode](../schemas/EPISODE_SCHEMA.md)

## Tính số đo và xử lý lỗi

- Tính va chạm theo cặp ID đã phân loại. Dùng sự kiện tiếp xúc trong toàn khoảng step để xác định có va chạm. Nếu cần số đợt va chạm, gộp các mẫu tiếp xúc liên tiếp của cùng cặp theo quy tắc đã chốt.
- Đo khoảng hở bằng khoảng cách có dấu giữa footprint trong `world` tại các snapshot đã chọn. Ghi rõ cặp nào được tính và tần số lấy mẫu. Nếu đo tại control tick, ghi đây là số đo rời rạc. Dùng sự kiện substep để ghi nhận tiếp xúc giữa tick.
- `terminated` và `truncated` là kết quả hợp lệ theo [schema episode](../schemas/EPISODE_SCHEMA.md). Timeout theo ngân sách phải nằm trong mẫu số báo cáo thành công nhiệm vụ của các episode hợp lệ. `failed` hoặc chưa hoàn tất được đếm và giải thích riêng, với mẫu số là toàn bộ lần thử khi báo lỗi kỹ thuật.
- Lưu lỗi, điều kiện tái hiện, tick cuối hợp lệ và các file có liên quan. Khi hạ tầng lỗi, chỉ chạy lại theo chính sách đã công bố, với episode ID mới và liên kết tới lần thử gốc. Không chọn lại seed hoặc âm thầm bỏ lỗi lặp lại của một backend. Báo ảnh hưởng của các cặp thiếu đối với kết luận.
- Khi cổng kiểm định thất bại, giữ kết quả để chẩn đoán và đánh dấu cấu hình chưa đủ điều kiện cho bước nghiên cứu phụ thuộc. [Lộ trình](../ROADMAP.md)
