# Giao thức đối chứng hành vi người

Mục tiêu là đo ảnh hưởng của backend hành vi người trong khi giữ cố định môi trường và cơ chế thực thi chuyển động. Đơn vị so sánh ghép cặp là `(scenario_id, episode_seed, robot_policy_id)`. Mỗi điều kiện chạy một episode riêng với ID riêng.

## Điều kiện đối chứng

| Backend | Định nghĩa điều kiện |
| --- | --- |
| `orca` | Backend đề xuất vận tốc phẳng theo bài toán tránh va chạm tương hỗ. Khóa tham số riêng như horizon, phạm vi lân cận và bán kính mô hình trước đánh giá. |
| `social_force` | Backend dùng mô hình lực xã hội để đề xuất vận tốc. Khóa hệ số mục tiêu, tương tác người, robot và vật cản. Mọi chuyển đổi từ đầu ra nội bộ sang vận tốc thuộc backend. |
| `scripted` | Backend dùng tuyến, mốc thời gian hoặc quy tắc nhường đường được công bố. Xuất vận tốc theo cùng hợp đồng, kể cả khi bám waypoint. Dùng làm đối chứng dễ tái hiện và kiểm tra hạ tầng. |
| Backend mở rộng | Cung cấp ID, phiên bản, tham số, giả định quan sát, seed và hợp đồng reset/propose tương đương. Đăng ký một điều kiện độc lập. |
| Hybrid được khai báo | Một điều kiện riêng có ID và cấu hình nêu thuật toán thành phần, cách phối hợp và điều kiện chuyển chế độ. Ghi chế độ sử dụng theo tick trong chẩn đoán. |

Một run chỉ chọn một backend cho toàn bộ người. Thực hiện đối chứng chính cho từng backend độc lập. Tách seed hoặc scenario hiệu chỉnh tham số khỏi tập đánh giá. Lưu tham số, phiên bản và ngân sách hiệu chỉnh cho từng điều kiện. [Ma trận baseline](../research/BASELINE_MATRIX.md)

## Thiết kế cặp so sánh

- Dùng tối thiểu các nhóm scenario có ý nghĩa khác nhau. Hành lang đối đầu kiểm tra nhường lối. Cửa hẹp cắt ngang kiểm tra quyền đi trước và chờ. Giao cắt khuất tầm nhìn kiểm tra tương tác với robot khi cảm biến bị che. Thêm scenario robot đứng yên và người đi thẳng để cô lập hành vi cơ bản khi kiểm định.
- Với mỗi cặp, giữ nguyên world/MJCF, hình học, footprint, bề mặt, mục tiêu, pose và vận tốc khởi tạo, số lượng/ID người và danh sách vật cản. Cố định seed episode, lịch bước, cấu hình cảm biến, tham số cơ cấu, limiter và tiêu chí dừng. So fingerprint tài nguyên, trường cấu hình kiểm soát và snapshot reset. Chỉ các trường nhận diện, backend và nguồn gốc riêng của điều kiện được phép khác theo kế hoạch.
- Giữ nguyên robot policy, tham số, nhiệm vụ và seed luồng. Dùng kịch bản lệnh cố định khi cần cùng kích thích từ robot. Nếu dùng policy phản hồi, robot có thể phát lệnh khác do quan sát khác, nên ghi lại lệnh và diễn giải kết quả như tương tác vòng kín.
- Trước khi có sensor, dùng profile lệnh robot cố định với danh sách sensor rỗng. Khi đối chứng dùng policy phản hồi, kiểm định các kênh cảm biến cần thiết trước khi chạy.
- Cấp cùng phép chiếu `HumanContext` cho mọi backend, với dữ liệu tại tick hiện tại. Backend có thể dùng tập con đã khai báo. Giữ luồng ngẫu nhiên cho spawn, sensor và robot độc lập với số lần rút của backend. Nếu mô hình hóa ngẫu nhiên cảm biến theo vị trí, giữ quy tắc nhiễu và seed, đồng thời ghi nhận phép đo có thể khác do cảnh đã thay đổi.

## Thực thi và số đo

1. Reset theo manifest, kiểm tra điều kiện ghép cặp, rồi cho từng backend đề xuất `HumanMotion` trong `world` tại mỗi control tick. Quy định đến đích, chờ và xử lý actor đã tới đích trong scenario. Actor giữ ID và có lệnh cho mọi tick.
2. Đưa mọi backend qua cùng một limiter vận tốc/gia tốc, cùng adapter, cơ cấu, substep và quy tắc tiếp xúc. Script chỉ phát lệnh chuyển động. Chỉ reset được đặt lại pose; trong episode không dịch chuyển actor tức thời để bám tuyến, phục hồi hay giải va chạm.
3. Ghi `requested`, `limited`, `executed` và pose thực tế. Tính tỷ lệ tick có giới hạn lệnh và độ lệch giữa limited/executed để phân biệt tác động của hành vi đề xuất với cơ chế thực thi.
4. Tính theo episode tỷ lệ đến đích, thời gian đến đích trong episode thành công, va chạm robot–người/người–người/người–vật cản, khoảng hở nhỏ nhất, thời gian đứng chờ và thời gian dưới ngưỡng tiến triển. Báo riêng episode bị timeout. Khai báo trước ngưỡng chờ, ngưỡng tiến triển, cách lấy khoảng hở và cửa sổ đo. Đối chiếu hành vi quan sát được với tiêu chí hợp lý theo scenario. Tuyên bố tương đồng với người thật cần dữ liệu đối chiếu độc lập.
5. Báo số cặp dự kiến, số cặp hoàn tất, phân bố lý do kết thúc và lỗi theo điều kiện. So chênh lệch trên các cặp hoàn tất và trình bày bất định theo đơn vị cặp hoặc cụm scenario. Giữ kết quả lỗi trong bảng tổng hợp theo [giao thức kiểm định](SIMULATION_VALIDATION_PROTOCOL.md).
