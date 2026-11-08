# Giao thức đối chứng hành vi người

TASK 04A bắt đầu bằng fixture một người `scripted` đi tới đích khi robot đứng yên. Sau khi kiểm chứng vận tốc thực thi và phép đo tiếp xúc, dùng các điều kiện dưới đây để đo ảnh hưởng của backend người và phản ứng với hành động robot. Mỗi điều kiện chạy một episode riêng với ID riêng. [Lộ trình](../ROADMAP.md)

## Điều kiện đối chứng

| Backend | Định nghĩa điều kiện |
| --- | --- |
| `scripted` | Backend dùng tuyến, mốc thời gian hoặc quy tắc nhường đường được công bố. Xuất vận tốc theo cùng hợp đồng, kể cả khi bám waypoint. Dùng làm đối chứng dễ tái hiện và kiểm tra hạ tầng. |
| `orca` | Dùng ORCA/RVO2 để đề xuất vận tốc phẳng. Khóa horizon, phạm vi lân cận, bán kính và cách biểu diễn robot khi robot chạy lệnh cố định. ORCA chuẩn giả định tác nhân cùng chia sẻ trách nhiệm tránh va chạm. [Bài ORCA](https://gamma.cs.unc.edu/ORCA/publications/ORCA.pdf) · [Tài liệu RVO2](https://gamma-web.iacs.umd.edu/RVO2/documentation/2.0/) |
| `social_force` | Dùng mô hình lực xã hội để đề xuất vận tốc. Khóa thành phần hướng tới mục tiêu, tương tác người, robot, vật cản và cách chuyển đầu ra mô hình sang vận tốc. [Helbing–Molnár](https://doi.org/10.1103/PhysRevE.51.4282) |
| Backend mở rộng | AVM và CSM là ứng viên khảo sát. Mỗi mô hình khi đưa vào đối chứng có ID, phiên bản, tham số, giả định quan sát, seed và hợp đồng reset/propose riêng. [JuPedSim](https://www.jupedsim.org/stable/pedestrian_models/index.html) |
| Hybrid được khai báo | Một điều kiện riêng có ID và cấu hình nêu thuật toán thành phần, cách phối hợp và điều kiện chuyển chế độ. Ghi chế độ sử dụng theo tick trong chẩn đoán. |

Một run chỉ chọn một backend cho toàn bộ người. Thực hiện đối chứng chính cho từng backend độc lập. Tách seed hoặc scenario hiệu chỉnh tham số khỏi tập đánh giá. Lưu tham số, phiên bản và ngân sách hiệu chỉnh cho từng điều kiện. [Ma trận baseline](../research/BASELINE_MATRIX.md)

## Căn cứ chọn hành vi và hiệu chỉnh

- Mục 3–4 của báo cáo *Đề xuất xây dựng & kiểm định bộ dữ liệu mô phỏng* xác định các mẫu tương tác, nhóm hành vi người và các nhánh hành động robot cần khảo sát. Ma trận triển khai bắt đầu từ fixture và cặp tiến/dừng trong lộ trình, rồi mở rộng theo kết quả kiểm định. [Lộ trình](../ROADMAP.md)
- Dùng HuNavSim 2.0 để tham khảo cách kích hoạt hành vi theo điều kiện. Nếu đưa các mức hợp tác, trung tính và ít hợp tác vào thí nghiệm, định nghĩa bằng tham số hoặc quy tắc có thể đo và công bố riêng trong cấu hình. [HuNavSim 2.0](https://arxiv.org/html/2507.17317v1#S4)
- Dùng nghiên cứu HRI để chọn hành vi và thiết kế kích thích. Trong nghiên cứu của Hetherington, 10 cm là khoảng lùi của robot và 85% là tỷ lệ người xem video diễn giải đúng tín hiệu lùi để nhường. Senft quan sát dịch ngang và xoay thân khi người đi qua nhau, rồi đánh giá cách robot thực hiện các động tác đó. Tham số phản ứng của người trong NarrowNav cần được hiệu chỉnh cho kịch bản của dự án. [Hetherington](https://arxiv.org/pdf/2104.02279) · [Senft](https://doi.org/10.1145/3319502.3374812)
- Dùng track người được gán nhãn trong NavWareSet để hiệu chỉnh hoặc đánh giá trên các nhóm bản ghi tách riêng. Notebook của nhóm tác giả minh họa cách hiệu chỉnh Social Force. Dùng SCAND để đối chiếu kênh quan sát, lệnh teleop và nhãn tình huống. Bản phát hành SCAND thiếu nhãn tracking người. Giữ provenance theo nguồn khi đối chiếu với kết quả MuJoCo. [NavWareSet](https://anr-navware.github.io/navwareset/) · [Notebook SFM](https://github.com/anr-navware/NavWareSet-SFM) · [SCAND](https://www.joydeepb.com/Publications/ral2022_scand.pdf)

## Thiết kế cặp so sánh

- Khi đo tác động của backend, ghép cặp theo `(scenario_id, episode_seed, robot_policy_id)` và đổi backend. Khi đo tác động của lệnh robot, giữ nguyên backend, cấu hình, trạng thái ban đầu và nguồn nhiễu, rồi đổi nhánh lệnh robot tại mốc đã định. Dùng tiến/dừng cho cặp đầu tiên, thêm né/nhường hoặc lùi ở nơi có hình học phù hợp.
- Kiểm tra phần chạy chung trước mốc phân nhánh theo dung sai đã khai báo. Có thể dùng reset và phát lại chuỗi lệnh chung. Khôi phục trực tiếp giữa episode chỉ dùng sau khi đã kiểm chứng khôi phục cả trạng thái vật lý, backend, RNG và hàng đợi có liên quan.
- Dùng tối thiểu các nhóm scenario có ý nghĩa khác nhau. Hành lang đối đầu kiểm tra nhường lối. Cửa hẹp cắt ngang kiểm tra quyền đi trước và chờ. Giao cắt khuất tầm nhìn kiểm tra tương tác với robot khi cảm biến bị che. Thêm scenario robot đứng yên và người đi thẳng để cô lập hành vi cơ bản khi kiểm định.
- Với mỗi cặp, giữ nguyên world/MJCF, hình học, footprint, bề mặt, mục tiêu, pose và vận tốc khởi tạo, số lượng/ID người và danh sách vật cản. Cố định seed episode, lịch bước, cấu hình cảm biến, tham số cơ cấu, limiter và tiêu chí dừng. So fingerprint tài nguyên, trường cấu hình kiểm soát và snapshot reset. Ghi rõ biến được đổi và các trường định danh, nguồn gốc đi kèm.
- Khi so backend, giữ nguyên robot policy, tham số, nhiệm vụ và seed luồng. Dùng kịch bản lệnh cố định để cấp cùng kích thích từ robot. Với policy phản hồi, robot có thể phát lệnh khác do quan sát khác, nên ghi lại lệnh và diễn giải kết quả như tương tác vòng kín.
- Trước khi có sensor, dùng profile lệnh robot cố định với danh sách sensor rỗng. Khi đối chứng dùng policy phản hồi, kiểm định các kênh cảm biến cần thiết trước khi chạy.
- Cấp cùng phép chiếu `HumanContext` cho mọi backend, với dữ liệu tại tick hiện tại. Backend có thể dùng tập con đã khai báo. Giữ luồng ngẫu nhiên cho spawn, sensor và robot độc lập với số lần rút của backend. Nếu mô hình hóa ngẫu nhiên cảm biến theo vị trí, giữ quy tắc nhiễu và seed, đồng thời ghi nhận phép đo có thể khác do cảnh đã thay đổi.

## Thực thi và số đo

1. Reset theo manifest, kiểm tra điều kiện ghép cặp, rồi cho từng backend đề xuất `HumanMotion` trong `world` tại mỗi control tick. Quy định đến đích, chờ và xử lý actor đã tới đích trong scenario. Actor giữ ID và có lệnh cho mọi tick.
2. Đưa mọi backend qua cùng một limiter vận tốc/gia tốc, cùng adapter, cơ cấu, substep và quy tắc tiếp xúc. Script chỉ phát lệnh chuyển động. Chỉ reset được đặt lại pose; trong episode không dịch chuyển actor tức thời để bám tuyến, phục hồi hay giải va chạm.
3. Ghi `requested`, `limited`, `executed` và pose thực tế. Tính tỷ lệ tick có giới hạn lệnh và độ lệch giữa limited/executed để phân biệt tác động của hành vi đề xuất với cơ chế thực thi.
4. Tính theo episode tỷ lệ đến đích, thời gian đến đích trong episode thành công, va chạm robot–người/người–người/người–vật cản, khoảng hở nhỏ nhất, thời gian đứng chờ và thời gian dưới ngưỡng tiến triển. Với cặp hành động robot, đo thêm chênh lệch quỹ đạo người sau khi phân nhánh. Báo riêng episode bị timeout. Khai báo trước ngưỡng chờ, ngưỡng tiến triển, cách lấy khoảng hở và cửa sổ đo. Kiểm từng loại contact được profile hỗ trợ. Khi dùng hình học để phát hiện giao lấn, báo số đo đó riêng với contact solver. Tuyên bố tương đồng với người thật cần dữ liệu đối chiếu độc lập.
5. Báo số cặp dự kiến, số cặp hoàn tất, phân bố lý do kết thúc và lỗi theo điều kiện. So chênh lệch trên các cặp hoàn tất và trình bày bất định theo đơn vị cặp hoặc cụm scenario. Giữ kết quả lỗi trong bảng tổng hợp theo [giao thức kiểm định](SIMULATION_VALIDATION_PROTOCOL.md).
