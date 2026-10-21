# Kế hoạch tiếp theo: đánh giá hành vi người, tách khỏi nhiễu định vị

Cơ sở: cohort12 lượt seed42/43/44 cho baseline và LF-prob+beam-skip đều5/6 goal thật. SFM thay đổi2/3→3/3, ORCA+3/3→2/3; đây là mẫu nhỏ, không chứng minh cấu hình mới tốt hơn. Giữ AMCL mặc định và dữ liệu hai failure, chưa chọn framework. [Bằng chứng](benchmark/localization_validation/reports/run01/MULTISEED_REPORT.md).

Cập nhật: bước1 đã chạy đủ6 probe CrowdSimPlus; bước2 đã chạy18 probe nhóm SocNavGym. [Báo cáo replay](benchmark/human_response_replay/reports/run01/REPLAY_REPORT.md). Bước3 đã có đủ sáu tổ hợp scene/variant hợp lệ sau đợt bổ sung ba lượt đi bộ: sáu action báo SUCCEEDED, năm lượt đạt sai số thật ≤0.20m. Một lỗi khởi tạo cũ được giữ riêng; QoS nhận AMCL đã sửa bằng tùy chọn trong harness. GUI kéo/zoom và Pause/Play đã được người dùng xác nhận. [Báo cáo Gazebo](benchmark/group_gazebo_demo/reports/recovery01/RECOVERY_REPORT.md). Bước4 chưa bắt đầu tích hợp. Không triển khai World Model, chuyển Isaac Sim, sửa ROS robot source hoặc thêm model học mới.

## 1. Bước triển khai kế tiếp: sáu probe replay cho CrowdSimPlus

Mục tiêu là đo phản ứng người khi nhận cùng đầu vào robot, tránh dùng lỗi AMCL/Nav2 để đánh giá human policy.

- Hai native policies: ORCA+ và SFM; cùng fixture, initial state, map geometry, radius, timestep0.1s và seed42.
- Ba input robot định trước: ở xa làm đối chứng; tiến gần đối đầu; đi ngang cắt hướng. Mỗi trace tối đa20sim s, dùng chung byte-for-byte cho hai policies. Ghi pose/vận tốc ground truth, không đưa AMCL vào human input.
- Chạy offline tuần tự trong runtime CrowdSimPlus có sẵn trước; không chạy Nav2/Gazebo trong giai đoạn replay. Giữ act/step và tham số native; không thêm clipping, tách người hoặc luật nhường để làm đẹp kết quả.
- Trước khi đo, kiểm tra trace không bắt đầu với overlap, đúng frame/clock và đúng geometry. Lưu trace, config/source hashes và mọi outcome.
- Báo cáo thời điểm đổi vận tốc/hướng khi robot tiến gần, khoảng hở signed, overlap và tiến độ đến waypoint. Quỹ đạo phản ứng khác nhau là dữ liệu; không yêu cầu mọi model phải tránh thành công mới coi probe là hợp lệ.

Nghiệm thu harness: đủ6 probe, đầu vào robot giống nhau trong mỗi cặp, parity với native step được kiểm tra, kết quả có thể tái lập trong runtime đã ghim. Không dùng6 probe để xếp hạng thống kê hoặc chứng nhận an toàn. Replay là thử nghiệm riêng, không trình bày hình vẽ replay như camera Gazebo.

## 2. Quay lại SocNavGym: nhóm và trò chuyện

Sau khi contract replay được kiểm định, dùng cùng cách ghi input/output để khảo sát một nhóm3 người ở hai scene đã hỗ trợ: đi nhóm và đứng trò chuyện.

- Tách rõ native SocNavGym và adapter robot-aware đã chỉnh; giữ native làm đối chứng.
- Quan sát giữ nhóm/membership, khoảng cách giữa thành viên, hướng quay và phản ứng robot trước contact; ghi cả overlap.
- Với CrowdSimPlus, cảnh đứng yên chỉ là stationary fixture, không gọi là conversation hoặc group model.
- Formation/dispersal tiếp tục ở demo độc lập; chỉ đưa vào Gazebo sau khi kiểm định lifecycle/ID/geometry riêng. Không giả định adapter nhóm hiện tại đã hỗ trợ phần này.

Nghiệm thu: phân biệt được năng lực native, biến thể adapter và fixture bên ngoài; có trace và hạn chế của từng scene, chưa chọn model chính thức chỉ từ hình ảnh.

## 3. Trình diễn và benchmark lại trong Gazebo

Sau replay, chạy từng backend tuần tự trong cùng nhà xưởng. Giữ robot Mecanum, LiDAR/IMU, controller và baseline AMCL; dùng mode shutdown WSL D3D12 đã kiểm chứng.

Có thể dùng lệnh robot được lập trước trong harness riêng để minh họa phản ứng; phải đo pose/vận tốc Gazebo thực tế. Cùng lệnh không bảo đảm cùng quỹ đạo do tương tác vật lý, nên không gọi đây là replay robot input chính xác. Nếu input thực tế khác nhau, chỉ kết luận trên phép thử kín vòng và công bố độ khác biệt đó.

Ghi hai nhóm kết quả riêng: chất lượng hành vi người và chất lượng điều hướng robot. Nav2 SUCCEEDED phải được đối chiếu với goal error thật; không gộp success với không va chạm. Ảnh/GIF lấy từ camera Gazebo, ghi policy/seed/sim time và tốc độ phát; biểu đồ từ log được gắn nhãn riêng. Kiểm tra renderer NVIDIA, cleanup và khả năng thao tác GUI trong phiên demo.

## 4. ComplexityNav sau cùng

Giữ demo độc lập hiện có làm đối chứng. Chỉ nối Gazebo sau khi adapter contract pose/velocity/ID/reset/clock và phép ánh xạ geometry nhà xưởng đã đạt kiểm định với các backend trước. Bắt đầu population nhỏ, đối chiếu native step và LiDAR, rồi mới tăng mật độ/lối hẹp. Group clustering không đồng nghĩa chủ động giữ nhóm hay hội thoại.

## Điều kiện chọn mô hình

Ưu tiên khả năng đúng mục tiêu (nhóm, đứng tương tác, tránh người/robot), hành vi quan sát được và hạn chế có số đo. Sau đó mới xét chi phí runtime và độ thuận tiện tích hợp. Mỗi đợt có protocol cố định, không thay failure bằng lượt đẹp; chưa mở rộng thêm các ma trận AMCL nếu chưa có câu hỏi chẩn đoán cụ thể.


## Công việc kế tiếp sau đợt này

Chốt nghiệm thu và lưu Git các probe replay, báo cáo Gazebo, cùng bản sửa subscriber QoS; chỉ commit khi người dùng yêu cầu. Để đáp ứng mục tiêu formation, bước kỹ thuật kế tiếp là kiểm định lifecycle/ID/reset/geometry của nhóm SocNavGym độc lập (nhóm hình thành/tách ra), vì adapter hiện tại chỉ có nhóm cố định ba người. Sau kiểm định mới mở rộng Gazebo. ComplexityNav tiếp tục chạy riêng, chưa đưa vào ma trận Gazebo hoặc thay kiến trúc robot.
