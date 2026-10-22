# Kế hoạch tiếp theo: đánh giá hành vi người, tách khỏi nhiễu định vị

**Trạng thái mới nhất22/10/2026:** [bản sửa formation đã nối Gazebo và kiểm thử](benchmark/socnavgym_warehouse_lifecycle/reports/WAREHOUSE_LIFECYCLE_REPORT.md). Các mục bên dưới giữ tiến trình và quyết định ở thời điểm trước; kế hoạch hiện hành ở cuối trang.

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

Đã lưu ba commit: replay `b3f59e2`, observer AMCL `195f4e8`, Gazebo nhóm/GUI `e749fb5`. [Lifecycle SocNavGym](benchmark/socnavgym_lifecycle/README.md) đã kiểm tra ba seed với4100 update native: ID và reset prefix500 bước đạt, nhưng formation có nhảy vị trí tới32.6cm và overlap proxy hình tròn56–70cm. Quyết định: giữ formation độc lập, không promote vào Gazebo trong milestone này; không sửa policy để che hạn chế. Adapter nhóm cố định vẫn giữ nguyên.

Bước kế tiếp chuyển sang contract/geometry ComplexityNav trước khi nối Gazebo: giữ runtime Python3.8 riêng, native human update và source gốc; kiểm tra robot feedback, ID/reset/clock và toàn bộ collision AABB nhà xưởng bằng fixture nhỏ, rồi native parity và LiDAR. Chưa chạy thêm ma trận AMCL hoặc thay kiến trúc robot. Khi backend đủ điều kiện, chạy bộ cảnh chung và chốt bảng lựa chọn; formation không đạt sẽ được ghi là hạn chế của phương án, không giữ toàn dự án chờ vô hạn.


## Cập nhật sau yêu cầu sửa formation

Đã triển khai [continuous lifecycle adapter riêng](benchmark/socnavgym_lifecycle_fix/README.md), không sửa vendor hoặc robot.5476 update kiểm chứng: ba seed gốc, reset lúc có nhóm và fixture nhóm đi. Giữ vị trí khi chốt formation, giới hạn quay1.57rad/s, vận tốc0.8m/s, global individual-disc ORCA và swept fallback. Các phép đo độc lập không ghi nhận xuyên proxy; formation/dispersal và chuyển động vẫn hoạt động. Native reference được giữ nguyên.

**Ưu tiên tiếp theo vẫn là SocNavGym:** mở rộng kiểm định obstacle nhà xưởng và robot có vận tốc đo thực, rồi nối formation variant vào Gazebo và kiểm tra actor/proxy/LiDAR/reset/clock. Không chuyển ComplexityNav trước khi chốt phần này. Bản sửa hiện mới nhận robot đứng yên trong phòng vuông native, chưa nghiệm thu Gazebo hay hứa hành vi tự nhiên ở mọi cảnh. Các thay đổi lifecycle mới chưa commit.


## Sau pilot formation trong nhà xưởng —22/10/2026

Đã hoàn tất kiểm định obstacle/robot chuyển động và nối adapter riêng vào Gazebo:3 trace offline ×1200 bước,2 lượt live ×120s,6 người,42 AABBs. Seed42 đạt kỹ thuật nhưng chưa thấy tách nhóm; giữ PARTIAL. Seed43 có hai nhóm đứng hình thành/tan và một nhóm đi2.17m rồi tan. Proxy/clock/LiDAR/GPU/shutdown đạt; người dùng xác nhận thấy6 người và kéo/zoom. Chỉ restart session mới được kiểm tra; reset ngay trong world bị từ chối. [Report, hạn chế và camera thật](benchmark/socnavgym_warehouse_lifecycle/reports/WAREHOUSE_LIFECYCLE_REPORT.md).

Phần sửa standalone và tích hợp mới vẫn chưa commit. Trước khi mở milestone khác, lưu riêng các phần đã hoàn tất theo yêu cầu commit của người dùng; bảo toàn staging/code ngoài experiment.

Bước kỹ thuật tiếp theo là kiểm định contract ComplexityNav trong runtime độc lập hiện có: ID/frame/clock/reset, robot feedback và42 collision boxes; so native step trước khi nối population nhỏ vào Gazebo. Chỉ sau khi đạt mới chạy chung các scene để so hành vi, tăng mật độ/lối hẹp và chốt bảng chọn mô hình. Không mở thêm ma trận AMCL. Pilot formation hiện tại không thay thế benchmark Nav2 và không chứng minh tính tự nhiên ở mọi cảnh.


## Sau kiểm định ComplexityNav —22/10/2026

Ba phần SocNavGym đã commit riêng: `2480158` native reference, `779c338` sửa continuous formation, `b3f0022` pilot Gazebo6 người. Phần uncommitted hiện tại là [contract ComplexityNav và báo cáo](benchmark/complexitynav_contract/README.md). ID/frame/clock/pause/reset/native parity đạt ở runtime ghim, nhưng centralized ORCA/SFM đi xuyên kệ thật trong fixture độc lập; gate NOT_READY_FOR_GAZEBO được giữ.

Bước tiếp theo: adapter hình học riêng cho42 AABBs, giữ native đối chứng; kiểm định obstacle/swept disc/robot vận tốc/progress/reset trước khi nối population nhỏ vào Gazebo. Không gọi ledger42 boxes là native avoidance đã hỗ trợ. Sau khi đạt mới đối chiếu proxy/LiDAR rồi chạy các scene chung để chốt framework. Không đổi AMCL mặc định, robot/Nav2 hoặc kiến trúc project.


## Sau adapter geometry ComplexityNav —22/10/2026

Preflight native đã commit `b0b3c33`. [Adapter hình học riêng](benchmark/complexitynav_geometry/reports/GEOMETRY_REPORT.md) đã vượt gate offline:6/6 cảnh,1040 update adapter/960 native reference,42 boxes, mixed6 người/robot chuyển động, đi vòng kệ tới đích và hai người qua khe1.8m; reset80 bước khớp. Native xuyên kệ giữ làm đối chứng. Waypoint routing + ORCA safety projection là biến thể chỉnh hành vi, gồm cả nhánh SFM; chưa được gọi là native hoặc benchmark Gazebo.

Bước kế tiếp: commit riêng phần đã đạt, sau đó pilot Gazebo population nhỏ, timestep native0.25s với endpoint/interpolation đúng clock. Worker lỗi phải freeze/fail-until-restart; đối chiếu proxy/LiDAR/actual robot feedback, pause/owned cleanup/GPU trước khi tăng mật độ hoặc chạy bảng so sánh chung. Không thay robot/Nav2, AMCL mặc định hoặc kiến trúc project.


## Sau pilot ComplexityNav Gazebo —22/10/2026

Adapter offline đã commit `be816c3`; [pilot mới](benchmark/complexitynav_gazebo/reports/GAZEBO_PILOT_REPORT.md) chạy4 người trong Gazebo60s, measured robot tiến/ngang,23 arrival, actual proxy/LiDAR/pause/GPU/GUI/cleanup đạt. Fault injection xác nhận freeze rồi restart session mới chạy tốt. Pilot này chưa commit. ComplexityNav geometry variant hiện có Gazebo integration; trained RGL/native geometry chưa được promote.

Bước tiếp theo: commit pilot riêng, chốt protocol bộ cảnh chung cho3 backend (same warehouse/robot inputs, population/attributes/metrics khai báo rõ, native vs adapter variant tách riêng), rồi chạy tuần tự và chốt bảng lựa chọn. Bao gồm phản ứng robot tiến gần, vòng kệ/lối hẹp và nhóm/conversation đúng năng lực backend; không gọi clustering hay người đứng fixture là social formation. Không dùng local-aisle pilot này để kết luận dense crowd hoặc Nav2 đã đạt, không thay AMCL/controller gốc.


## Sau cohort chung offline —22/10/2026

Pilot ComplexityNav đã commit `c9dffe2`. [36 phép thử common response](benchmark/common_response/reports/run01/COMMON_RESPONSE_REPORT.md) đã hoàn tất,6 repeat-prefix đạt; đầu vào physical discs/targets/robot trace chung, native dt và prediction assumptions khác nhau được công bố. Guarded Soc, CSP ORCA+ và CN geometry không ghi overlap>1mm ở6 case/control; native Soc/CN và SFM giữ các hạn chế đã đo. Formation tắt trong cohort avoidance, không áp số6/6 cho nhóm.

Theo mục tiêu người đi nhóm/đứng tương tác/formation, ưu tiên SocNavGym lifecycle variant đã có Gazebo; CrowdSimPlus ORCA+ giữ baseline tránh người. CN geometry là biến thể phụ, không gọi clustering là social formation hoặc ngầm dùng RGL chưa có checkpoint. Chưa chuyển backend mặc định của robot.

Bước còn lại: commit cohort này, đưa đúng normalized fixture/config vào cohort Gazebo riêng, đoactual pose/LiDAR/robot path; chỉ sau đó so Nav2 cùngscene nếu cần. Không trộn pilot4 người radius0.3m với cohort6 người radius0.36m thành kết quả chung. Không thêm framework, World Model, Isaac Sim hoặc mở rộng ma trận AMCL.
