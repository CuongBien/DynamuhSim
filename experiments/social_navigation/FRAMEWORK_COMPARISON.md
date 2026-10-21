# Phạm vi đã kiểm tra: demo độc lập và tích hợp Gazebo

Bước kiểm tra tiếp theo là [pilot localization riêng](benchmark/localization_trials/README.md), không thêm simulator mới hoặc thay native hành vi người. [Workaround shutdown](benchmark/shutdown_fix/README.md) chỉ giải quyết vòng đời Gazebo trên WSL D3D12 đã thử.

Đây là bảng so tính năng quan sát được ở các version cụ thể, không xếp hạng
hiệu năng, độ an toàn hoặc mức tự nhiên của hành vi người.

| Mục tiêu | SocNavGym vòng 1 | CrowdSimPlus vòng 2 | ComplexityNav vòng 3 |
|---|---|---|---|
| Nhóm đi có membership | Đã quan sát | Chưa có model tương đương trong bản thử | Clustering nhận diện, chưa có luật giữ nhóm |
| Nhóm đứng trò chuyện | Scene bố trí người quanh tâm, hướng vào nhau | Chưa có model tương đương | Static agents, chưa phải hội thoại |
| Hình thành/tan nhóm | Đã ghi sự kiện | Chưa có model tương đương | Phân cụm quỹ đạo, không phải tạo/tan nhóm theo ý định |
| Tránh người, phản ứng robot | Người đi riêng có phản ứng; nhóm chưa chủ động tránh robot | ORCA/SFM; đã thử visibility và choke point | ORCA/SFM/mixed policies; đã thử visibility |
| Crowd/lối hẹp | Chưa benchmark riêng đủ | Có cảnh hallway/bottleneck và case kẹt | Có width/density/directionality/policy mixture |
| Runtime đã chạy | Python 3.12 venv | Python 3.8.13 container | Python 3.8.13 container |
| Điều khiển người trong Gazebo | Adapter thử nghiệm 1 nhóm 3 người, native/modified variants | Đã chạy 3 người ORCA+/SFM qua Docker, cùng Mecanum/Nav2 | Chưa |

Bằng chứng: [vòng 1](reports/ROUND1.md), [vòng 2](round2/reports/ROUND2.md),
[vòng 3](round3/reports/ROUND3.md).

Nhà xưởng 60×40m + Mecanum/Nav2 đã chạy trong Gazebo. SocNavGym hiện có thêm [adapter một nhóm 3 người](gazebo_adapter/README.md), với hai mode đi cùng nhau và đứng trò chuyện. CrowdSimPlus đã có [backend Gazebo riêng](crowdsimplus_adapter/README.md), dùng native human ORCA+ kernel trong Docker Python 3.8. ComplexityNav vẫn chỉ ở demo độc lập. Adapter chưa đưa formation/dispersal vào Gazebo và đã có [pilot Nav2 ghép cặp RVO2/SocNavGym](benchmark/reports/BENCHMARK_REPORT.md).

[Kiểm định adapter](gazebo_adapter/reports/ADAPTER_REPORT.md) xác nhận simulation clock, pause/resume, robot feedback, pose proxy và phản xạ LiDAR. Giới hạn đáng chú ý: native ORCA nhóm không thêm robot vào phép tính tránh; probe gần/xa cùng seed cho quỹ đạo giống nhau khi chưa tiếp xúc. Cơ chế contact native có thể dừng nhóm khi overlap với robot. Lượt tuần tra dài còn ghi 199/440 trạng thái overlap giữa thành viên, clearance cặp nhỏ nhất −0.354 m. Vì vậy kết quả reactive của người đi riêng trong vòng 1 không chứng minh group walking đã chủ động tránh robot.

Không so trực tiếp tỷ lệ thành công hay clearance giữa ba vòng vì khác môi trường, controller, timestep và điều kiện sampling. Vòng 3 có thêm screening ban đầu; benchmark chung phải áp dụng cùng kiểm tra chất lượng cảnh cho mọi backend.

Pilot gồm 9 goal đạt, nhưng SocNavGym counterflow seed 43 có overlap với footprint Nav2 khoảng 2.0 s, sâu nhất 0.142 m; seed 42 có overlap giữa người trên 1 cm khoảng 0.8 s. RVO2 không có overlap trên 1 cm trong mẫu đo của pilot. Đây chưa phải xếp hạng thống kê. Sau pilot đã có [chẩn đoán seed 43](benchmark/diagnostic/reports/run01/DIAGNOSIS_REPORT.md), [normalization geometry](benchmark/geometry_comparison/reports/run01/GEOMETRY_REPORT.md) và [robot-aware group adapter](benchmark/avoidance_comparison/reports/run01/AVOIDANCE_REPORT.md). Cần ghi rõ sự khác nhau giữa tránh robot của người đi riêng và của nhóm; mọi chỉnh sửa hành vi được gắn nhãn adapter variant, không trình bày như upstream native. Sau khi lượt đó ổn định mới tăng mật độ và thử bottleneck. Không chạy hai backend điều khiển cùng người.

## CrowdSimPlus trong Gazebo

[Report 7 lượt mới](benchmark/crowdsim_comparison/reports/run01/CROWDSIM_GAZEBO_REPORT.md): 4 CrowdSimPlus visible, 2 SocNavGym robot-aware adapter và 1 CrowdSimPlus invisible-robot control, tất cả đến đích. Hai CrowdSimPlus counterflow visible không có overlap robot–người/người–người trong sample đã ghi, robot clearance min khoảng 14,6/13,2 cm. Blind control có overlap sâu; goal success không đồng nghĩa an toàn.

Comparator SocNavGym ở vòng này là bản **modified robot-aware group adapter**, không phải native. CrowdSimPlus vẫn không có membership/cohesion/conversation; stationary fixture chỉ giữ người đứng yên. Runtime CrowdSimPlus hiện khá chậm với 168 static segments, RTF khoảng 0,52; khác biệt timing và các lượt Nav2 chưa được khóa chung noise không cho phép xếp hạng framework từ vài con số.

Cả hai backend dùng cùng initial fixture, map, robot, Nav2 và timestep 0,1 s (CrowdSimPlus original demo là 0,25 s). Source/runtime native được ghim, wrapper parity kiểm tra sai số 0 qua 30 bước. ComplexityNav chưa đưa vào Gazebo. Chưa chọn model người chính thức hoặc thay kiến trúc robot/World Model.

## CrowdSimPlus SFM trong Gazebo

[Report SFM/ORCA+ cùng fixture](benchmark/sfm_comparison/reports/run01/SFM_GAZEBO_REPORT.md):
đã chạy demo SFM upstream trước, kiểm định parity sai số 0, rồi 4/4 goal Gazebo
đạt và không ghi nhận overlap trên 1 cm trong sample. SFM giữ khoảng hở robot
25,67/28,27 cm, ORCA+ 13,61/13,72 cm ở hai seed. **SFM không hoàn tất patrol**:
ba người dừng cách waypoint chung khoảng 0,616 m và không đổi goal, còn ORCA+
tiếp tục tuần tra. Không dùng số này xếp hạng policy hoặc gọi là hội thoại.

SFM vẫn là người độc lập, không giải quyết group walking/conversation/formation.
Bước kế cần fixture có đích riêng cho từng người, dùng chung cho SFM/ORCA+,
rồi mới thử giao cắt/lối hẹp. ComplexityNav vẫn chỉ ở demo độc lập.

## Fixture đích riêng: đã kiểm tra tuần tra trong Gazebo

[Report đích riêng](benchmark/individual_goals/reports/run01/INDIVIDUAL_GOALS_REPORT.md):
4/4 goal Nav2 đạt, mỗi người hoàn tất 2–4 chặng, 39/39 arrival events khớp pose
Gazebo và không ghi nhận overlap >1 cm trong sample. Chỉ đổi endpoint của mỗi
người, giữ initial poses, native force/arrival rule, map, robot và Nav2.

SFM đã tiếp tục tuần tra ở hai seed mới. Shared-endpoint control với robot cố
định cho 1/0/2 chặng, nên không khẳng định goal chung luôn làm cả ba kẹt.
Không có group/conversation model, chưa xếp hạng tính tự nhiên. Bước tiếp:
giao cắt rồi lối hẹp; ComplexityNav vẫn chưa nối Gazebo.

## Giao cắt SFM/ORCA+: đã chạy Gazebo

[Report crossing](benchmark/crossing/reports/run01/CROSSING_REPORT.md): kiểm định
native trước rồi 4/4 goal Gazebo/Nav2 đạt, mỗi người cắt qua tim lối 4–9 lần.
Không overlap >1 cm trong mẫu pose; 78/78 arrival events khớp proxy Gazebo.
SFM robot gap min 21,9/25,4 cm, ORCA+ 11,8/11,9 cm; không dùng bốn lượt để
xếp hạng tính tự nhiên. Nav2 vòng xuống phía dưới khu người đi qua lại.

Crossing có initial poses và endpoint mới, cùng fixture trong mỗi cặp policy;
giữ native parameters, map, robot và Nav2. Đây là ba người độc lập, chưa
có group/conversation. Bước tiếp theo là lối hẹp thật trong nhà xưởng.
ComplexityNav vẫn chưa nối Gazebo.

## Lối hẹp nhà xưởng: đã chạy, còn nhiễu định vị

[Report bottleneck](benchmark/bottleneck/reports/run01/BOTTLENECK_REPORT.md):
empty control qua khe, SFM/ORCA+ mỗi policy 1/2 goal đạt. Seed42 cả hai bị
Nav2 hủy goal; AMCL max error 2,23 m ở ORCA+, 0,45 m ở SFM, pose ước lượng
nằm vào khối máy. Seed43 cả hai qua hết khe thật và đến đích.

Cả ba người qua ít nhất một lần ở mọi lượt, nhưng SFM tiến sát tường và có
overlap cặp người 1,33 mm ở seed42. Không ghi overlap >1 cm trong sample.
Giữ native parameters/controller; chỉ đổi spawn/goal và AMCL initial pose
trong runtime config copy. Cần chẩn đoán định vị/scan/costmap trước khi dùng
kết quả lối hẹp để chọn model. Đây vẫn là ba người độc lập; ComplexityNav
chưa nối Gazebo, chưa có World Model hoặc chuyển Isaac Sim.

## Sau bottleneck: đã khoanh vùng yếu tố định vị

[Chẩn đoán offline](benchmark/localization_diagnosis/reports/run01/LOCALIZATION_DIAGNOSIS.md)
xác nhận ORCA+ seed42 có AMCL jump 1,992 m, trong khi odometry khớp Gazebo
trong khoảng 1 cm. Có mode sensor score cao trong khối occupied, đã kiểm
với thư viện AMCL native. Đây chưa phải replay PF hoặc bản sửa hành vi.
SFM seed42 dừng tịnh tiến trước khi tâm AMCL vào occupied; cần thu global
plan/costmap và particle cloud để tách tiếp. Kết quả 2/4 goal giữ nguyên.

Bổ sung [capture global navigation/particle cloud](benchmark/navigation_diagnostics/reports/run03/NAVIGATION_DIAGNOSTICS.md): SFM/ORCA+ seed42 mới đều qua khe và tới đích, chưa tái hiện failure cũ; không đổi kết quả ma trận gốc hoặc dùng để xếp hạng mô hình.

[Loạt lặp seed42 sau cleanup](benchmark/stability/reports/run01/STABILITY_REPORT.md): SFM2/3 và ORCA+1/3 goal; một mode switch AMCL được xác nhận từ cloud, SFM có proxy overlap2,65cm. Không dùng số lượt nhỏ bị ảnh hưởng định vị để chọn framework hoặc kết luận safety.
