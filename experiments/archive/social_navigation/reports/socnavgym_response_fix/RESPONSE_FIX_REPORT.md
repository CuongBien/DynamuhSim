# Sửa SocNavGym giao cắt và dừng phiên khi worker lỗi

Ngày22/10/2026. Baseline9 lượt đã commit **`9ab3c9f`** (`feat: benchmark shared Gazebo scenes and retain SocNavGym crossing failure`). Bản sửa ở experiment riêng; source ROS/Nav2/vendor, các kernel cũ và bằng chứng lỗi cũ giữ nguyên. **Đã kiểm lại ca giao cắt; chưa bật Nav2.**

## Đã sửa và kiểm được gì

**4 lượt Gazebo chuyển động bình thường hoàn tất**:2 giao cắt ở hai session riêng,1 đối đầu,1 lối hẹp; mỗi lượt6 người,radius0.36m,10s,seed42. **2 lượt lỗi chủ động đạt pause interlock**: worker trả lỗi và worker đóng socket, đều tại bước55/t=5.5025s.6/6 scene/control offline hoàn tất600 update và đạt swept geometry/ràng buộc tốc độ.5 test hình học gồm trường hợp bắt buộc đổi hướng, tường, người đối đầu, static human bất khả thi và initial overlap; IPC thật khớp exact20 bước với output offline mới và đóng terminal khi clock sai.

| Lượt Gazebo | Kết quả | Min người–người | Min robot–người | Min tĩnh–người | LiDAR khớp<15cm |
|---|---|---:|---:|---:|---:|
|crossing01|Đủ10s|0.0408m|0.0399m|0.7714m|99.17%|
|fault_error01|Pause đúng khi fault|0.0407m|0.0409m|0.7714m|99.15%|
|fault_disconnect01|Pause đúng khi fault|0.0407m|0.0409m|0.7714m|99.23%|
|opposed01|Đủ10s|0.4762m|0.3901m|1.1735m|99.10%|
|narrow01|Đủ10s|0.7397m|1.3028m|0.3688m|98.94%|
|crossing02|Đủ10s|0.0407m|0.0409m|0.7714m|99.20%|

Bảng giữ cả phần kết thúc phiên, không loại overlap bằng cắt cửa sổ trước lỗi. Signed minima dùng SceneBroadcaster actual poses, closest-relative approach và GEOS segment-to-box độc lập; không lấy command pose làm actual. Ngưỡng gắn nhãn penetration ở live5mm; minima thô lưu nguyên.4cm là target projection, không phải social comfort hoặc bảo đảm khoảng hở thực luôn4cm. LiDAR chỉ tính các tia hợp lệ dự đoán trúng proxy người gần hơn42 boxes, không phải recall toàn cảnh; ID cụ thể lưu từng validation.

![Khoảng hở đo trong Gazebo trước/sau](crossing_gap_before_after.png)

## Nguyên nhân và thay đổi

Baseline lấy một đề xuất ORCA rồi thử33 hệ số giảm đồng loạt vận tốc của6 người; có thể không tìm được phương án dù một người đổi hướng thì vẫn đi được. Diagnostic replay khớp chính xác55 bước robot input đã lưu và human state baseline. Request ở bước gây lỗi trước đây chưa được log, nên nó được dựng lại từ pose Gazebo gần nhất và vận tốc giữa hai mốc; diagnostic ghi rõ không gọi request này là byte-identical. Với request dựng lại đó, legacy tiếp tục lỗi còn projector mới tìm được phương án có swept gap robot khoảng6.02mm, pair21.41mm và tốc độ<=0.8m/s.

Bản v2 giữ native intention và đề xuất ORCA cũ, rồi chiếu vận tốc từng người lên các halfspace phân cách người/robot/vật cản và speed discs bằng Dykstra. Robot displacement được giữ nguyên theo đo thực; không giả định robot nhường và không giảm/ẩn chuyển động robot để làm đẹp kiểm tra. Target endpoint margin4cm; nếu tập ràng buộc bảo thủ bất khả thi, thử2cm rồi2mm; chỉ nhận phương án qua kiểm swept floor2mm và speed bound. Nếu vẫn không có phương án hợp lệ hoặc trạng thái đã overlap, worker lỗi terminal. Không tuyên bố global feasibility của bài toán tránh va chạm gốc.6 ca offline dùng margin4cm; diagnostic trạng thái cũ đã sát robot dùng2cm endpoint target. Margins và iterations từng live step đều được lưu.

Sau lỗi hoặc socket loss, plugin gửi zero cmd_vel và phát Pause ngay trong Gazebo. Nhờ pause world, robot/người không trôi tiếp trong simulation; đây **không phải chứng minh phanh vật lý** hoặc hệ dừng khẩn cấp cho robot thật. Fault là terminal, cần session mới. Trong hai test chủ động, clock đứng yên1.2s wall; khi thử Play, time chỉ tiến1tick=2.5ms rồi lại Pause, actual robot drift từ pose ghi ở fault bằng0 trong các mẫu đo. Không gọi là simulation time tuyệt đối không thể tiến một tick khi bấm Play.

Native dt0.1s, physics0.0025s,42 boxes, giới hạn người0.8m/s và actor yaw1.57rad/s giữ nguyên. Không cài SciPy/system packages hoặc đổi Python ROS. Native/ORCA vẫn dùng CPU, worker chỉ2 CPU/one BLAS thread; renderer/camera/GPU LiDAR đã kiểm NVIDIA D3D12 qua cả6 lượt. Chi phí projection và whole native step có trong validation; không khẳng định kernel chạy realtime chỉ vì rendering dùng GPU.

![Camera Gazebo thật — crossing v2,4×](crossing01/camera_preview_4x.gif)

![Camera Gazebo thật — lỗi chủ động và Pause,4×](fault_error01/camera_preview_4x.gif)

Camera640×480,1frame/s,quantize khi đóng GIF,4×. Các ảnh/GIF lấy từ RGB Gazebo thật, không phải AI. Đồ thị là dữ liệu đo, không phải camera. GUI mở ởcrossing01; chưa có xác nhận kéo/zoom của người dùng riêng cho bản v2. Phần camera của test Pause dừng ởsimtime5s vì simulation không tiến tiếp.

## Giới hạn và bước kế tiếp

Đây là sửa adapter riêng, không phải native SocNavGym mới hoặc chứng nhận toàn bộ framework. One seed,10s mỗi lượt,4 normal sessions; hai giao cắt dùng cùng initial scene/lệnh nhưng startup và actual robot traces khác nhau, không gọi exact replay hoặc statistical success rate. Formation/conversation tắt trong cohort này: capability nhóm của lifecycle cũ vẫn là bằng chứng riêng, chưa chuyển phép đo v2 sang kết luận mọi nhóm đứng sẽ nhường robot. Các static humans vẫn bị giữ đứng và có thể khiến bài toán infeasible; test riêng chứng minh trường hợp đó dừng terminal, không tự dịch nhóm đứng đi. Đối đầu đối xứng có thể dừng/kẹt; projection không bảo đảm thoát deadlock, tính tự nhiên hay goal completion.

Baseline lỗi, attempt target4cm tại legacy close state và harness NameError trước update0 được giữ, không ghi đè hoặc giấu. Source/report cũ, staging và thay đổi ngoài experiment được kiểm bảo toàn. CPU/GPU/cleanup, exact IPC prefix, raw worker requests kể cả fault, actual pose/scan/odom và SDF sinh riêng có hash/gzip. Chưa kiểm timeout worker kéo dài/reset-world trong phiên, repeated-Play spam, Nav2, mật độ cao hoặc robot thật.

**Bước tiếp theo:** chạy pilot Nav2 nhỏ với adapter v2 sau mốc sửa đã kiểm này, robot và goal rõ ràng; kiểm robot chủ động nhường/tránh người trước khi mở rộng benchmark. Không mở rộng sang World Model/Isaac Sim hoặc thay cấu trúc robot.

[Protocol](../protocol.json) · [Diagnostic](diagnostic01/diagnostic.json) · [Offline GEOS audit](offline01/validation.json) · [IPC](transport01/validation.json) · [Từng lượt live](summary.json) · [Lệnh và trạng thái audit](final_audit.json).
