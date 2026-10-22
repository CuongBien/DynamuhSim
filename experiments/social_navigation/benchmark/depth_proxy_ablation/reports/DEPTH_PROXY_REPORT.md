# Kiểm khối trụ người trong depth — 22/10/2026

**Kết luận có giới hạn:** đã kiểm chứng ẩn riêng visual trụ khỏi camera depth ở **2 cảnh tĩnh**, đồng thời giữ ảnh depth đối chứng, hình học LiDAR và55 định nghĩa collision. **Chưa nghiệm thu đủ3 cảnh dự kiến, shutdown tổng thể, điều hướng động hoặc dữ liệu train.** Không đổi cấu hình robot/ROS/Nav2 hiện tại.

## Thiết kế và dữ liệu thật

Hai bộ tọa độ ghi từ pilot SocNavGym standingv3 tại12.6025s và23.1025s, mỗi cảnh6 người. Ba điều kiện: baseline trụ gốc; masked gồm hai cặp camera ON/OFF cùng pose/timestamp; null_visuals bỏ toàn bộ visual người làm đối chứng âm nhưng giữ collision. Camera12/40cm,424×240,float32,10Hz;10frame/camera, hai LiDAR5Hz với5scan/sensor. Một LiDAR giữ noise Gaussian gốcσ=1cm; một bản chẩn đoán bỏ noise để đối chiếu hình học.

Phạm vi đạt:6 fixture,160 ảnh depth,30 scan LiDAR gốc và30 scan chẩn đoán. Cả6 dùng D3D12 NVIDIA RTX4060Laptop, exit0, cleanup không còn process. Actor được giữ frame0 của mesh walk, không phải animation trò chuyện; không bước chính sách người, không lệnh Nav2. Ảnh dưới được vẽ trực tiếp từ GPU depth thật; không dùng AI sinh ảnh, không phải ảnh RGB hay dự đoán World Model.

Nguồn ghép minh bạch: `generated/accepted_static02/snapshot01` sao byte từ run06; snapshot02 từ run07. Đây không phải hai run hoàn chỉnh thành công. [lineage](static02/lineage.json) và [mọi lượt thử](static02/attempts.json) giữ SHA và lỗi; raw đầy đủ nằm local trong `generated/runs/`, không ghi đè. [Audit độc lập](static02/audit.json) kiểm76 file raw,16 ảnh float đại diện bitwise, dữ liệu đối chứng, collision và process.

## Số đo

| Cảnh | Camera | Pixel đổi ON→OFF/frame | Pixel người gần hơn null/frame | BEV known ON→OFF | Ô free trong đĩa proxy OFF |
|---|---:|---:|---:|---:|---:|
|01|12cm|46,558|22,242|11.4%→28.4%|42|
|01|40cm|63,738|28,642|12.9%→36.4%|39|
|02|12cm|46,176|19,324|8.1%→26.2%|34|
|02|40cm|65,625|30,616|11.5%→29.5%|25|

Pixel đổi tính chênh>2cm hoặc đổi trạng thái hữu hạn. Bằng chứng mesh dùng pixel OFF gần hơn đối chứng không có visual người>2cm. Known gồm free+occupied trên4096 ô, không phải thước đo chất lượng hoặc an toàn. [Số liệu đầy đủ](static02/summary.json).

Ảnh camera40cm, cảnh01: trên giữ trụ, dưới chỉ ẩn trụ trong depth; cùng extrinsic, tọa độ người và phép dựng BEV.

![Depth và BEV thật, ON/OFF trụ](static02/snapshot01_high_comparison.png)

- Camera ON trong world tagged **bitwise giống baseline** ở cả4 cặp cảnh/chiều cao.
- LiDAR chẩn đoán baseline/masked **bitwise giống nhau** ở cả2 cảnh. LiDAR gốc không được yêu cầu bằng bitwise vì giữ noise: độ lệch chuẩn hiệu hai lượt≈1.42cm, max≈4.43–4.47cm.
-55 định nghĩa collision, pose và static flag bằng nhau giữa3 điều kiện. Đây là kiểm cấu trúc; **chưa test tiếp xúc vật lý động**.
- Depth OFF biến thiên lặp tối đa14.2µm, finite mask không đổi, dưới ngưỡng khai báo100µm. Ba test hồi quy đạt, gồm từ chối dữ liệu đo đủ nhưng phải SIGKILL khi đóng.

## Các lỗi giữ nguyên, không biến thành PASS

| Lượt | Kết quả / lý do loại |
|---|---|
|run01–02|Dùng bit0x10000000 dành riêng selection: cả hai depth camera đều bỏ trụ; setup không hợp lệ.|
|run03|Đổi sang bit người dùng0x4 giữ đối chứng/LiDAR, nhưng camera visibility_mask SDF không tạo khác biệt ON/OFF trên stack này. Analyzer FAIL; [đánh dấu không nghiệm thu](run03/NOT_ACCEPTED.md).|
|run04|Plugin đầu liên kết apt OgreNext2.3.1 khác runtime ROS vendor2.3.3; abort trước callback. Không vá thư viện hệ thống.|
|run05|Filter đã có hiệu lực, reference/LiDAR bằng nhau; OFF lặp lệch14.2µm vượt ngưỡng ban đầu10µm. Giữ ERROR; khai báo ngưỡng100µm cho lượt mới, không nâng ngưỡng hình học/clearance.|
|run06|Cảnh01 đủ3 điều kiện đạt; mở baseline cảnh02 lỗi D3D12 device removal/GLXBadFBConfig.|
|run07|Chạy bù cảnh02/03, nghỉ3s giữa các context. Cảnh02 đạt; masked cảnh03 đo đủ nhưng đóng cần SIGKILL, exit−9.|
|run08|Chỉ thử lại cảnh03 một lần; masked vẫn exit−9 sau callback teardown cân bằng46/46. Dừng retry, cảnh03 bị loại, không thu null của cảnh này.|

Không kết luận filter là nguyên nhân hay đã sửa lỗi shutdown: log teardown cân bằng chỉ loại được giả thiết callback chưa restore, chưa xác định nơi Gazebo/driver đang treo. Capture mới ghi trạng thái MEASURED_UNCLEAN_SHUTDOWN cho trường hợp này; raw cũ giữ nguyên trạng thái gốc và được loại bằng exit code. Planned three-scene cohort_complete=false.

## Kỹ thuật và khả năng tích hợp

Ubuntu24.04/Python3.12.3/ROS2 Jazzy chạy được thí nghiệm headless này với SDK vendor phù hợp. Phải cấu hình build sau khi source ROS Jazzy; lưu [ldd](static02/ldd_jazzy.txt). Bit0x4 tránh reserved flags. Chỉ SDF visibility_mask chưa đủ trong Ogre2 depth pass đang cài; plugin riêng dùng public Ogre Camera Listener, pre-render ẩn6 proxy visual, post-render khôi phục, chỉ đăng ký hai camera OFF. SceneUpdate/RenderTeardown chạy phía render; không sửa ECM/collision/chính sách người. LiDAR và camera ON là các đối chứng trực tiếp của tác dụng phụ.

Tài liệu gốc: [SDF camera visibility mask](https://sdformat.org/spec/1.11/sensor/), [SDF visual flags](https://sdformat.org/spec/1.12/visual/), [gz-sensors DepthCameraSensor](https://raw.githubusercontent.com/gazebosim/gz-sensors/gz-sensors8_8.2.2/src/DepthCameraSensor.cc), [Ogre2DepthCamera](https://raw.githubusercontent.com/gazebosim/gz-rendering/gz-rendering8/ogre2/src/Ogre2DepthCamera.cc). Kết luận mask không có hiệu lực dựa trên đo ở stack hiện tại; không khái quát cho mọi phiên bản Gazebo.

## Giới hạn và bước kế

Depth nhìn được khe giữa chân trong khi LiDAR/collision vẫn là trụ bảo vệ, nên ghi rõ geometry của từng modality và không suy free BEV thành vùng robot được đi. Chưa sửa thuật toán ray carving, association, nhãn H1–H5, social cost hoặc chuyện robot chen vào giữa nhóm. Nâng camera không tự giải quyết các việc này; World Model cũng không tự sửa planner.

Mốc này giữ thành **thử nghiệm hiển thị tĩnh có giới hạn**, không tích hợp mặc định và không thu train lớn. Bước kế cụ thể là chẩn đoán lifecycle của cảnh03 rồi kiểm một pilot động riêng của filter; sau khi đạt mới đưa vào recorder thử nghiệm và tiếp tục schema/association/persona của dataset. HuNavSim vẫn giữ test cuối; không triển khai World Model/Isaac Sim trong mốc này.
