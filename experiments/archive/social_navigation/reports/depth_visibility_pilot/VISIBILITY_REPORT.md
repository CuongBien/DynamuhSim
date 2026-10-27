# Kiểm chứng pixel người và che khuất — 23/10/2026

**Đã có bộ đối chứng pixel người trong một cảnh tĩnh. Bộ lọc hình học giữ cả4 cụm người và2 cụm đồ vật; chưa thể coi track là người hoặc bật nhãn train.** [Ảnh đo thật](visibility_review.png), [evaluation](evaluation.json), [acceptance](acceptance.json), [audit](audit.json), [code/lệnh chạy](../README.md).

## Phép thử

Dùng nhà xưởng và mesh người native hiện có, robot/camera thấp nguyên bản, nhưng dựng một fixture tĩnh riêng: sáu người, hai hộp gây che khuất/nhiễu, animation cố định frame0. Không chạy Nav2, bước policy người hoặc chỉnh camera cao lên. Không phải replay nguyên trạng một frame động của seed44.

Trên GPU NVIDIA RTX4060 D3D12,15 camera depth đồng vị trí ghi5 thời điểm2–6s, tổng75ảnh424×240 float32. Các view gồm full, full đối chứng, null-human, lần lượt ẩn từng người và chỉ giữ riêng từng người. Mỗi lần render tạm đổi visibility của visual rồi khôi phục, không thay collision/pose/physics. Sáu actor mesh và sáu proxy được mapping từ scene thật; camera không nhìn proxy hình trụ. Các view này chỉ phục vụ LABEL/evaluation; inference hiện tại chỉ đọc full depth và hiệu chuẩn. Null là phép đối chứng âm riêng.

Một pixel được gán cho người_i khi bỏ người_i làm ray sâu hơn/trống và ray trong view chỉ-người_i khớp full trong1mm. Có mâu thuẫn hoặc nhiều owner thì mask unknown. Không lấy tâm GT gần nhất để tự suy người đang thấy, không gán nhãn cho người đứng sau vật thể mà camera không nhìn thấy.

## Quan sát thực

| Người thiết kế | Pixel nhìn thấy / silhouette riêng trong tầm | Phần silhouette nhìn thấy | Điểm sau stride/crop |
|---|---:|---:|---:|
|1, phía trước|4877/4877|100%|309|
|2, sau người1|342/2870|11.92%|16|
|3, sau hộp|2922/4480|65.22%|194|
|4, bên cạnh đồ vật|2508/4174|60.09%|157|
|5, sau camera|0/0|không áp dụng|0|
|6, ngoài5m|0/0|không áp dụng|0|

Tỷ lệ tính trong field-of-view/range của camera thấp; không có nghĩa thấy toàn bộ cơ thể. Người2 vẫn lộ một phần, vì vậy **chưa có ca thật fully-occluded được nghiệm thu**; nhánh đó mới được kiểm bằng unit test. Không tính người5/6 là bỏ sót detector. Bốn người còn lại đủ tiêu chí≥20pixel và≥5điểm stride/crop, khai báo trước khi xem kết quả.

Cả unfiltered và shape-filtered đều có6proposal:4 cụm thuần người theo pixel ownership (purity≥80%) và2cụm môi trường. Khi ẩn hết người, cả hai vẫn giữ2cụm môi trường. Không tăng threshold, thêm luật theo vị trí fixture hoặc bỏ proposal để làm đẹp số liệu. Nếu gọi mọi proposal là người, diagnostic precision=4/6 (66.67%), recall trong4 người nhìn thấy=4/4; đây không phải performance thống kê của semantic detector. Hiện semantic_class vẫn unknown_geometry_candidate, nên kết luận đúng là **pipeline thiếu bước phân biệt người–đồ vật**, không phải đã có semantic classifier được nghiệm thu.

Static camera1Hz khác dữ liệu tracker10Hz: activeTTL0.5s khiến ID không tích lũy continuity bình thường. Không dùng trường confirmed_nonperson_proposals rỗng để chứng nhận tracker loại được vật thể. Phép thử này đánh giá detection geometry, không re-ID động.

## Lỗi thử nghiệm và thay đổi tiêu chí được giữ lại

- static01: depth có nhưng bridge chờ /info, trong khi native DepthCameraSensor phát /camera_info. Lượt thiếu hiệu chuẩn không được chấp nhận.
- static02: dừng sau khi xác định topic native. ROS tự shutdown khi SIGINT, finalizer gọi lại shutdown gây lỗi nên thiếu final receipt. Có interrupted_recovery.json đánh dấu rõ, raw không đồng bộ được giữ; pause/stop server đã chạy. Không dựng lại một result thành công giả.
- static03: sửa chưa hết literal của bridge, vẫn thiếu info; dừng có kiểm soát, result/cleanup báo lỗi thu và đóng sạch. Đã sửa đồng bộ, thêm contract test.
- static04: đủ75ảnh, full/control khớp bitwise theo từng timestamp, nhưng strict inter-frame bitwise FAIL: tối đa4pixel thay đổi, sai số lớn nhất1.66893e−6m, không đổi finite mask/ownership. Giữ nguyên FAIL, code/protocol1 và dữ liệu.
- Trước static05, protocol2 khai báo ổn định số học≤1mm (đúng tolerance label đã có), finite mask không đổi; vẫn đòi full/control cùng timestamp khớp từng byte. static05 đạt gate số học; strict inter-frame bitwise **vẫn false**, không đổi thành PASS cũ.

Đây là các lượt phát triển cùng một fixture, không phải năm scene độc lập hoặc held-out validation. Không có thay đổi inference/detector/threshold từ kết quả này. Native GPU logs, source từng bản thử và fail giữ trong [attempts](attempts/).

## Nghiệm thu giới hạn

Đã đo10649pixel người/ảnh,0mâu thuẫn,0overlap owner,0pixel ảnh hưởng bởi null-human mà không có owner. Ownership map giữ nguyên cả5timestamp. [Audit độc lập](audit.json) kiểm scalar57070pixel và16histogram proposal,75ảnh/20source hash trước thu/9source hash phân tích; đồng thời xác nhận14985file có sẵn không đổi. [9test](tests.json) chạy trước static05. Actual Ogre NVIDIA, callback balanced, pause→stats đứng→stop exit0, cleanup không còn process của phiên.

Đạt harness LABEL tĩnh theo protocol2; **semantic_person_validated=false, association_accepted=false, persona_validated=false, training_ready=false**. Bộ đối chứng không tự sửa annotation các episode42/43/44, không tạo supervision M1 mới cho cả episode. Nhãn H1–H5 vẫn masked, HuNavSim chưa dùng. Lỗi navigation TIMEOUT4.012m của seed44 không được sửa ở đây.

## File và bước tiếp

Git lưu code, protocol, fixture, fail/source/log, evaluation/audit, ảnh, mask và representative_LABEL_ONLY.npz (15view cùng thời điểm). Đây là dữ liệu đo/thể hiện từ Gazebo, **không phải AI sinh ảnh**. Raw5frame/view nằm trong generated/runs/static05, Gitignore; Git không backup toàn bộ raw. Không đưa NPZ LABEL ONLY vào input của World Model.

Bước kế cụ thể là triển khai thêm semantic person-vs-object từ cảm biến, lấy hai cụm đồ vật này làm regression bắt buộc, rồi kiểm cảnh động/occlusion và một scene mới. Giữ hiện trạng ROS/Nav2 và chưa thu batch/train. Không dùng oracle pixel labels làm detector đầu vào.
