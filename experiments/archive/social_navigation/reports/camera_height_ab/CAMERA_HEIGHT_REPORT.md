# Kết quả A/B chiều cao camera —22/10/2026

**Đã kiểm3cảnh,2chiều cao12/40cm. Cao hơn giúp độ phủ BEV trong các cảnh đã thử, nhưng không loại bỏ che khuất bởi khối trụ người. Không đổi chiều cao robot/camera mặc định.**

Hai camera cùng poseXYyaw/FOV/K/range và cùng10timestamp mỗi cảnh. Robot và6proxy đứng yên; hoạt ảnh người bị giữ animation_time=0 bằng plugin riêng.60ảnh raw (30cặp cùng thời điểm) ổn định bitwise trong từng stack,5unit tests đạt. NVIDIA RTX4060 D3D12 được xác nhận cho cả3server; exit0, cleanup không còn tiến trình. [Audit](run04/audit.json), [số đo/SHA](run04/summary.json), [protocol](run04/protocol.json).

|Cảnh / thời điểm nguồn|Depth hợp lệ12cm /40cm|BEV đã quan sát12cm /40cm|Depth khớp mặt trụ trong pixel hợp lệ12cm /40cm|
|---|---:|---:|---:|
|snapshot01 / 12.6025s|78.5% / 86.8%|11.4% / 12.9%|58.4% / 72.5%|
|snapshot02 / 23.1025s|79.2% / 88.0%|8.1% / 11.5%|62.7% / 77.8%|
|snapshot03 / 33.8025s|61.6% / 68.2%|23.0% / 30.5%|0.0% / 0.0%|

BEV đã quan sát=(free+occupied)/4096; không diễn giải unknown là vật cản và không coi tăng free là tăng chất lượng. Tỷ lệ khớp mặt trụ tính bằng tia camera cắt các cylinder thật của fixture, sai số depth≤2cm; không coi đây là bộ phân đoạn semantic đầy đủ. Floorprojection consistency p99≤3.68mm ở6ảnh đại diện; có thể còn sai khác pixel-center/renderer, chưa chứng nhận calibration robot3D hoàn chỉnh. Không ghi freecell nằm hoàn toàn bên trong proxy tại3cảnh này; không suy thành chứng minh mọi freecell đúng.

## Cảnh sát nhóm: vì sao vẫn hình chữ V?

![Depth/BEV thật: camera12cm và40cm trên cùng cảnh](run04/snapshot02_comparison.png)

Ở cảnh23.1025s, phần BEV biết tăng8.1%→11.5%, nhưng số occupiedcell vẫn20 và mặt trụ khớp62.7%→77.8% pixel depth hợp lệ. Camera cao40cm vẫn thấp hơn nhiều so với trụ1.7m nên không nhìn xuyên/qua phía trên chúng. Đây là bằng chứng vị trí camera có ảnh hưởng, còn proxy là nguồn che khuất rõ rệt; chưa làm ablation loại proxy nên không quy mọi đặc điểm hình ảnh cho duy nhất nó.

Camera thấp có vùng sàn gần bị clip và dễ bị người sát phía trước che. Camera cao thay đổi vị trí giao tia với sàn/mặt trụ; số pixel sàn và diện tích mặt sàn quan sát không phải cùng một đại lượng. BEV một frame chỉ cho phần frustum nhìn thấy, không phải floorplan nhà xưởng. Unknown chiếm nhiều diện tích là hợp lệ; hình chữ V không tự chứng minh transform bị đảo hoặc dữ liệu hỏng.

[Cảnh1](run04/snapshot01_comparison.png) · [Cảnh3](run04/snapshot03_comparison.png) · [Depth float32 nén lossless](run04/representative_depths.npz).

## Thay đổi và bằng chứng

Chỉ world/runtime thử: snapshot positions, robot/proxy static, bỏ drive/sensors không liên quan, thêm hai depth sensors và static TF camera tương ứng, giữ actor animation time0. Thân/footprint và hình học robot giữ nguyên, không có giá đỡ vật lý mới. URDF camera_joint clones đổi riêngz; sensor extrinsics/SDF/URDF/TF đối chiếu bằng assertions. Nav2, human worker, source robot, vendor và dữ liệu pilot cũ không bị sửa. Không đo lại goal, né người hay social comfort trong cảnh camera tĩnh.

Rawdata và generatedworld ở generated/runs/run04; report ghi SHA256 từng file. 6ảnh representative trong Git khớp từng byte với frame thứ6 của rawstack.10ảnh mỗi camera là phép kiểm ổn định một cảnh, không phải10scene độc lập.3cảnh cũng không đủ chọn chiều cao mount cho robot thật.

## Các lượt không nghiệm thu được giữ

- run01: bridge nhận sai tên camera_info; dừng sạch, không đủ cặp ảnh để phân tích.
- diagnostic01: liệt kê native topics để sửa đúng /height/{low,high}/camera_info.
- run02: có paired images, nhưng một actor skeleton vẫn tự chạy; kết quả trung gian có nhãn Frozen không chính xác, nay đánh dấu NOT_ACCEPTED_AS_FROZEN. Không dùng nó làm cohort đạt.
- run03: thử khóa bằng script đơn thuần, không có depth; requeststop không kết thúc server, đã dừng cưỡng bức đúng processgroup của phiên. Cleanup cuối rỗng. Không gọi là cleanexit.
- run04: plugin riêng giữ world trajectory/animation time theo cách adapter cũ, sensorrepeat gate xác nhận hoàn toàn tĩnh;3cảnh đạt, không thay failure cũ bằng kết quả mới.

## Quyết định

Giữ12cm baseline và40cm là variant thử nghiệm. Bước kế là ablation geometry cảm biến có/không proxy, kiểm Depth lẫn LiDAR/collision trước khi sửa tích hợp; cần tránh model học hình người dạng cột như đặc tính thế giới thật. Chưa promote dataset thành train-ready, chưa train World Model. Việc robot xâm nhập giữa nhóm vẫn cần tiêu chí/chi phí ở planner và metric riêng, không được xem đã xử lý nhờ nâng camera.
