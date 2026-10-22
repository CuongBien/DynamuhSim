# Benchmark chung Gazebo — kết quả và giới hạn

Ngày22/10/2026. Bộ offline đã commit **`ff79dbf`** (`feat: benchmark shared offline social navigation scenarios`). Cohort này dùng6 người, radius0.36m,42 vật cản nhà xưởng và robot Mecanum thật trong Gazebo; robot nhận cùng lệnh vận tốc theo thời gian, worker nhận pose/velocity đo thực. **Chưa bật Nav2; formation tắt trong cohort avoidance này.**

Đã thực hiện9/9 lượt: **8 hoàn tất**, **1 lỗi terminal**. Không retry hoặc thay lượt lỗi bằng lượt khác. Toàn bộ9 lượt có dữ liệu actual pose/LiDAR/camera, renderer NVIDIA và shutdown/cleanup được kiểm tra. Audit dữ liệu một prefix lỗi không có nghĩa lượt đó đạt nghiệm thu.

| Cấu hình | Cảnh | Thực thi | Gap người–người | Gap robot–người | Gap vật cản | LiDAR khớp<15cm |
|---|---|---|---:|---:|---:|---:|
|SocNavGym guarded|crossing_near|Lỗi tại5.5s|0.0191m|0.0051m|0.7714m|98.80%|
|CrowdSimPlus ORCA+|crossing_near|Đủ10s|0.0568m|0.0397m|0.7714m|99.32%|
|ComplexityNav geometry|crossing_near|Đủ10s|0.0400m|0.0404m|0.7191m|98.96%|
|SocNavGym guarded|opposed_near|Đủ10s|0.4762m|0.3953m|1.1075m|99.02%|
|CrowdSimPlus ORCA+|opposed_near|Đủ10s|0.4800m|0.0400m|1.5882m|99.64%|
|ComplexityNav geometry|opposed_near|Đủ10s|0.4720m|0.0400m|0.7807m|99.40%|
|SocNavGym guarded|blocked_narrow_near|Đủ10s|0.7570m|1.3585m|0.3753m|98.94%|
|CrowdSimPlus ORCA+|blocked_narrow_near|Đủ10s|0.1585m|0.0267m|0.0215m|99.04%|
|ComplexityNav geometry|blocked_narrow_near|Đủ10s|0.4866m|0.6898m|0.0200m|98.58%|

Signed gaps dùng actual SceneBroadcaster poses, closest-relative-approach và GEOS segment-to-box độc lập; không lấy command pose làm actual. Bảng giới hạn0..10s theo phase worker; prefix lỗi dừng tại thời điểm fault. Ngưỡng gắn nhãn overlap là5mm; mọi minima thô vẫn giữ, không phải chứng minh an toàn liên tục giữa mọi physics tick. Nominal robot disc radius0.25m không thay cho mọi chi tiết robot/mesh cơ thể. Lidar chỉ tính các tia dự đoán trúng proxy người gần hơn42 boxes, trong range hợp lệ; tỷ lệ này không phải recall toàn bộ người/cảnh. IDs cụ thể lưu từng validation.json.

![Quỹ đạo đo từ Gazebo](actual_trajectories.png)

## Lỗi SocNavGym phải xử lý trước Nav2

Ở `soc_crossing01`, worker dừng terminal tại5.5025s sau55 update. Bộ33 tỷ lệ của một đề xuất ORCA không tìm được phương án đạt clearance2mm khi giữ vận tốc robot đo thực. Đây là giới hạn của họ đề xuất đang thử, không phải chứng minh mọi chuyển động người đều bất khả thi. [Audit source](soc_failure_source_audit.json) mô tả đúng phạm vi; native/vendor/guard đã đo không được sửa để làm đẹp kết quả.

Đến trước fault, khoảng hở robot–người nhỏ nhất còn **+5.12mm**. Trong phần kết thúc phiên, người bị giữ tại pose cuối trong khi robot tiếp tục trôi; toàn recording có gap **−10.83cm**. Đó là một lỗi vận hành cần sửa, không được loại khỏi báo cáo dù bảng cửa sổ tiền fault còn dương. Recorder hiện kết thúc server khi callback báo lỗi; chưa có xác nhận dừng robot tức thời hoặc bảo đảm khoảng dừng. Dữ liệu này không được dùng làm demo tự chạy an toàn.

![Gazebo camera thật — SocNavGym prefix lỗi,4×](soc_crossing01/camera_preview_4x.gif)

![Gazebo camera thật — CrowdSimPlus ORCA+,4×](crowd_crossing01/camera_preview_4x.gif)

![Gazebo camera thật — ComplexityNav geometry,4×](complex_crossing01/camera_preview_4x.gif)

Camera lưu1frame/s,640×480, quantize khi đóng GIF, phát4×. Đây là RGB camera Gazebo đã chạy, không phải AI hoặc hoạt cảnh tự dựng. Hình quỹ đạo ở trên là đồ thị dữ liệu đo, không phải ảnh camera. GUI được mở ở lượt Soc crossing; chưa có xác nhận thao tác GUI riêng cho cohort này. Các xác nhận kéo/zoom các pilot trước giữ riêng.

## So sánh được đến đâu

Ba cấu hình dùng cùng initial fixture/targets/radius/geometry và velocity script cho3 near scenes. Native dt0.1s(Soc/CSP),0.25s(CN), native intended speed/observation buffers khác nhau. Worker mới dùng nguyên lớp Runner offline đóng băng, không chép lại thuật toán; IPC thật đã khớp exact20 bước đầu dữ liệu offline cho cả3 và đóng terminal khi clock sai. Một test harness đầu tiên thiếu omega bị worker từ chối trước update0, giữ ở transport_attempt01; đã sửa request test, không sửa policy. CLI transport sau đo chỉ thêm tên output mới.

Gazebo có khởi động/acceleration/communication latency: cùng lệnh vận tốc không có nghĩa robot đi đúng trajectory offline hoặc robot paths ba nhánh giống hệt. Sai lệch lớn nhất giữa actual XY và scripted reference được lưu; kết quả near/far offline không chuyển trực tiếp thành kết luận nhân quả tại Gazebo. Trong cohort này không có far controls, multi-seed, long-run deadlock, hay navigation success benchmark.10s không yêu cầu tới goal xa. Robot đi open-loop và không nhường chủ động; actor vẫn dùng walk clip, không chứng nhận gait/hội thoại tự nhiên.

Min khoảng hở dương vài cm không đủ chọn mô hình tự nhiên nhất. Khả năng nhóm đi/đứng tương tác/formation của SocNavGym dựa pilot lifecycle trước;6 người trong cohort này đi cá nhân. CrowdSimPlus ORCA+ giữ làm baseline tránh người. ComplexityNav geometry là native mixed intent + roadmap/ORCA projection, không phải native solver dùng42 boxes hay trained RGL. Không bật World Model hoặc thay kiến trúc robot.

## Quyết định và việc tiếp theo

**Giữ SocNavGym làm ứng viên chính cho nhóm, nhưng chưa nghiệm thu bản này cho giao cắt sát robot/Nav2.** CrowdSimPlus ORCA+ tiếp tục làm đối chứng độc lập. Giữ ComplexityNav geometry làm lựa chọn nghiên cứu phụ. Bước cần làm trước Nav2: bổ sung proposal né robot cho SocNavGym trong phiên bản mới và xử lý dừng robot khi worker fault, tái hiện đúng ca5.5s để đo lại; không ghi đè baseline này. Sau đó mới chạy Nav2 trên scene thống nhất, không mở rộng sang Isaac Sim/World Model trong bước hiện tại.

[Protocol](../protocol.json) · [9 lệnh và trạng thái](execution.json) · [Toàn bộ số đo](summary.json) · [IPC exact-prefix](transport01/validation.json). Mỗi report con chứa source/input hashes, result/validation, worker log và observations gzip byte-identical, SDF sinh riêng, ảnh/GIF. Các file source/ROS/Nav2/vendor và evidence cũ được kiểm hash; staging của người dùng được giữ.
