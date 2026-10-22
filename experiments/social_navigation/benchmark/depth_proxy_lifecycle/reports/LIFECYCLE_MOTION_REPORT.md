# Đóng Gazebo và pilot cảm biến động — 22/10/2026

**Đã làm:** bổ sung đối chứng tĩnh thứ ba và thử1lượt robot Nav2 tới nhóm đứng trò chuyện, camera depth có/không trụ cùng pose và timestamp. **Chưa nghiệm thu điều hướng động hoặc sửa dứt điểm lỗi treo cũ.** Robot/ROS/Nav2/native human policy và dữ liệu pilot cũ giữ nguyên.

## Lifecycle và cảnh tĩnh còn thiếu

- diagnostic01 có helper backtrace cục bộ: đóng sạch, không thu được stack treo. Đây là lượt chẩn đoán, không dùng làm bằng chứng sửa lỗi.
- control01 nguyên bản, không helper: cũng đóng sạch. Lỗi cũ run07/run08 không tái hiện trong lượt này; không suy ra nguyên nhân hoặc tuyên bố đã sửa.
- paused01 dùng trình tự mới: yêu cầu pause, nhận WorldStatistics paused cùng một simulation timestamp, đợi0.5s rồi gọi stop_server theo partition. Baseline, masked và null_visuals của snapshot03 đều exit0/cleanup rỗng; đều NVIDIA.
- Hai camera12/40cm ở cảnh không có người trong phía trước: ON/OFF/null **bitwise bằng nhau**. LiDAR chẩn đoán original/tagged bằng bitwise,55 định nghĩa collision bằng nhau. [Đối chứng tĩnh](static03/summary.json).

Như vậy đã bổ sung phép kiểm trống phía trước bị thiếu trước đây, với shutdown khác được ghi rõ. Hai cảnh trước vẫn thuộc mốc static02 cũ; không viết lại lịch sử thành một run9fixture đồng nhất thành công. Pause0.5s là biện pháp giảm lỗi qua kiểm tra, **không phải GPU fence**. Chưa có hung stack hoặc nguyên nhân driver; chưa khẳng định root cause fixed.

## Pilot động motion01

Dùng robot và Nav2/MPPI/AMCL hiện có, start(−28,0), goal(−20,0), SocNavGym standingv3 seed42 với6người. Giữ camera gốc offset(.12,0,.08) so với base_link, không chọn chiều cao40cm mặc định. Chỉ runtime copy thêm camera ON đồng pose và filter OFF từ binary cũ; giữ LiDAR/collision. [Diff driver](motion_driver.diff) và [implementation dùng khi chạy](implementation/motion_driver.py) cho phép kiểm phạm vi.

| Nội dung | Kết quả |
|---|---:|
|Cặp depth float32 đồng timestamp|153 (306 ảnh)|
|Cặp trong active phase|68; cả68 có khác biệt ON/OFF>100pixel|
|Callback OFF pre/post|153/153; teardown cân bằng|
|Robot dịch chuyển thực|2.662m|
|Người1 / người3 dịch chuyển tối đa|4.09cm /11.26cm; người còn lại đứng|
|LiDAR / IMU trong MCAP|77scan /1520mẫu|
|Ảnh OFF khớp raw MCAP bitwise|153/153|
|Nguồn GPU|D3D12 NVIDIA RTX4060Laptop|
|Shutdown có pause|STOPPED; Gazebo clean exit; cleanup rỗng|
|Điều hướng tới goal|**Chưa hoàn thành; terminal human adapter fault**|

Ảnh dưới được lấy từ GPU depth ở11.9s; trên camera ON, dưới camera OFF, bên phải là BEV tính từ từng ảnh. Không phải hình AI hoặc World Model dự đoán. Preview observer RGB/GIF cũng ghi trực tiếp từ Gazebo.

![Cặp depth động thật](motion01/pair_1.png)

![Preview observer thật, lượt dừng sớm](motion01/camera_preview_4x.gif)

Camera ON còn thấy trụ trong đoạn đang di chuyển:17,227mẫu dự đoán mặt trụ phẳng tại hàng115 cho median sai số2.46mm,99.0%trong3cm. LiDAR:5,917mẫu dự đoán đĩa người cho median7.82mm,99.7%trong5cm. Đây là mô tả với noise gốc, vật che và pose gần nhất (ảnh lệch tối đa2.5ms), **không phải đối chứng LiDAR động bằng bitwise hoặc chứng nhận per-frame geometry tuyệt đối**. Đối chứng LiDAR bằng bitwise chỉ có ở các cảnh tĩnh.

[Audit độc lập](audit.json) đọc16topic MCAP, so153ảnh OFF với NPY gốc bitwise, kiểm306SHA ảnh, callback, collision robot và process. [Số liệu prefix](motion01/summary.json), [raw manifest](motion01/raw_manifest.json), [ledger mọi lượt](attempts.json). Header/camera intrinsic gốc được giữ, chỉ dùng sửa metadata runtime như pilot cũ. Chưa chứng nhận contact dynamics, social comfort hoặc association.

## Vì sao dừng

Tại15.2025s, robot tiến về người1 với vận tốc theo đường nối tâm≈0.336m/s, khoảng hở hai đĩa danh nghĩa chỉ4.47mm. Với dt0.1s và guard2mm, ràng buộc projection đòi người1 tránh theo pháp tuyến≈0.311m/s, vượt cap0.20m/s của nhóm đứng. Cả target margin4cm/2cm/2mm đều không khả thi; worker báo fatal và adapter yêu cầu world pause. [Trạng thái chính xác](motion01/failed_worker_request.json), [phân tích support](motion01/feasibility.json).

Test offline gọi lại projection gốc với chính trạng thái đã thu và chỉ một obstacle rất xa: vẫn từ chối vì ràng buộc tốc độ. Vì vậy không cần đổ lỗi cho tường để giải thích rejection này. Test đầu với danh sách obstacle rỗng gặp giới hạn min() của helper cũ; đổi sang obstacle rất xa, không sửa mã cũ hay nới tiêu chí, rồi test đạt. Test tái hiện đúng lỗi **không có nghĩa lỗi đã sửa**.

Renderer đã chạy/đóng sạch, nhưng không có đối chứng robot trajectory khớp để quy nguyên nhân đường robot cho filter hay loại hoàn toàn ảnh hưởng timing của camera thêm. Không đẩy cap người lên cao hoặc hạ clearance để buộc đạt. Vấn đề tiếp theo là robot cần giảm tốc/tránh nhóm sớm, thay vì luôn buộc người nhường khi đã sát.

Episode được [quarantine](motion01/quarantine.json): chỉ dùng phân tích cảm biến và replay failure. Không có action_end/future-label tail; không phải episode train/benchmark success, không gán H1–H5. Pilot dataset cũ vẫn giữ nguyên.

## Phạm vi lưu và bước tiếp theo

Lưu nguồn thí nghiệm mới, ảnh/GIF thật, float32 đại diện, SHA raw và trạng thái lỗi. Không áp filter/shutdown mới vào cấu hình ROS mặc định. Chưa triển khai World Model/Isaac Sim hay thêm framework. Bước kế cụ thể là replay interval lỗi và kiểm một biến thể planner riêng để robot chậm/tránh nhóm sớm; sau đó mới chạy lại1episode hoàn chỉnh rồi kiểm recorder. Theo dõi lifecycle riêng, không gọi biện pháp pause là sửa dứt điểm driver.
