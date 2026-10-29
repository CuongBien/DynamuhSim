# Xem mô phỏng, depth và classifier — 29/10/2026

Đã chạy **một lượt mô phỏng mới**, standing seed45, recipe v3, Nav2 điều khiển robot. Robot đạt đích thật: sai số XY **0,070m**, yaw **0,0455rad**. GPU D3D12 NVIDIA RTX4060; không lỗi worker, shutdown sạch. Audit capture PASS: 850 cặp depth đã lưu khớp raw bitwise, raw tổng 851 frame. Không append vào dataset01 hoặc đổi model/nhãn/test.

Người dùng xác nhận nhìn thấy Gazebo và kéo/zoom được; báo chạy khoảng 30%. Đo đoạn active: 69,07s mô phỏng / 214,74s wall ≈0,322×. Lượt này chạy đồng thời Nav2, depth ON/OFF, MCAP và GUI. Sau khi kết thúc đã mở pose replay của chính lượt mới, bỏ Physics/Nav2/sensor/recorder, đo RTF khoảng1,004×. Đây là xem quỹ đạo đã ghi, không phải tăng tốc simulator vật lý đã nghiệm thu.

## Kết quả xem

- [Video camera mới](simulation_4x.webm): 69 frame RGB Gazebo thật, phát4×, không ảnh sinh.
- [Depth mới ON/OFF](new_depth_comparison.png): hai timestamp20/35s, 424×240; ON thấy proxy cylinder, OFF thấy mesh người. Cùng cơ chế ẩn visual camera, không bỏ collision/LiDAR proxy.
- [Classifier validation](classifier_validation.png): inference CUDA mới trên validation đã ghi, giữ model depth_linear và threshold0,5; 11TP/1FN/0FP/18TN, 96,7% accuracy trên30proposal. Có [ảnh người bị bỏ sót](classifier_missed_case.png).
- [Trang xem với controls](index.html): khi mở local có thể serve thư mục này qua HTTP localhost; video có dừng/tua/tốc độ/toàn màn hình. Đường localhost35315 trong phiên demo là tạm thời, không phải địa chỉ cố định khi clone repo.

10/10 test classifier CUDA PASS; GPU inference và NumPy thỏa atol=rtol=1e−6 (max absolute logit difference1,43e−6). Không fit lại, không chọn lại threshold, không đọc/evaluate test split. Model chỉ phân loại proposal geometry; chưa là detector toàn ảnh hoặc ROS node chạy live. Nhãn được đọc riêng để tính metric, không đưa vào inference.

Code tạo ảnh classifier: classifier_visual.py; chạy từ môi trường CUDA có Torch/Matplotlib, truyền --output /đường/dẫn/mới. Script tự tìm root repo và từ chối ghi đè output đã có. JSON summary chứa đường dẫn raw, audit và scope. Raw MCAP/cặp ảnh/script trước audit được giữ trong local_evidence (ignored). Audit đầu thiếu ROS/source-root được giữ nguyên; lệnh đúng môi trường và source-root đã PASS, không sửa dữ liệu/threshold để đổi kết quả.

Giao diện Windows được xác nhận ở **lượt này**; không suy rộng rằng mọi cấu hình WSLg đã hết lỗi. GUI capture30% vẫn là vấn đề hiệu năng của chế độ thu, chưa thay timing/tham số Nav2 để làm đẹp phép đo.
