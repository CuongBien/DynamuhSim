# Bộ demo ngắn để chọn mô hình người

Ưu tiên mở các GIF camera đã ghi trước khi chạy lại live; đây là RGB từ Gazebo thật, không AI. Cửa sổ GUI vẫn có lỗi Qt/OpenGL ngẫu nhiên ở WSL; headless vẫn dùng NVIDIA cho camera/LiDAR/rendering. Không hứa một lượt mới có kết quả y hệt vì AMCL/MPPI/physics chưa joint-seeded.

1. **Nhà xưởng và robot:** cho xem [khung đầu](reports/walking01_v2/initial.png): factory60×40m, robot Mecanum/LiDAR/IMU,6 người; map và Nav2 hiện có.
2. **Nhóm đi cùng nhau:** mở [walking42](reports/walking01_v2/camera_preview_4x.gif). Nêu rõ hai nhóm ba người là fixture ban đầu; native membership/goal/lifecycle tiếp tục thay đổi. Pose Gazebo xác nhận nhóm đi thật, không chỉ gắn nhãn moving.
3. **Nhóm đứng và phản ứng robot:** mở [standing42 v3](reports/standing01_v3_headless/camera_preview_4x.gif). Người hướng vào tâm; khi robot sát gần, geometry variant cho nhích tránh≤0.2m/s. Không có giọng nói/clip hội thoại riêng. Robot có thể đi giữa nhóm vì Nav2 chưa group-aware.
4. **Đối chiếu ba framework:** mở [bảng Nav2 cùng nhiệm vụ](../shared_nav2_pilot/reports/SHARED_NAV2_REPORT.md), rồi [báo cáo nhóm](reports/GROUP_NAV2_REPORT.md). CSP ORCA+ là người độc lập tránh nhau/robot; ComplexityNav là geometry hybrid, không trained RGL. Không dùng vài lượt để xếp hạng an toàn/tốc độ hoặc mức tự nhiên.
5. **Kết luận lựa chọn:** ưu tiên SocNavGym variant cho nghiên cứu nhóm/formation, giữ CSP ORCA+ làm baseline người độc lập, CN geometry làm nhánh nghiên cứu môi trường chật. Chưa đổi backend mặc định hay kiến trúc robot.

Khi trình bày kết quả, đọc cột **đích thật**: action SUCCEEDED chưa đủ. Hai walking mới vượt ngưỡng20cm nên không đạt actual goal; giữ nguyên, không nâng ngưỡng. Lượt standing v2 bị dừng bảo vệ và hai lỗi GUI vẫn có log. Group behavior, goal và clearance là ba kết quả riêng.

GIF phát4× simulation time, không realtime. Dùng [README](README.md) để chạy lại từng cảnh bằng output directory mới, hoặc chỉ xem dữ liệu đã lưu để demo ổn định. Không chạy hai simulator/experiment cùng ROS domain đồng thời.

Phạm vi khảo sát/chọn mô hình đã có đủ demo và bằng chứng để trình bày. Các phần chưa nghiệm thu gồm GUI WSL ổn định, Nav2 đạt goal thật trong mọi lượt, robot tránh không gian xã hội nhóm, dense crowd/narrow-aisle nhóm, in-place reset và animation hội thoại. Đây là các công việc bổ sung cần chốt phạm vi riêng; không tự gọi toàn bộ project social navigation đã hoàn tất.
