# Mục tiêu nghiên cứu

Đánh giá liệu dự báo chuyển động của người có điều kiện theo hành động robot có cải thiện điều hướng trong hành lang, cửa hẹp và giao cắt khuất tầm nhìn hay không. Tách tác động của mô hình hành vi người, mô hình dự đoán và chính sách điều hướng bằng đối chứng có điều kiện chạy xác định.

| Mục tiêu | Bằng chứng cần thu |
| --- | --- |
| Mô phỏng người và robot trong không gian hẹp bằng MuJoCo | Fixture đúng hình học, chuyển động, cảm biến, tiếp xúc và tái lập theo seed. |
| Đối chứng các mô hình hành vi người | So sánh ghép cặp ORCA, Social Force, scripted và backend mở rộng theo tương tác, chuyển động và tỷ lệ lỗi. |
| Xây dựng dữ liệu đa cảm biến | Episode đồng bộ depth, LiDAR, odometry, IMU và lệnh robot, kèm kênh nhãn tách biệt, manifest và split có thể tái lập. |
| Dự báo có điều kiện theo hành động robot | Dự báo quỹ đạo người, occupancy và uncertainty đến 3 giây. Đo sai số và hiệu chuẩn trên scenario và nhóm seed chưa dùng khi huấn luyện. |
| Chuyển dự báo thành tín hiệu điều hướng | Tạo Risk Map từ dự báo và uncertainty, rồi đo hiệu quả của WM-PPO so với PPO no-WM và Nav2/MPPI trên cùng tập scenario giữ lại. |
| Kiểm chứng trên robot thực | Đối chiếu đồng bộ cảm biến, độ trễ và hành vi trong scenario tương ứng. Ghi rõ sai khác giữa mô phỏng và thực tế. |

Tiêu chí chính cho điều hướng là tỷ lệ episode có va chạm và tỷ lệ đến đích trong ngân sách. Báo thời gian, khoảng hở, độ mượt và độ trễ suy luận để diễn giải sự đánh đổi. Khóa cách tính, ngưỡng và điều kiện thành công trước khi đánh giá. [Câu hỏi nghiên cứu](RESEARCH_QUESTIONS.md), [Ma trận baseline](BASELINE_MATRIX.md)
