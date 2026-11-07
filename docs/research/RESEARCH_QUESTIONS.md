# Câu hỏi nghiên cứu

| ID | Câu hỏi | Đối chứng và cách đo |
| --- | --- | --- |
| RQ1 | Kết quả tương tác đổi bao nhiêu khi thay backend điều hướng người? | Ghép cặp scenario/seed/robot policy giữa ORCA, Social Force, scripted và backend mở rộng. Đo khoảng hở, va chạm, nhường/chờ, hoàn thành và chênh lệch lệnh thực thi. |
| RQ2 | Đưa hành động robot vào dự đoán có cải thiện chất lượng dự báo tới 3 giây? | So với baseline vận tốc không đổi và với cùng họ mô hình bỏ điều kiện hành động, giữ input hợp lệ, split và ngân sách huấn luyện. Đo ADE/FDE theo horizon, occupancy IoU ở ngưỡng đã chốt, và hiệu chuẩn uncertainty theo từng horizon. |
| RQ3 | Risk Map từ world model có cải thiện an toàn và hiệu quả điều hướng? | So WM-PPO với PPO no-WM để đo tác động của tín hiệu dự đoán trong họ PPO. So với Nav2/MPPI như đối chứng hệ thống. Báo va chạm, đến đích, thời gian, khoảng hở, độ mượt và độ trễ. |
| RQ4 | Kết quả có giữ được khi đổi bố cục, backend người hoặc chuyển sang robot thật? | Báo riêng nhóm layout, hành vi và môi trường giữ lại. Ghép các trial có hình học và nhiệm vụ tương ứng trong mô phỏng và thực tế. Phân tích thay đổi về cảm biến, điều khiển và thời gian tính. |

Với dự báo đa mode, chốt cách chọn mode hoặc metric phân phối trước khi chấm. Định nghĩa occupancy gồm vùng đánh giá, frame, độ phân giải và cách xử lý che khuất. Mask phần horizon không có nhãn hợp lệ. Khi đánh giá đóng vòng, tách kết quả theo scenario và seed. Chốt đơn vị bootstrap hoặc phương pháp ước lượng bất định trước khi xem kết quả. [Mục tiêu](RESEARCH_OBJECTIVES.md), [Ma trận baseline](BASELINE_MATRIX.md)
