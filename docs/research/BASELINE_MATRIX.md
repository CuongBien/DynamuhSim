# Ma trận baseline

Các hàng là điều kiện dự kiến cho các task sau. Mỗi điều kiện có cấu hình và ID riêng.

| Nhóm | Điều kiện | Tín hiệu đầu vào | Biến so sánh |
| --- | --- | --- | --- |
| Hành vi người | ORCA | `HumanContext` | Tránh va chạm tương hỗ. |
| Hành vi người | Social Force | `HumanContext` | Tương tác theo mô hình lực. |
| Hành vi người | Scripted | `HumanContext` và kịch bản cố định | Mốc hành vi dễ lặp lại. |
| Hành vi người | Backend mở rộng | `HumanContext` và cấu hình backend đã đăng ký | Điều kiện độc lập theo [giao thức hành vi](../protocols/HUMAN_BEHAVIOR_PROTOCOL.md). |
| Dự báo | Vận tốc không đổi | Lịch sử ước lượng từ kênh quan sát | Mốc quỹ đạo xác định. Chỉ báo metric occupancy/uncertainty nếu baseline định nghĩa đầu ra tương ứng. |
| Dự báo | Mô hình không điều kiện hành động | Lịch sử quan sát hoặc feature | Đối chứng cho việc đưa hành động robot vào dự đoán. |
| Dự báo | Action-Conditioned World Model | Cùng lịch sử, thêm lệnh robot đã biết và chuỗi lệnh ứng viên | Dự báo quỹ đạo, occupancy, uncertainty tới 3 giây. |
| Điều hướng | Nav2/MPPI | Cảm biến, bản đồ và nhiệm vụ theo giao thức | Đối chứng hệ thống. Khóa phiên bản, tham số MPPI và cấu hình costmap. |
| Điều hướng | PPO no-WM | Quan sát, nhiệm vụ và thông tin bản đồ hợp lệ | Đối chứng PPO không cấp dự báo/Risk Map từ world model. |
| Điều hướng | WM-PPO | Cùng quan sát/nhiệm vụ, thêm Risk Map từ world model | Tác động của dự báo trong họ PPO. |

## Kiểm soát so sánh

- Ghép cặp episode theo scenario và seed, cố định robot/cảm biến/giới hạn lệnh, quy tắc dừng và thông tin nhiệm vụ. Giữ backend người trong một cặp điều hướng. Báo riêng nếu đánh giá chuyển sang backend chưa thấy.
- Dùng cùng chính sách cung cấp bản đồ và định vị từ thông tin cho phép. Nav2/MPPI có thể dùng biểu diễn costmap riêng theo cấu hình. Công bố mọi khác biệt về thông tin và chi phí tính toán.
- Với hai biến thể PPO, giữ họ policy, tập scenario train, reward, ngân sách môi trường và quy trình chọn checkpoint nhất quán. Ghi khác biệt cần thiết về đầu vào và thời gian suy luận. Dùng tập đánh giá giữ lại đã khóa cho cả ba hệ thống.
- Chỉ dùng chuỗi lệnh được biết tại thời điểm suy luận làm điều kiện hành động. Với dự báo ngoại tuyến có chuỗi lệnh được cấp sẵn, báo đó là điều kiện đánh giá riêng. Trong đóng vòng, lấy chuỗi ứng viên do policy hoặc bộ lập kế hoạch cấp tại tick quyết định. Lưu kết quả theo [schema episode](../schemas/EPISODE_SCHEMA.md).
