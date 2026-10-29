# Kiểm tra kích thước robot và inflation — 29/10/2026

## Kết luận

Nhận xét robot trên Gazebo nhỏ hơn footprint điều hướng là đúng. Đọc **runtime** SDF của lượt đã ghi cho thấy chassis24×20cm, collision gồm bánh bao24×28cm, bán kính tròn bao toàn collision khoảng16,806cm. Nav2 recipev3 dùng robot_radius25cm + padding1cm, thư viện Jazzy thực sự dựng footprint16đỉnh ngang52cm; bán kính inscribed25,493cm, circumscribed26,414cm. Đây là vùng điều hướng bảo thủ lớn hơn mô hình vật lý, không phải kích thước chassis.

URDF hiện tại khai báo thân28×22cm và bánh ở(±10,±12)cm, khác SDF thân24×20cm/bánh(±8,±10)cm. Capture launch dùng SDF để spawn, còn URDF cho robot_state_publisher. Đọc lại TF static từ MCAP xác nhận camera_link x14cm, trong SDF là12cm; khác2cm. Không tự sửa hình học, TF, raw hoặc nhãn trong lượt kiểm tra này. Tính nhất quán source/frame và lựa chọn kích thước chuẩn cần được giải quyết trước lần capture mới.

## Tách hai tác động

[Ảnh so sánh cùng tỷ lệ và costmap](footprint_inflation_comparison.png).

| Tình huống | Bề ngang footprint | Inflation | Cost giữa kheA |
|---|---:|---:|---:|
| Bản ghi thật hiện tại | 52cm | 0,80m | 99 |
| Tính lại, chỉ giảm inflation | 52cm | 0,45m | 99 |
| Giả định radius17cm + padding1cm | 36cm | 0,80m | 78 |
| Giả định giảm cả hai | 36cm | 0,45m | 78 |

Ba hàng sau **chỉ là tính offline**, giữ nguyên lethal cells của local costmap sim32,9025s (xấp xỉ video5s). Không chạy planner/controller với candidate, không coi đây là nghiệm thu đường tắt. Radius17cm được chọn từ hình tròn bao collision SDF để phân tích, không tự coi là cấu hình đúng cho robot thật hoặc model URDF đang khác kích thước.

Inflation là vùng chi phí mềm bên ngoài vùng inscribed. Việc giảm inflation_radius không loại vùng inscribed do footprint xác định. Cost phía ngoài còn phụ thuộc cost_scaling_factor và MPPI critics. Vì vậy có cơ sở nói footprint là một nguyên nhân cụ thể; chưa chứng minh0,8m tự nó sai trong mọi tình huống.

## Kiểm chứng phương pháp

- footprint_probe.cc liên kết thư viện Nav2 đang cài, gọi makeFootprintFromRadius, padFootprint và calculateMinAndMaxDistances; không chỉ đo đường tròn xấp xỉ trên hình.
- Probe không gọi ROS init hoặc chạy robot. Kết quả raw ở footprints.json.
- Dựng lại inflation theo khoảng cách tới recorded lethal cells và computeCost Jazzy, chuyển sang OccupancyGrid0–100. Baseline khớp14.398/14.400cell (99,986%), 2cell khác tối đa2đơn vị. Do đó bản dựng là xấp xỉ được kiểm, không công bố parity tuyệt đối.
- actual_tf_static.json lấy từ MCAP; geometry.json từ runtimeSDF cùng episode, không đo bằng kích thước pixel của video.
- Không thay controller, radius, inflation, SDF/URDF/TF, model classifier hoặc tập dữ liệu. Recipe/capture freeze gốc được verify lại.

## Bước tiếp theo hợp lý

1. Chốt model kích thước chuẩn; đồng bộ SDF, URDF và sensor TF theo model đó.
2. Chọn footprint bao đủ thân/bánh/phần nhô của model, giữ padding có giải thích; không lấy riêng24×20cm của chassis làm footprint toàn robot.
3. Dùng config/episode mới để A/B footprint đúng và inflation, đo collision/clearance/goal/time. Không ghi đè kết quả v3 hoặc dùng hình offline để khẳng định robot đã lách thành công.

Code compare_inflation.py/read_tf.py đang trỏ raw demo tại đường dẫn máy gốc trong script; cần raw MCAP khi tái lập. Build probe với ROS Jazzy sourced và CMake ở thư mục output mới.

Nguồn thuật toán: [Jazzy footprint](https://raw.githubusercontent.com/ros-navigation/navigation2/jazzy/nav2_costmap_2d/src/footprint.cpp), [Jazzy inflation computeCost](https://raw.githubusercontent.com/ros-navigation/navigation2/jazzy/nav2_costmap_2d/include/nav2_costmap_2d/inflation_layer.hpp).
