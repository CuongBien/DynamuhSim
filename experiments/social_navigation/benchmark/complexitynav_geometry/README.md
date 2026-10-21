# ComplexityNav: adapter hình học độc lập

[Báo cáo đã đo và hình trước/sau](reports/GEOMETRY_REPORT.md):6/6 cảnh adapter đạt kiểm định offline, có6 native reference ghép cặp. Tổng1040 update adapter (gồm reset80 bước),960 native reference. Đã tránh42 vật cản, đi vòng kệ tới đích và đi hai chiều qua lối hẹp1.8m. **Chưa chạy adapter này trong Gazebo.**

Đây là biến thể có sửa hành vi: roadmap đưa waypoint vào intention native ORCA/SFM, rồi global disc-ORCA có42 obstacles và swept filter giới hạn chuyển động. Với SFM, lớp cuối cũng là ORCA, vì vậy không gọi là SFM native hoặc dùng để xếp hạng SFM nguyên bản. Giữ source upstream và native reference không đổi.

Runtime image vòng3 ghim ID `sha256:edfd591ea7551813419e511b341089ad956b6d3e4c1bd395a326035865ef1d5d`, Python3.8.13, timestep0.25s. Human radius0.3m, robot radius0.25m. Không cài dependency vào ROS Jazzy/Python3.12 host; không sửa robot/Nav2/vendor hoặc image. Chạy CPU tối đa2 core, BLAS/OMP một thread,4GB RAM, không mạng. Cohort thực tế kết thúc khoảng16.1s wall; không cần CUDA cho phép thử toán học này.

Từ repo root, dùng tên report MỚI:

```bash
python3 experiments/social_navigation/benchmark/complexitynav_geometry/run.py --report-name my_run
experiments/social_navigation/.venv-socnavgym/bin/python \
  experiments/social_navigation/benchmark/complexitynav_geometry/analyze.py \
  --report-dir experiments/social_navigation/benchmark/complexitynav_geometry/reports/my_run
```

Wrapper sao chép dependency chỉ đọc từ `complexitynav_contract/kernel.py`, `socnavgym_warehouse_lifecycle/geometry.py` và layout nhà xưởng vào generated/. Analyzer kiểm SHA256 với source snapshot. Native/adapter dùng cùng fixture và robot input byte-identical trong mỗi cặp. Lượt ghi nhận run01 chạy Docker trực tiếp theo command trong execution.json; wrapper được kiểm cú pháp/help, không tự nhận đã đo lại toàn bộ qua wrapper.

Frame world, native local origin(−24,0), epoch/clock/reset theo contract cũ. Roadmap xét disc bằng GEOS trên đủ42 boxes, không dùng cửa sổ goal làm tường. Projection dùng polygon42 AABBs counter-clockwise, radius đệm0.02m, speed ceiling1m/s; swept margin0.002m. Robot chuyển động dự đoán đủ interval0.25s, không bị ORCA hoặc fraction của người di chuyển hộ. Input không khả thi báo lỗi, không silently ignore robot.

Giữ quy tắc arrival/goals của env gốc; waypoint chỉ đưa vào bản sao FullState cho intention, không thay native final goal hoặc counter. Kiểm định xác nhận arrival bằng khoảng cách pose thật tới goal trước khi counter tăng. Không spawn lại người. Pause không gọi planner. Reset tạo env/planner mới và tăng epoch; lặp80 bước chính xác. Static roadmap cache chỉ lưu đồ thị/goal distances.

Giới hạn: một seed42, fixtures6 người/lối hẹp2 người, chưa test mật độ cao hoặc mọi tuyến nhà xưởng. Safety filter không phải chứng minh an toàn cho mọi robot input. Yaw/animation, actual robot feedback/LiDAR và socket/ROS clock chưa được nghiệm thu trong cohort này. Khi planner lỗi, live worker cần freeze/fail-until-restart; cơ chế transport/error lifecycle sẽ được kiểm chứng khi nối Gazebo. Trước chạy trên checkout khác cần checkpoint bảo toàn source/staging tương đương, không viết lại provenance cũ.

Các PNG là biểu đồ từ trajectory2D đã chạy, có nhãn NOT GAZEBO; không phải ảnh AI hay camera. Bước kế tiếp: lưu phần mới thành commit riêng, nối pilot Gazebo population nhỏ, kiểm tra interpolation/proxy/LiDAR/robot feedback thực tế và NVIDIA/shutdown; chỉ sau đó tăng mật độ hoặc benchmark chung.
