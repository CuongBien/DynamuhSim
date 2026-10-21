# ComplexityNav: contract và geometry trước Gazebo

[Báo cáo đã đo](reports/CONTRACT_REPORT.md): contract cập nhật người đạt, **chưa đủ điều kiện nối Gazebo nhà xưởng**. Nhánh centralized ORCA/SFM native bỏ qua vật cản bên trong; hai fixture đi xuyên kệ dù bắt đầu hợp lệ. Không sửa source upstream hoặc che kết quả bằng clipping.

Đã chạy trong image vòng3 có sẵn, ghim theo ID `sha256:edfd591ea7551813419e511b341089ad956b6d3e4c1bd395a326035865ef1d5d`. Python3.8.13/Gym0.21; giữ ROS Jazzy/Python3.12 của host nguyên trạng. Không tải package/model mới, không dùng RGL hay CV-MPC robot ở cohort này. Đây là native multipolicy human dynamics với robot input định trước.

Từ repository root, chạy lại tuần tự với tên report MỚI:

```bash
python3 experiments/social_navigation/benchmark/complexitynav_contract/run.py --report-name my_run
experiments/social_navigation/.venv-socnavgym/bin/python \
  experiments/social_navigation/benchmark/complexitynav_contract/analyze.py \
  --report-dir experiments/social_navigation/benchmark/complexitynav_contract/reports/my_run
```

Host wrapper tạo bản sao layout từ nhà xưởng và chỉ mount thư mục experiment này. Container không mạng, tối đa2 CPU/4GB, BLAS/OMP một thread; không cần CUDA cho phép thử ORCA/SFM nhỏ này. Wrapper mới được kiểm tra cú pháp/help; run02 thực tế dùng lệnh Docker tương đương đã lưu trong execution.json, không tự nhận đã chạy lại qua wrapper. Analyzer chạy bằng venv hiện có, không cài gì vào ROS. Local preservation checkpoint được tạo trước cohort; checkout khác cần checkpoint tương đương để audit staging/source.

Contract: frame world, native local origin(−24,0), timestep0.25s giữ nguyên. Input ở đầu interval gồm x/y/vx/vy/yaw robot; output endpoint có hiệu lực ở time+0.25s. Kernel không điều khiển robot thật. ID ngoài adapter1..6 giữ ổn định trong epoch; reset tạo env mới và tăng epoch. Pause không advance. Sai frame/epoch/clock/nonfinite bị từ chối trước mutation. Đây là contract offline, chưa kiểm tra transport/socket/ROS clock hay LiDAR trong Gazebo.

`geometry_ledger.json` ánh xạ đủ42 AABBs và đối chiếu SDF gốc nhưng là ledger kiểm toán; không tuyên bố native đã dùng chúng. Native outer rectangle đặt theo mặt trong tường thật, không tạo hàng rào quanh vùng lấy mẫu. Human radius0.3m native; robot radius0.25m cấu hình theo nominal project. Fixture6 người/goal sequences do harness cung cấp, không phải generator demo gốc và không có group formation/hội thoại.

Run01 thất bại ngưỡng so native ở hai frame do độ chính xác float32; giữ source snapshot/error. Run02 tách native parity cùng frame (ngưỡng1e−10, đo0) khỏi phép kiểm tra dịch frame20s (ngưỡng5mm, đo≤0.263mm). Native reference parity được assert trong probe; không xuất riêng trajectory reference. Analyzer hậu kỳ kiểm tra trace/geometry/source độc lập, không tự nhận tái lập parity từ dữ liệu reference chưa lưu.

Bước kế tiếp là adapter hình học riêng có gắn nhãn biến thể, giữ native làm đối chứng: chọn cách đưa đủ42 vật cản vào human solver rồi kiểm tra swept/progress/reset/robot feedback; chỉ nối population nhỏ vào Gazebo khi đạt. Chưa bắt đầu bước đó trong cohort này. Các thay đổi ngoài social experiment và source cũ được bảo toàn.
