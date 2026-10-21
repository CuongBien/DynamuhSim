# Gazebo WSL D3D12 shutdown workaround

Đã kiểm tra trên Ubuntu24.04/WSLg, Gazebo8.15.0, Mesa25.2.8, RTX4060 Laptop, driver610. Đây là workaround cục bộ, opt-in; không patch thư viện hệ thống hay launch robot gốc.

```bash
bash experiments/social_navigation/benchmark/run.sh trial \
  --backend crowdsimplus --human-policy orca_plus --goal-layout individual \
  --condition bottleneck --seed 42 --shutdown-mode wsl_d3d12 \
  --report-dir experiments/social_navigation/benchmark/shutdown_fix/reports/my_demo --gui
```

Chọn thư mục mới mỗi lần. `--gui` chưa được xác nhận bằng thao tác tay trong đợt test này; các kiểm tra tự động dùng headless server nhưng vẫn render camera/GPU LiDAR bằng NVIDIA. Kết quả ngắn 2 sim s chỉ kiểm tra vòng đời, không đánh giá hoàn thành goal.

Cách hoạt động:

1. Wrapper sinh trong runtime chỉ đặt `LD_PRELOAD=/usr/lib/wsl/lib/libd3d12core.so` cho `gz sim`, giữ thư viện nạp đến hết tiến trình. Các lệnh `gz service` và ROS không được preload.
2. Sau khi đo xong, gửi `/server_control` với `stop: true` trong partition riêng. Đợi Gazebo trước khi dừng ROS/worker.
3. Bộ guard vẫn dọn descendants có session token riêng. Thoát sạch phải được xác nhận bằng log launch; phản hồi `data:true` hoặc không còn PID không đủ chứng minh.

`signals` (default lịch sử) vẫn giữ nguyên; `server_control` chỉ dùng đối chứng RPC, không giải quyết segfault trên máy này. Ma trận localization mới truyền rõ `--shutdown-mode wsl_d3d12`.

Bằng chứng trong [reports/probes.json](reports/probes.json) và [reports/backtrace.json](reports/backtrace.json):

- Sáu lượt stability cũ đều có Gazebo -11 khi dừng. Lượt RPC riêng `probe01` vẫn -11, worker đã hết ConnectionResetError.
- Hai lượt portable GDB bắt SIGSEGV. Lượt debug02 đối chiếu PC với `/proc/PID/maps` ngay trước stop: địa chỉ lỗi thuộc `libd3d12core.so`; ở thời điểm fault thư viện đã không còn trong mappings. Thread Sensors đang join. Điều này phù hợp callback driver còn sống sau khi thư viện bị unload; chưa có native debug symbols để xác định hàm driver cụ thể.
- Một preload probe ban đầu và ba lượt kiểm chứng bình thường (ORCA, SFM, SocNavGym) đều có `gz` exit sạch, dùng fresh Ogre NVIDIA, không còn owned process. SocNavGym có cảnh báo observation-space từ Gymnasium, được giữ trong báo cáo; không có worker traceback.
- GDB và dependency chỉ download/extract trong `generated/debugger`, không cài system package, không sửa ptrace/core settings.

Đây là workaround đã kiểm tra trên máy này, chưa chứng minh portability sang driver/Gazebo khác. Startup preload có thể xung đột với preload khác hoặc driver khác; chỉ dùng mode này khi có WSL D3D12 core và GPU preflight qua. Chưa sửa lỗi driver upstream. Không đổi native policy, physics hay AMCL để che lỗi shutdown.
