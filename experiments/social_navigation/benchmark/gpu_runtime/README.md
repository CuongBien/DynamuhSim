# Kiểm tra GPU và dọn tiến trình thử nghiệm

Runner benchmark và demo SocNavGym hiện kiểm OpenGL thật trước khi khởi động
Gazebo; trên máy WSL này chọn D3D12/NVIDIA và bỏ cờ software rendering còn sót
trong môi trường của tiến trình con. Không sửa shell profile/driver/cấu hình
Windows hay ROS/Nav2 gốc. [Kết quả kiểm tra](reports/run01/GPU_RUNTIME.md).

Các lệnh demo/benchmark hiện có giữ nguyên. Ví dụ kiểm tra ngắn, không phải
thử tới đích; dùng output directory mới:

```bash
bash experiments/social_navigation/benchmark/run.sh trial \
  --backend empty --condition bottleneck --seed 42 --sim-budget 2 \
  --report-dir experiments/social_navigation/benchmark/gpu_runtime/reports/run02/smoke
```

Runner ghi `rendering.json`, `process_cleanup.json` vào runtime được chỉ trong
result JSON; nếu Ogre log cập nhật trong phiên, lưu thêm `ogre2_session.log`.
Không chấp nhận llvmpipe/softpipe hoặc fallback software. Nếu không mở được
GLX display hoặc không có hardware context, dừng và báo lỗi trước mô phỏng.
Probe dùng g++, libGL/libX11 và headers hiện có; không cài CUDA/toolkit mới.
WSL cần WSLg DISPLAY hoạt động; đây chưa phải hỗ trợ EGL displayless.

Kết thúc phiên, ngoài stop process group cũ, guard tìm tiến trình **cùng UID
và cùng token phiên riêng** trong `/proc`, kể cả child đã đổi session và cha
đã thoát; SIGINT → SIGTERM → SIGKILL nếu cần. Không pkill theo tên ROS/Gazebo
và không chạm phiên khác. SIGKILL trực tiếp vào chính runner hoặc việc child
xóa token không nằm trong đảm bảo `finally` của guard này.

Kiểm thử hành vi bằng child giả, không khởi động ROS/Gazebo:

```bash
python3 experiments/social_navigation/benchmark/gpu_runtime/check.py
```

Script kiểm child detached/cha đã thoát, child phớt lờ shutdown mềm, bảo toàn
process không cùng phiên, và kiểm GPU thật khi shell còn cờ software cũ.
Checks cố định ở reports/run01, dùng cho kiểm tra triển khai local.

GPU tăng tốc phần render/camera/GPU LiDAR. MPPI Nav2 phiên bản đang dùng,
AMCL/costmaps, native ORCA và SFM vẫn tính trên CPU; thêm CUDA vào container
không tự chuyển các thuật toán này lên GPU. Không tăng render load để làm
GPU đạt 100%; chạy một lượt mỗi lần và đảm bảo cleanup sau lượt.
