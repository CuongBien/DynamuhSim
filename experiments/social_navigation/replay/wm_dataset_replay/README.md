> Cập nhật 29/10/2026: đã sửa replay cố định animation frame0. Người dùng xác nhận tay/chân chuyển động; gait dựng từ quãng đường ghi được, không tái tạo chính xác bộ xương. Cửa sổ mới đã xem và thao tác được; các giới hạn GUI bên dưới mô tả lượt trước. [Costmap và giải thích điều hướng mới](../../../../docs/demos/2026-10-29/navigation/README.md).

# Phát lại batch WM đã ghi

Backup hoàn tất: [Google Drive](https://drive.google.com/drive/folders/1I3kfKbTm5X6Bj32RyPVck5kFSjmVgqnl), 14 ZIP độc lập và hướng dẫn. Xem stage wm_episode_batch/reports/BACKUP_DRIVE.md.

Có hai cách xem, nguồn đều là **train wm_standing_45**, không đọc hình test:

1. **Camera đã ghi:** video WebM từ 90 frame camera Gazebo thật, 640×480, không tổng hợp hình; mặc định 4×, nút 1× đặt playbackRate=0.25. Có dừng/tua/toàn màn hình. Phục vụ file cục bộ, không publish mạng ngoài. Đã kiểm tra decode đủ 90 frame và HTTP từ Windows trả 200; lệnh mở trình duyệt đã được gửi. Chưa có xác nhận người dùng nhìn thấy trang này.
2. **Gazebo pose replay:** scene nhà xưởng copy riêng, robot + 6 người dùng XY/yaw từ gazebo_poses.jsonl, nội suy theo timestamp, phát 1× và lặp sau khoảng 95.14 giây. Không chạy Physics/Nav2/human policy, không tạo lại cảm biến. Chiều cao/roll/pitch và trạng thái bộ xương không có trong log; pha bước chân hiện được dựng lại từ quãng đường XY đã ghi, không phải trạng thái bộ xương gốc. Đây là xem lại quỹ đạo trong 3D, không phải RGB gốc hay tái thực thi vật lý.

**GUI Gazebo chưa nghiệm thu trên màn hình Windows.** Đã render được scene thật bằng D3D12 NVIDIA RTX4060 và lấy ảnh qua service Gazebo, kiểm chứng độc lập 3.206 model poses với sai số ≤4.45e−16. Tuy nhiên người dùng chỉ thấy taskbar, kể cả sau di chuyển/raise cửa sổ Linux. Computer Use helpers không khởi động được. Thử Wayland sau cùng lỗi segfault Qt MinimalScene; runner đã dừng server của phiên đó. Không tuyên bố cửa sổ đã thao tác được. Ảnh reports/native_gazebo_scene.png là ảnh nội bộ render, không phải bằng chứng cửa sổ Windows hiện ra.

## Mở bản ghi camera

```bash
python3 experiments/social_navigation/replay/wm_dataset_replay/serve_recording.py
```

Mở URL localhost vừa in trong trình duyệt. Server chỉ chia sẻ hai file của viewer, không phục vụ cả repository. Ctrl+C dừng server. Có thể dùng `--port 8765` nếu cổng trống. Video cũng xem trực tiếp ở reports/recorded_camera_4x.webm.

## Thử Gazebo replay riêng

```bash
source /opt/ros/jazzy/setup.bash
cmake -S experiments/social_navigation/replay/wm_dataset_replay -B experiments/social_navigation/replay/wm_dataset_replay/generated/build_gait
cmake --build experiments/social_navigation/replay/wm_dataset_replay/generated/build_gait -j 2
# Local raw đã thu trước refactor; đổi BATCH_DATA nếu restore ở chỗ khác.
BATCH_DATA="$PWD/experiments/social_navigation/benchmark/wm_episode_batch"
python3 experiments/social_navigation/replay/wm_dataset_replay/run.py --seconds 1800 \
  --raw-root "$BATCH_DATA/generated/runs/wm_standing_45" \
  --runtime-root "$BATCH_DATA/generated/runtime/wm_standing_45"
```

Đóng GUI để runner dọn server, hoặc Ctrl+C ở terminal chạy runner. `--headless` kiểm tra server; `verify.py` trong shell đã source ROS kiểm tra actual Pose_V của SceneBroadcaster so với log gốc khi replay đang Play. Mỗi phiên dùng GZ_PARTITION riêng; không launch ROS bridges hoặc sửa source/config ROS, asset, GUI config mặc định. Runtime/config/log chỉ ở generated/. Preflight bắt buộc NVIDIA trên môi trường WSL hiện tại; LD_PRELOAD libd3d12core và Qt QSG basic chỉ áp dụng cho tiến trình của demo. `--gui-platform wayland` được giữ để tái hiện lỗi, không phải cấu hình đã nghiệm thu.

## Bằng chứng

- reports/pose_verification.json: so sánh SceneBroadcaster, không lấy số từ plugin status.
- reports/media_provenance.json: nguồn và giới hạn kiểm chứng UI.
- reports/native_gazebo_scene.png: screenshot service của Gazebo ở phiên xcb riêng.
- reports/recorded_camera_4x.webm: bản mã hóa từ GIF camera gốc, 22.5 giây.
- generated/view_*/: server/gui logs, session/closed metadata, screenshot thô; giữ cả lượt fail.

Chưa train World Model hoặc RL trong bước backup/replay này.

`--raw-root` và `--runtime-root` chọn đúng recording/world đã thu; không copy hoặc thu lại dữ liệu. Nếu bỏ hai cờ, mặc định tìm dưới `datasets/wm_episode_batch/generated/`.
