# ADR-0002 · Hình ảnh người dùng chung

**Trạng thái** · Chấp nhận cho tích hợp Quaternius

Đặt bộ đọc rig, lấy mẫu animation và tạo khai báo skin trong `narrow_nav.visualization`. Bộ dựng world và demo dùng cùng phần xử lý Python/NumPy. `simulation/mujoco/` sở hữu MjModel, MjData và việc đồng bộ bone mocap với pose mô phỏng. Bộ dựng world chuyển chuỗi MJCF cùng tài nguyên bất biến qua biên này.

Da, tóc và quần áo lấy từ bundle Quaternius đã chuẩn hóa trong `sim/mujoco/assets/humans/`. Skin là lớp render. Collision, ground truth và điều khiển chuyển động giữ nguồn hiện có từ body và geom vật lý. Fingerprint asset đã nạp đi cùng fingerprint MJCF trong metadata.
