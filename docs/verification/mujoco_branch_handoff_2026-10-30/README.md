# Nghiệm thu nhánh bàn giao MuJoCo — 30/10/2026

**PASS phần chuẩn bị bàn giao. MuJoCo backend chưa triển khai.**

- Nhánh: feat/mujoco-backend. Implementation:04e38dab651dbc17e6d5ce5a58642d55c5dd015d.
- Mốc Gazebo giữ nguyên local và GitHub:dd7a2b03d9fe40841052c0db6c7dc52619e13e67.
- README root tập trung việc nhận bàn giao; [runbook](../../MUJOCO_HANDOVER.md) có overview,clone/setup,data,ROS topic types,reference commands,scope và M1–M6.
- [Workspace riêng](../../../experiments/social_navigation/mujoco_backend/README.md), [task manifest](../../../configs/handoff/mujoco_port.json), [preflight](../../../tools/mujoco_handoff.py).

[Result](result.json), [fresh checkout receipt](fresh_checkout_check.json), [environment/coverage](fresh_checkout_summary.json), [log](fresh_checkout.log).

Kiểm thực tế trên một git clone --shared mới tại /tmp, không có ignored vendor/venv/install/build/raw/generated. Clone mượn object Git của repo gốc để tiết kiệm copy; code working tree là tracked checkout sạch. Dùng venv tách biệt chỉ cài NumPy1.26.4, không import ROS khi đọc data.

1. Reference ancestry và116protected hashes PASS.
2.38contract tests PASS trong môi trường chỉ có NumPy.
3. ZIP riêng đúngSHA,CRC/inventory41file,517window vàloader PASS;ROS_imported=false.
4. Từ chối receipt đã có mà không đổi bytes; từ chối --archive thiếu --restore-dir.
5. Link nội bộ mới resolve đầy đủ; tất cả original source ngoài README/docs/.gitignore không thay đổi. Không xóa/đổi tên code cũ.
6. Nhánh Gazebo local và GitHub vẫn ở cùngdd7a2b0.

Máy tham chiếu thiếu ensurepip. Không cài/đổi Python hệ thống; test dùng python -m venv --without-pip và system pip --python nhắm venv riêng. Runbook giải thích gói python3.12-venv và fallback này.

Chỉ kiểm code/data handoff trên host tham chiếu, không tuyên bố fresh OS provision, Gazebo native/render hoặc MuJoCo đã chạy. Chưa cài MuJoCo, tạo MJCF/collector/bridge, thu batch hoặc train. Gói portable/raw vẫn phải chuyển riêng ngoài Git. Các milestone MuJoCo hiện NOT_STARTED.
