# Nhánh làm việc hiện tại — 01/11/2026

- `mcp-setup`: tên mới của `feat/mujoco-backend`; chứa commit6c40f78a thêm tools/mcp_server/{server.py,run_server.sh,README.md}. Tên remote cũ đã bỏ sau khi publish nhánh mới và kiểm ancestor.
- `test/gazebo-harmonic-benchmark`: nhánh tích hợp. Đã có detector47fce50f qua merge6df01c83; merge này thêm MCP và scaffold/runbook MuJoCo từ mcp-setup. Model detector mới vẫn experimental.
- `mujoco`: nhánh port mới, tạo từ tip nhánh test sau merge MCP; workspace chính chuyển sang nhánh này. Chưa có backend MuJoCo chạy được.

`cuong.txt` là ghi chú cục bộ, không commit/push. Chỉ3file trong tools/mcp_server được đưa vào commit MCP. Không tự mở server, cấu hình Tailscale/firewall hoặc thay ROS/Nav2 khi thực hiện thao tác nhánh.

[Hướng dẫn MCP](../tools/mcp_server/README.md) · [Port MuJoCo](MUJOCO_HANDOVER.md) · [Review detector](INCOMING_DETECTOR_REVIEW_2026-10-31.md).

Các tên nhánh trong docs/verification và reports lịch sử giữ nguyên để phản ánh lúc kiểm chứng; dùng tên nhánh hiện hành ở trang này cho công việc mới.
