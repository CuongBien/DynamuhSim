# Commit MCP, đổi tên nhánh và merge — 01/11/2026

Nhánh feat/mujoco-backend đổi thành mcp-setup. Commit6c40f78a chứa đúng3file MCP mới; không chỉ merge các commit bàn giao cũ. Nhánh mới đã push trước khi bỏ tên remote cũ. Nhánh test nhận mcp-setup qua merge --no-ff; README conflict được giải quyết bằng cách giữ các entrypoint detector và thêm MCP/MuJoCo.

344tests OK (2skip); preflight MuJoCo PASS CODE_CHECKS_ONLY với protected116file và contract smoke. MCP Python/bash syntax và real SDK import/CLI --help PASS. Không mở server/simulator, không cấu hình Tailscale/firewall, không train hoặc promote detector. Logs StartingGazebo/timeout là mock tests.

Nhánh mujoco mới được tạo từ test HEAD sau merge; configs/runbook active dùng tên mujoco. cuong.txt giữ local và không stage/push. Proof lịch sử giữ tên nhánh cũ theo thời điểm chạy.

[Receipt](acceptance.json) · [Full suite](full_tests.log) · [Preflight](mujoco_preflight.json) · [MCP CLI](mcp_help.log) · [Nhánh hiện tại](../../BRANCH_WORKFLOW.md).
