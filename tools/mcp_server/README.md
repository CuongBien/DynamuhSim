# DynamuhSim Remote ROS 2 MCP Server qua Tailscale

Hệ thống MCP Server (Model Context Protocol) chạy trên WSL2, cho phép AI Agent của bạn bè (Cursor, Claude Desktop, v.v.) kết nối trực tiếp qua mạng riêng ảo Tailscale để điều khiển ROS 2, đọc ghi file và debug workspace từ xa.

---

## 1. Khởi chạy MCP Server trên máy của bạn (WSL2)

Thư mục đã được cài đặt tại:
`tools/mcp_server/`

### Chạy trực tiếp:
```bash
cd /home/cuongbien/DynamuhSim/tools/mcp_server
./run_server.sh
```
Mặc định server sẽ lắng nghe trên cổng `8765` với transport `SSE`:
- SSE Endpoint: `http://<tailscale-ip>:8765/sse`

---

## 2. Kết nối mạng Tailscale

Tailscale tạo ra mạng riêng ảo (VPN Mesh) mã hóa, cho phép máy bạn bè kết nối trực tiếp tới WSL của bạn bằng IP cố định (dạng `100.x.y.z`).

### Cách A: Cài Tailscale trên Windows (Khuyên dùng)
1. Tải và đăng nhập [Tailscale cho Windows](https://tailscale.com/download/windows).
2. Kiểm tra IP Tailscale của máy bạn (chuột phải vào icon Tailscale ở Taskbar hoặc gõ `tailscale ip -4`). Ví dụ: `100.80.20.10`.
3. Để bạn bè từ bên ngoài kết nối được cổng `8765` vào WSL2, mở **PowerShell (Run as Administrator)** trên Windows và chạy lệnh chuyển tiếp port một lần:
   ```powershell
   # Lấy IP của WSL2
   $wslIp = (wsl hostname -I).Trim().Split(" ")[0]
   # Chuyển tiếp cổng 8765 từ Windows sang WSL2
   netsh interface portproxy add v4tov4 listenport=8765 listenaddress=0.0.0.0 connectport=8765 connectaddress=$wslIp
   ```
   *(Hoặc nếu Windows 11 đã bật `networkingMode=mirrored` trong `.wslconfig`, bạn không cần chạy lệnh `netsh` trên vì WSL2 dùng chung IP với Windows).*

### Cách B: Cài Tailscale trực tiếp bên trong WSL2
1. Chạy trong terminal WSL:
   ```bash
   curl -fsSL https://tailscale.com/install.sh | sh
   sudo tailscale up
   ```
2. Lấy IP Tailscale:
   ```bash
   tailscale ip -4
   ```

---

## 3. Cấu hình trên máy của bạn bè

Chia sẻ IP Tailscale của máy bạn (ví dụ `100.80.20.10`) cho bạn bè (hoặc mời họ vào Tailnet của bạn qua tính năng Tailscale Share).

### Cấu hình cho **Cursor**:
Vào **Cursor Settings** -> **Features** -> **MCP Servers** -> **Add New MCP Server**:
- **Name**: `dynamuhsim-ros2`
- **Type**: `sse`
- **URL**: `http://100.80.20.10:8765/sse`

Hoặc thêm vào file `.cursor/mcp.json` trong dự án của bạn bè:
```json
{
  "mcpServers": {
    "dynamuhsim-ros2": {
      "url": "http://100.80.20.10:8765/sse"
    }
  }
}
```

### Cấu hình cho **Claude Desktop**:
Do Claude Desktop bản hiện tại sử dụng transport stdio để khởi tạo client, bạn bè dùng tiện ích `mcp-remote` (chạy qua npx, không cần cài đặt phức tạp).

Mở file cấu hình của Claude Desktop:
- Windows: `%APPDATA%\Claude\claude_desktop_config.json`
- macOS: `~/Library/Application Support/Claude/claude_desktop_config.json`

Thêm cấu hình:
```json
{
  "mcpServers": {
    "dynamuhsim-ros2": {
      "command": "npx",
      "args": [
        "-y",
        "mcp-remote",
        "http://100.80.20.10:8765/sse"
      ]
    }
  }
}
```

---

## 4. Các Tools AI Agent có thể gọi

| Tool | Mô tả |
| :--- | :--- |
| `execute_bash` | Chạy bất kỳ lệnh shell nào trên WSL. Tự động `source setup_env.sh` (ROS 2 Jazzy & DynamuhSim workspace). |
| `ros2_topic_list` | Liệt kê tất cả các topic ROS 2 đang hoạt động kèm kiểu message. |
| `ros2_topic_echo` | Echo tin nhắn từ topic (mặc định lấy 1 message, có timeout an toàn). |
| `ros2_node_list` | Danh sách các node ROS 2 đang chạy. |
| `ros2_node_info` | Xem chi tiết publishers, subscribers, services của một node. |
| `read_file` | Đọc nội dung file trong workspace theo phân trang dòng. |
| `list_directory` | Duyệt cây thư mục trong workspace. |

---

## 5. Ví dụ trải nghiệm của bạn bè

Khi bạn bè mở Agent (Cursor hoặc Claude Desktop) và prompt:
> *"Kiểm tra xem node ROS 2 vừa launch có bắn topic /cmd_vel không"*

Agent của bạn bè sẽ tự động:
1. Gọi tool `ros2_node_list` để kiểm tra các node đang active.
2. Gọi tool `ros2_topic_echo` với tham số `topic: "/cmd_vel", message_count: 1`.
3. Nhận payload JSON/text trả về từ WSL của bạn qua mạng Tailscale.
4. Trả lời bạn bè: *"Node đã xuất bản tin nhắn cmd_vel với linear.x = 0.2, angular.z = 0.05..."*
