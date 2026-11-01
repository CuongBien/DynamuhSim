#!/usr/bin/env bash
# run_server.sh: Starts the MCP server on WSL2 for remote AI access over Tailscale.

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PORT="${1:-8765}"
HOST="${HOST:-0.0.0.0}"

echo "=========================================================="
echo " Starting DynamuhSim ROS 2 MCP Server"
echo "=========================================================="
echo "Port: ${PORT}"
echo "Host: ${HOST}"
echo ""

# Get WSL IP
WSL_IP=$(hostname -I | awk '{print $1}')
echo "WSL Local IP: ${WSL_IP}"

# Check Tailscale IP if available
if command -v tailscale >/dev/null 2>&1; then
    TAILSCALE_IP=$(tailscale ip -4 2>/dev/null || true)
    if [ -n "$TAILSCALE_IP" ]; then
        echo "Tailscale IP (WSL): ${TAILSCALE_IP}"
        echo "Endpoint URL: http://${TAILSCALE_IP}:${PORT}/sse"
    fi
fi

echo "----------------------------------------------------------"
echo "If Tailscale is running on Windows, use your Windows Tailscale IP:"
echo "Endpoint URL: http://<windows-tailscale-ip>:${PORT}/sse"
echo "=========================================================="

python3 "$SCRIPT_DIR/server.py" --host "$HOST" --port "$PORT" --transport sse
