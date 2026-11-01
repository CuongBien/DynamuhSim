#!/usr/bin/env python3
"""
DynamuhSim Remote MCP Server (Model Context Protocol over SSE/HTTP)
Allows AI clients (Cursor, Claude Desktop, etc.) connected over Tailscale
to execute ROS 2 commands, inspect topics/nodes, and interact with the workspace.
"""

import argparse
import os
import subprocess
from pathlib import Path
from typing import Optional

from mcp.server.mcpserver import MCPServer
from mcp.server.transport_security import TransportSecuritySettings
from starlette.responses import HTMLResponse, JSONResponse

WORKSPACE_ROOT = Path("/home/cuongbien/DynamuhSim").resolve()
SETUP_ENV_SCRIPT = WORKSPACE_ROOT / "setup_env.sh"

server = MCPServer("DynamuhSim-ROS2-MCP")


@server.custom_route("/", methods=["GET"])
async def root_status(request):
    """Web test page for browsers."""
    html_content = """<!DOCTYPE html>
<html>
<head>
    <meta charset="utf-8">
    <title>DynamuhSim ROS 2 MCP Server</title>
    <style>
        body { font-family: -apple-system, BlinkMacSystemFont, 'Segoe UI', Roboto, Helvetica, Arial, sans-serif; background: #0f172a; color: #f8fafc; text-align: center; padding: 60px 20px; }
        .card { max-width: 600px; margin: 0 auto; background: #1e293b; border-radius: 12px; padding: 32px; box-shadow: 0 10px 25px rgba(0,0,0,0.5); border: 1px solid #334155; }
        .badge { display: inline-block; padding: 6px 16px; border-radius: 9999px; background: #059669; color: white; font-weight: bold; font-size: 14px; margin-bottom: 20px; }
        h1 { margin: 0 0 10px 0; font-size: 26px; }
        p { color: #94a3b8; font-size: 15px; line-height: 1.6; }
        .code-box { background: #0f172a; padding: 12px; border-radius: 6px; font-family: monospace; color: #38bdf8; font-size: 14px; word-break: break-all; margin: 16px 0; }
    </style>
</head>
<body>
    <div class="card">
        <div class="badge">● SERVER ONLINE</div>
        <h1>DynamuhSim MCP Server</h1>
        <p>Kết nối mạng Tailscale và Server MCP trên WSL2 đang hoạt động bình thường!</p>
        <p><strong>Endpoint cho Cursor / VS Code:</strong></p>
        <div class="code-box">/sse</div>
        <p style="font-size: 13px; color: #64748b;">(Đừng mở /sse trực tiếp trên trình duyệt vì /sse là luồng dữ liệu thời gian thực dành riêng cho AI Agent).</p>
    </div>
</body>
</html>"""
    return HTMLResponse(html_content)


@server.custom_route("/health", methods=["GET"])
async def health_check(request):
    """JSON health check endpoint."""
    return JSONResponse({
        "status": "online",
        "service": "DynamuhSim-ROS2-MCP",
        "workspace": str(WORKSPACE_ROOT),
        "ros_distro": os.environ.get("ROS_DISTRO", "jazzy"),
    })



def _run_shell(cmd: str, cwd: Optional[Path] = None, timeout: int = 60, source_ros: bool = True) -> str:
    """Helper to run a bash command with optional ROS 2 workspace sourcing."""
    effective_cwd = cwd or WORKSPACE_ROOT
    if source_ros and SETUP_ENV_SCRIPT.exists():
        full_cmd = f"source '{SETUP_ENV_SCRIPT}' >/dev/null 2>&1 || true; {cmd}"
    else:
        full_cmd = cmd

    try:
        proc = subprocess.run(
            ["bash", "-c", full_cmd],
            cwd=str(effective_cwd),
            capture_output=True,
            text=True,
            timeout=timeout,
        )
        output = []
        if proc.stdout:
            output.append(proc.stdout)
        if proc.stderr:
            output.append(f"[STDERR]\n{proc.stderr}")
        if proc.returncode != 0:
            output.append(f"[Exit code: {proc.returncode}]")
        return "\n".join(output) if output else "(No output, command succeeded)"
    except subprocess.TimeoutExpired:
        return f"[ERROR] Command timed out after {timeout} seconds."
    except Exception as e:
        return f"[ERROR] Execution failed: {e}"


@server.tool()
def execute_bash(command: str, timeout: int = 60, auto_source_ros: bool = True) -> str:
    """
    Execute an arbitrary bash command in the DynamuhSim workspace.
    
    Args:
        command: The bash command to run (e.g. 'colcon build', 'ls -la', 'ps aux').
        timeout: Maximum duration in seconds to wait for command completion (default 60).
        auto_source_ros: If True, automatically sources ROS 2 Jazzy and DynamuhSim environment.
    """
    return _run_shell(command, timeout=timeout, source_ros=auto_source_ros)


@server.tool()
def ros2_topic_list() -> str:
    """
    List all active ROS 2 topics along with their message types.
    """
    return _run_shell("ros2 topic list -t", timeout=15, source_ros=True)


@server.tool()
def ros2_topic_echo(topic: str, message_count: int = 1, timeout: int = 10) -> str:
    """
    Echo messages from a specified ROS 2 topic.
    
    Args:
        topic: The topic name to echo (e.g. '/cmd_vel', '/odom', '/scan').
        message_count: Number of messages to receive before stopping (default 1).
        timeout: Maximum time to wait for message(s) in seconds (default 10).
    """
    if not topic.startswith("/"):
        topic = f"/{topic}"
    cmd = f"ros2 topic echo {topic} -n {message_count}"
    result = _run_shell(cmd, timeout=timeout, source_ros=True)
    if "[ERROR] Command timed out" in result:
        return f"Timeout waiting for message on topic '{topic}'. Is a publisher active?"
    return result


@server.tool()
def ros2_node_list() -> str:
    """
    List all running ROS 2 nodes in the network.
    """
    return _run_shell("ros2 node list", timeout=15, source_ros=True)


@server.tool()
def ros2_node_info(node_name: str) -> str:
    """
    Get detailed information about a specific ROS 2 node (publishers, subscribers, services).
    
    Args:
        node_name: Full node name (e.g. '/robot_state_publisher' or '/diff_drive_controller').
    """
    if not node_name.startswith("/"):
        node_name = f"/{node_name}"
    return _run_shell(f"ros2 node info {node_name}", timeout=15, source_ros=True)


@server.tool()
def read_file(relative_or_abs_path: str, start_line: int = 1, max_lines: int = 200) -> str:
    """
    Read contents of a file within the workspace.
    
    Args:
        relative_or_abs_path: Path to the file (relative to workspace or absolute).
        start_line: Line number to start reading from (1-based index).
        max_lines: Maximum number of lines to return (default 200).
    """
    path = Path(relative_or_abs_path)
    if not path.is_absolute():
        path = WORKSPACE_ROOT / path

    if not path.exists():
        return f"[ERROR] File not found: {path}"
    if not path.is_file():
        return f"[ERROR] Not a regular file: {path}"

    try:
        lines = []
        with open(path, "r", encoding="utf-8", errors="replace") as f:
            for idx, line in enumerate(f, start=1):
                if idx < start_line:
                    continue
                if idx >= start_line + max_lines:
                    lines.append(f"... (truncated at line {idx - 1}, {max_lines} lines shown)")
                    break
                lines.append(f"{idx}: {line.rstrip('\\n')}")
        return "\n".join(lines) if lines else "(Empty file)"
    except Exception as e:
        return f"[ERROR] Reading file failed: {e}"


@server.tool()
def list_directory(directory: str = ".", max_depth: int = 2) -> str:
    """
    List files and directories up to a specific depth.
    
    Args:
        directory: Relative or absolute path to list (default is workspace root).
        max_depth: Maximum directory recursion depth (default 2).
    """
    target = Path(directory)
    if not target.is_absolute():
        target = WORKSPACE_ROOT / target
    if not target.exists():
        return f"[ERROR] Path not found: {target}"

    cmd = f"find '{target}' -maxdepth {max_depth} -not -path '*/.*' | sort"
    return _run_shell(cmd, timeout=10, source_ros=False)


def main():
    parser = argparse.ArgumentParser(description="DynamuhSim ROS 2 MCP Server")
    parser.add_argument("--host", default=os.getenv("MCP_HOST", "0.0.0.0"), help="Host to bind (default: 0.0.0.0)")
    parser.add_argument("--port", type=int, default=int(os.getenv("MCP_PORT", "8765")), help="Port to listen (default: 8765)")
    parser.add_argument("--transport", choices=["sse", "stdio", "streamable-http"], default="sse", help="Transport mode")
    args = parser.parse_args()

    # Disable DNS rebinding protection so Tailscale IP and magic DNS names can connect without domain rejection
    security_settings = TransportSecuritySettings(enable_dns_rebinding_protection=False)

    print(f"🚀 Starting MCP Server on {args.host}:{args.port} using {args.transport} transport...")
    print(f"📁 Workspace: {WORKSPACE_ROOT}")
    print(f"🔗 SSE URL: http://<tailscale-ip>:{args.port}/sse")
    
    if args.transport == "sse":
        server.run(
            transport="sse",
            host=args.host,
            port=args.port,
            transport_security=security_settings,
        )
    elif args.transport == "streamable-http":
        server.run(
            transport="streamable-http",
            host=args.host,
            port=args.port,
            transport_security=security_settings,
        )
    else:
        server.run(transport="stdio")


if __name__ == "__main__":
    main()
