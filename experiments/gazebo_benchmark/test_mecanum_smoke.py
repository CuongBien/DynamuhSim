#!/usr/bin/env python3
import os
import sys
import time
import subprocess
import json

def run_test():
    print("=" * 70)
    print("SMOKE TEST: SMALL MECANUM ROBOT IN GAZEBO HARMONIC")
    print("=" * 70)

    env = os.environ.copy()
    env["GALLIUM_DRIVER"] = "d3d12"
    env["MESA_D3D12_DEFAULT_ADAPTER_NAME"] = "NVIDIA"

    cmd = [
        "ros2", "launch", "custom_corridor", "corridor_tb3.launch.py",
        "width:=0.90", "obstacle:=none", "gui:=false", "rviz:=false",
        "robot_model:=mecanum", "x:=-1.0", "y:=0.0", "yaw:=0.0"
    ]

    print("Starting Gazebo with small mecanum robot...")
    proc = subprocess.Popen(cmd, env=env, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True)

    t0 = time.time()
    depth_received = False
    odom_received = False
    init_x, init_y = None, None
    final_x, final_y = None, None

    try:
        # Wait up to 20s for nodes and simulation to initialize
        print("Waiting for simulation startup and topics...")
        time.sleep(8.0)

        # Check topics using gz topic or ros2 topic
        gz_topics_res = subprocess.run(["gz", "topic", "-l"], stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True, timeout=5)
        print("Gz topics:\n", gz_topics_res.stdout)

        # Check odometry
        print("Sampling initial odometry...")
        for _ in range(5):
            res = subprocess.run(
                ["gz", "topic", "-e", "-t", "/odom", "-n", "1"],
                stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True, timeout=5
            )
            if "pose" in res.stdout:
                odom_received = True
                print("Initial /odom sample received!")
                for line in res.stdout.splitlines():
                    if "x:" in line and init_x is None:
                        try:
                            init_x = float(line.split("x:")[1].strip())
                        except: pass
                    if "y:" in line and init_y is None and init_x is not None:
                        try:
                            init_y = float(line.split("y:")[1].strip())
                        except: pass
                break
            time.sleep(1.0)

        # Check depth image
        print("Sampling depth image topic...")
        depth_res = subprocess.run(
            ["gz", "topic", "-e", "-t", "/camera/depth_image", "-n", "1"],
            stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True, timeout=5
        )
        if "width:" in depth_res.stdout or "step:" in depth_res.stdout:
            depth_received = True
            print("Depth image successfully received from generic depth sensor!")

        # Send omnidirectional cmd_vel: vx=0.3, vy=0.2 for 3 seconds
        print("Publishing omnidirectional cmd_vel (vx=0.3 m/s, vy=0.2 m/s)...")
        for _ in range(15):
            subprocess.run([
                "gz", "topic", "-t", "/cmd_vel", "-m", "gz.msgs.Twist",
                "-p", "linear: {x: 0.3, y: 0.2}"
            ], timeout=2)
            time.sleep(0.2)

        # Sample final odometry
        print("Sampling final odometry after omnidirectional motion...")
        for _ in range(5):
            res = subprocess.run(
                ["gz", "topic", "-e", "-t", "/odom", "-n", "1"],
                stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True, timeout=5
            )
            if "pose" in res.stdout:
                lines = res.stdout.splitlines()
                for line in lines:
                    if "x:" in line and final_x is None:
                        try:
                            final_x = float(line.split("x:")[1].strip())
                        except: pass
                    if "y:" in line and final_y is None and final_x is not None:
                        try:
                            final_y = float(line.split("y:")[1].strip())
                        except: pass
                break
            time.sleep(1.0)

        # Check renderer log
        ogre_log_path = os.path.expanduser("~/.gz/rendering/ogre2.log")
        renderer_info = "Unknown"
        if os.path.exists(ogre_log_path):
            with open(ogre_log_path, "r") as f:
                lines = f.readlines()
                for line in lines:
                    if "GL_RENDERER" in line or "Device Name" in line:
                        renderer_info = line.strip()
                        print(f"Renderer verification: {renderer_info}")

        dx = (final_x - init_x) if (final_x is not None and init_x is not None) else 0.0
        dy = (final_y - init_y) if (final_y is not None and init_y is not None) else 0.0

        print("=" * 70)
        print("RESULTS:")
        print(f"Odom received: {odom_received}")
        print(f"Depth received: {depth_received}")
        print(f"Initial Pose: x={init_x}, y={init_y}")
        print(f"Final Pose:   x={final_x}, y={final_y}")
        print(f"Displacement: dx={dx:.4f} m, dy={dy:.4f} m")
        print(f"Renderer: {renderer_info}")
        print("=" * 70)

        results = {
            "odom_received": odom_received,
            "depth_received": depth_received,
            "init_x": init_x,
            "init_y": init_y,
            "final_x": final_x,
            "final_y": final_y,
            "dx": dx,
            "dy": dy,
            "renderer": renderer_info
        }

        with open("experiments/gazebo_benchmark/mecanum_smoke_test_results.json", "w") as f:
            json.dump(results, f, indent=2)

    finally:
        print("Shutting down Gazebo...")
        proc.terminate()
        try:
            proc.wait(timeout=5)
        except subprocess.TimeoutExpired:
            proc.kill()
        subprocess.run(["pkill", "-9", "-f", "gz sim"], stderr=subprocess.DEVNULL)
        subprocess.run(["pkill", "-9", "-f", "parameter_bridge"], stderr=subprocess.DEVNULL)
        print("Cleaned up processes.")

if __name__ == "__main__":
    run_test()
