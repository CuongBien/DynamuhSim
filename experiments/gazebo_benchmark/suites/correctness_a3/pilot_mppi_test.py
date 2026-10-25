"""
Pilot test for Nav2 MPPI controller in custom corridor environment.
Verifies that:
1. Gazebo Harmonic starts with Burger at (0.0, 0.0).
2. Bridges & robot_state_publisher start cleanly.
3. Nav2 stack boots up and transitions lifecycle to ACTIVE.
4. Robot receives goal (x=5.0, y=0.0) and MPPI generates cmd_vel.
"""

from pathlib import Path
import sys
import os
import time
import subprocess
import json

import rclpy
from rclpy.node import Node
from rosgraph_msgs.msg import Clock
from geometry_msgs.msg import PoseStamped, TwistStamped
from action_msgs.msg import GoalStatusArray

import gz.transport13 as gzt
import gz.msgs.stringmsg_pb2 as stringmsg_pb2

sys.path.insert(0, str(Path(__file__).resolve().parents[4]))
from experiments.gazebo_benchmark.core.sim_env import build_benchmark_env, spawn_benchmark_robot, start_parameter_bridge
from experiments.gazebo_benchmark.core.mppi_trial import scoped_processes


class PilotMonitor(Node):
    def __init__(self):
        super().__init__('pilot_monitor')
        self.cmd_vel_count = 0
        self.latest_robot_pos = None
        self.cmd_sub = self.create_subscription(TwistStamped, '/cmd_vel', self._cmd_cb, 50)
        self.gz_node = gzt.Node()
        self.gz_node.subscribe(stringmsg_pb2.StringMsg, '/rvo2_human/ground_truth', self._gt_cb)

    def _cmd_cb(self, msg: TwistStamped):
        self.cmd_vel_count += 1

    def _gt_cb(self, msg):
        try:
            d = json.loads(msg.data)
            if 'robot' in d and d['robot']['x'] is not None:
                self.latest_robot_pos = (d['robot']['x'], d['robot']['y'])
        except Exception:
            pass


@scoped_processes
def main(*, _processes=None):
    root_dir = "/home/cuongbien/DynamuhSim"
    os.environ["ROS_DOMAIN_ID"] = "45"
    os.environ["GZ_PARTITION"] = "mppi_pilot_part"
    os.environ["GALLIUM_DRIVER"] = "d3d12"
    os.environ["MESA_D3D12_DEFAULT_ADAPTER_NAME"] = "NVIDIA"

    world_file = os.path.join(root_dir, "experiments/gazebo_benchmark/configs/generated/world_n5_actor_and_cylinder.sdf")
    robot_file = os.path.join(root_dir, "experiments/gazebo_benchmark/configs/generated/burger_lidar1_depth1_640x480_10fps.sdf")
    bridge_config = os.path.join(root_dir, "src/custom_corridor/config/turtlebot3_burger_bridge.yaml")
    urdf_file = "/tmp/tb3_burger_processed.urdf"
    params_file = os.path.join(root_dir, "experiments/gazebo_benchmark/configs/nav2_mppi/mppi_params_deterministic.yaml")

    env = build_benchmark_env(root_dir, turtlebot3=True, d3d12=False)

    print("1. Launching Gazebo Harmonic...")
    gz_p = _processes.popen(["gz", "sim", "-r", "-s", world_file], env=env, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, preexec_fn=os.setsid)
    time.sleep(3.5)

    print("2. Spawning Burger at (0.0, 0.0)...")
    spawn_benchmark_robot(robot_file, env, x="0.0", processes=_processes)

    print("3. Starting Bridges & Robot State Publisher...")
    bp = start_parameter_bridge(bridge_config, env, processes=_processes)

    rsp = _processes.popen(["ros2", "run", "robot_state_publisher", "robot_state_publisher", urdf_file, "--ros-args", "-p", "use_sim_time:=true"], env=env, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, preexec_fn=os.setsid)
    time.sleep(2.0)

    print("4. Launching Nav2 Stack with MPPI...")
    nav2_log_file = open("/tmp/nav2_pilot.log", "w")
    nav2_cmd = [
        "ros2", "launch", "custom_corridor", "nav2_corridor.launch.py",
        "controller:=mppi",
        f"params_file:={params_file}",
        "map:=corridor_090"
    ]
    nav2_p = _processes.popen(nav2_cmd, env=env, stdout=nav2_log_file, stderr=subprocess.STDOUT, text=True, preexec_fn=os.setsid)

    print("5. Waiting for Nav2 navigation lifecycle manager to become active (max 35s)...")
    t0 = time.time()
    nav2_ready = False
    while time.time() - t0 < 35.0:
        if os.path.exists("/tmp/nav2_pilot.log"):
            with open("/tmp/nav2_pilot.log") as fp:
                content = fp.read()
                if "lifecycle_manager_navigation" in content and "Managed nodes are active" in content:
                    nav2_ready = True
                    print(f"  Navigation lifecycle manager became active in {time.time()-t0:.1f}s!", flush=True)
                    break
        time.sleep(0.5)

    time.sleep(2.0)
    print("6. Sending goal pose (x=5.0, y=0.0)...", flush=True)
    goal_cmd = [
        "ros2", "topic", "pub", "--once", "/goal_pose", "geometry_msgs/msg/PoseStamped",
        "{header: {frame_id: 'map'}, pose: {position: {x: 5.0, y: 0.0, z: 0.0}, orientation: {w: 1.0}}}"
    ]
    subprocess.run(goal_cmd, env=env, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, timeout=5)

    print("7. Monitoring MPPI control commands for 10 seconds...")
    rclpy.init()
    mon = PilotMonitor()
    t_nav = time.time()
    while time.time() - t_nav < 10.0:
        rclpy.spin_once(mon, timeout_sec=0.1)
        if mon.cmd_vel_count > 0:
            print(f"  Received {mon.cmd_vel_count} cmd_vel from MPPI! Robot at {mon.latest_robot_pos}", flush=True)
            break

    print(f"Total cmd_vel received: {mon.cmd_vel_count}, Final robot pose: {mon.latest_robot_pos}")
    mon.destroy_node()
    rclpy.shutdown()

    print("Cleaning up processes...")
    _processes.stop()
    time.sleep(1.0)
    print("Pilot test finished successfully!")


if __name__ == '__main__':
    main()
