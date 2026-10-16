import os, sys, time, subprocess, signal, json
import rclpy
from rclpy.node import Node
from rosgraph_msgs.msg import Clock
from geometry_msgs.msg import TwistStamped, PoseStamped
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient

ROOT_DIR = "/home/cuongbien/DynamuhSim"

class CmdVelMonitor(Node):
    def __init__(self):
        super().__init__('cmd_vel_monitor')
        self.cmd_msgs = []
        self.sim_time = 0.0
        self.clock_sub = self.create_subscription(Clock, '/clock', self._clock_cb, 100)
        self.cmd_sub = self.create_subscription(TwistStamped, '/cmd_vel', self._cmd_cb, 100)
        self.nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')

    def _clock_cb(self, msg: Clock):
        self.sim_time = msg.clock.sec + msg.clock.nanosec * 1e-9

    def _cmd_cb(self, msg: TwistStamped):
        w_t = time.time()
        s_t = self.sim_time
        vx = msg.twist.linear.x
        vy = msg.twist.linear.y
        w = msg.twist.angular.z
        self.cmd_msgs.append((w_t, s_t, vx, vy, w))

    def send_goal(self, gx=-7.0, gy=0.0):
        if not self.nav_client.wait_for_server(timeout_sec=10.0):
            return False
        goal = NavigateToPose.Goal()
        goal.pose = PoseStamped()
        goal.pose.header.frame_id = 'map'
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x = float(gx)
        goal.pose.pose.position.y = float(gy)
        goal.pose.pose.orientation.w = 1.0
        for attempt in range(5):
            future = self.nav_client.send_goal_async(goal)
            rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)
            if future.done():
                handle = future.result()
                if handle.accepted:
                    return True
            time.sleep(0.5)
        return False

def run_test(rtf=1.0, duration=15.0):
    domain_id = 42
    os.environ["ROS_DOMAIN_ID"] = str(domain_id)
    env = os.environ.copy()
    env["ROS_DOMAIN_ID"] = str(domain_id)
    env["GZ_PARTITION"] = f"gz_test_rtf_{rtf}"
    
    # Generate test world with given rtf
    world_src = os.path.join(ROOT_DIR, "src/custom_corridor/worlds/corridor_090.sdf")
    with open(world_src) as f:
        w_content = f.read()
    import re
    w_content = re.sub(r"<real_time_factor>.*?</real_time_factor>", f"<real_time_factor>{rtf}</real_time_factor>", w_content)
    test_world = f"/tmp/test_world_rtf_{rtf}.sdf"
    with open(test_world, "w") as f:
        f.write(w_content)
        
    nav2_params = os.path.join(ROOT_DIR, "install/custom_corridor/share/custom_corridor/config/nav2_mppi_pilot.yaml")
    
    procs = []
    # 1. Sim
    sim_cmd = [
        "ros2", "launch", "custom_corridor", "corridor_tb3.launch.py",
        "width:=0.90", f"custom_world:={test_world}",
        "obstacle:=none", "gui:=false", "rviz:=false",
        "robot_model:=mecanum", "x:=-13.0", "y:=0.0", "yaw:=0.0"
    ]
    p_sim = subprocess.Popen(sim_cmd, env=env, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, preexec_fn=os.setsid)
    procs.append(p_sim)
    time.sleep(4.0)
    
    # 2. Nav2
    nav2_log_path = f"/tmp/nav2_rtf_{rtf}.log"
    log_fp = open(nav2_log_path, "w")
    nav2_cmd = [
        "ros2", "launch", "custom_corridor", "nav2_corridor.launch.py",
        "controller:=mppi", f"params_file:={nav2_params}",
        "map:=corridor_090", "robot_model:=mecanum"
    ]
    p_nav = subprocess.Popen(nav2_cmd, env=env, stdout=log_fp, stderr=subprocess.STDOUT, text=True, preexec_fn=os.setsid)
    procs.append(p_nav)
    
    # Wait for Nav2 active
    t0 = time.time()
    while time.time() - t0 < 35.0:
        if os.path.exists(nav2_log_path):
            with open(nav2_log_path) as fp:
                if fp.read().count("Managed nodes are active") >= 2:
                    break
        time.sleep(0.5)
        
    time.sleep(2.0)
    
    rclpy.init()
    node = CmdVelMonitor()
    node.send_goal(-7.0, 0.0)
    
    # Collect for duration
    t_start = time.time()
    while time.time() - t_start < duration:
        rclpy.spin_once(node, timeout_sec=0.05)
        
    # Teardown
    node.destroy_node()
    rclpy.shutdown()
    
    for p in procs:
        try:
            os.killpg(os.getpgid(p.pid), signal.SIGTERM)
        except: pass
    time.sleep(2.0)
    for p in procs:
        try:
            os.killpg(os.getpgid(p.pid), signal.SIGKILL)
        except: pass
    log_fp.close()
    
    # Analyze
    with open(nav2_log_path) as fp:
        nav2_log = fp.read()
        
    loop_lines = [l for l in nav2_log.splitlines() if "missed its desired rate" in l or "Loop time" in l]
    
    # cmd_vel rates
    msgs = node.cmd_msgs
    if len(msgs) > 2:
        wall_dur = msgs[-1][0] - msgs[0][0]
        sim_dur = msgs[-1][1] - msgs[0][1]
        wall_hz = (len(msgs) - 1) / wall_dur if wall_dur > 0 else 0.0
        sim_hz = (len(msgs) - 1) / sim_dur if sim_dur > 0 else 0.0
    else:
        wall_hz, sim_hz = 0.0, 0.0
        
    print(f"=== RTF = {rtf} RESULTS ===")
    print(f"CmdVel msgs count: {len(msgs)}")
    print(f"Wall Hz: {wall_hz:.2f} Hz, Sim Hz: {sim_hz:.2f} Hz")
    print(f"Log loop warnings count: {len(loop_lines)}")
    if loop_lines:
        print("Sample warnings:")
        for l in loop_lines[:5]:
            print("  ", l)
            
if __name__ == '__main__':
    rtf_val = float(sys.argv[1]) if len(sys.argv) > 1 else 1.0
    run_test(rtf=rtf_val)
