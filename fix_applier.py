import re

with open('src/custom_corridor/worlds/school_arena/demo_1/collision_safe_pose_applier.py', 'r') as f:
    content = f.read()

# Add PoseStamped import
content = content.replace('from tf2_msgs.msg import TFMessage', 'from tf2_msgs.msg import TFMessage\nfrom geometry_msgs.msg import PoseStamped')

# Add robot_pose init
init_addition = """
        self.declare_parameter("robot_safe_distance", 0.45)
        self.declare_parameter("human_safe_distance", 0.35)
        self.robot_safe_dist = float(self.get_parameter("robot_safe_distance").value)
        self.human_safe_dist = float(self.get_parameter("human_safe_distance").value)
        self.robot_pose = None
        self.create_subscription(PoseStamped, "/demo/gazebo_robot_pose", self._robot_pose_cb, 10)
"""
content = content.replace('self.last_wait_log = 0.0', 'self.last_wait_log = 0.0' + init_addition)

# Add callback
cb_addition = """
    def _robot_pose_cb(self, msg: PoseStamped) -> None:
        self.robot_pose = (msg.pose.position.x, msg.pose.position.y)
"""
content = content.replace('def _target_cb', cb_addition + '\n    def _target_cb')

# Add tick logic
tick_addition = """
            if previous is not None:
                safe = safe and self.guard.segment_is_free(previous, target)
            
            # Check overlap with robot
            if safe and self.robot_pose is not None:
                d_robot = math.hypot(target[0] - self.robot_pose[0], target[1] - self.robot_pose[1])
                if d_robot < self.robot_safe_dist:
                    safe = False
                    count = self.blocked_count.get(name, 0) + 1
                    self.blocked_count[name] = count
                    if count == 1 or count % 50 == 0:
                        self.get_logger().warn(
                            f"BLOCKED robot collision for {name}: d={d_robot:.2f}m < {self.robot_safe_dist:.2f}m"
                        )
            
            # Check overlap with other humans
            if safe:
                for other_name, other_pose in self.current.items():
                    if other_name == name:
                        continue
                    d_human = math.hypot(target[0] - other_pose[0], target[1] - other_pose[1])
                    if d_human < self.human_safe_dist:
                        safe = False
                        count = self.blocked_count.get(name, 0) + 1
                        self.blocked_count[name] = count
                        if count == 1 or count % 50 == 0:
                            self.get_logger().warn(
                                f"BLOCKED human-human collision {name} <-> {other_name}: d={d_human:.2f}m"
                            )
                        break
"""
content = content.replace('''            if previous is not None:
                safe = safe and self.guard.segment_is_free(previous, target)''', tick_addition)

with open('src/custom_corridor/worlds/school_arena/demo_1/collision_safe_pose_applier.py', 'w') as f:
    f.write(content)
