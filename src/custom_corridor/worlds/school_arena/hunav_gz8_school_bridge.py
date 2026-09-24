#!/usr/bin/env python3
import argparse
import copy
import math
import time
import xml.etree.ElementTree as ET
from pathlib import Path

import yaml
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point, Pose, Transform, TransformStamped
from tf2_msgs.msg import TFMessage
from hunav_msgs.msg import Agent, Agents
from hunav_msgs.srv import ComputeAgents


def yaw_from_q(q):
    siny = 2.0 * (q.w * q.z + q.x * q.y)
    cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny, cosy)


def set_q_from_yaw(q, yaw):
    q.x = 0.0
    q.y = 0.0
    q.z = math.sin(yaw * 0.5)
    q.w = math.cos(yaw * 0.5)


def clamp(v, lo, hi):
    return max(lo, min(hi, v))


class StaticObstacleMap:
    """Extract simple static collision AABBs directly from the user's SDF."""

    def __init__(self, sdf_path, human_names):
        self.boxes = []
        root = ET.parse(sdf_path).getroot()
        world = root.find('world')
        if world is None:
            raise RuntimeError('No <world> found in SDF')

        for model in world.findall('model'):
            name = model.get('name', '')
            if name in human_names or name == 'ground':
                continue
            if (model.findtext('static', 'false').strip().lower() != 'true'):
                continue

            mp = self._pose6(model.findtext('pose', '0 0 0 0 0 0'))
            mx, my, mz, _, _, myaw = mp

            for link in model.findall('link'):
                lp = self._pose6(link.findtext('pose', '0 0 0 0 0 0'))
                lx, ly, lz, _, _, lyaw = lp
                for collision in link.findall('collision'):
                    cp = self._pose6(collision.findtext('pose', '0 0 0 0 0 0'))
                    cx, cy, cz, _, _, cyaw = cp
                    yaw = myaw + lyaw + cyaw
                    geom = collision.find('geometry')
                    if geom is None:
                        continue

                    # school_arena.sdf uses axis-aligned geometry. For safety,
                    # skip rotated collision geometry rather than pretending it is axis aligned.
                    if abs(yaw) > 1e-6:
                        continue

                    center = (mx + lx + cx, my + ly + cy, mz + lz + cz)
                    box = geom.find('box')
                    cyl = geom.find('cylinder')
                    sphere = geom.find('sphere')
                    if box is not None:
                        size = [float(v) for v in box.findtext('size').split()]
                        if len(size) == 3:
                            self._add_aabb(name, center, size[0], size[1], size[2])
                    elif cyl is not None:
                        r = float(cyl.findtext('radius'))
                        h = float(cyl.findtext('length'))
                        self._add_aabb(name, center, 2*r, 2*r, h)
                    elif sphere is not None:
                        r = float(sphere.findtext('radius'))
                        self._add_aabb(name, center, 2*r, 2*r, 2*r)

    @staticmethod
    def _pose6(text):
        vals = [float(v) for v in text.split()]
        vals += [0.0] * (6 - len(vals))
        return vals[:6]

    def _add_aabb(self, name, center, sx, sy, sz):
        x, y, z = center
        self.boxes.append((name, x-sx/2, x+sx/2, y-sy/2, y+sy/2, z-sz/2, z+sz/2))

    def nearby_points(self, x, y, max_dist=5.0):
        # Match HuNavActorPlugin_fortress::getHumanObstacles(): HuNav receives
        # one closest obstacle point per pedestrian.  Supplying one point from
        # every nearby wall segment multiplies the SFM obstacle force and can
        # create artificial equilibria in front of benches and doorways.
        nearest = None
        for _, xmin, xmax, ymin, ymax, _, _ in self.boxes:
            px = clamp(x, xmin, xmax)
            py = clamp(y, ymin, ymax)
            d = math.hypot(px-x, py-y)
            if d < max_dist and (nearest is None or d < nearest[0]):
                nearest = (d, px, py)
        if nearest is None:
            return []
        p = Point()
        p.x = nearest[1]
        p.y = nearest[2]
        p.z = 0.0
        return [p]

    def constrain_motion(self, x0, y0, x1, y1, radius):
        """Slide a planar step along expanded static collision AABBs."""
        margin = radius + 0.02
        for _, xmin, xmax, ymin, ymax, _, _ in self.boxes:
            xmin -= margin
            xmax += margin
            ymin -= margin
            ymax += margin

            # Do not trap an agent that was initialized too close to an
            # obstacle; allow its social force to move it back out.
            if xmin < x0 < xmax and ymin < y0 < ymax:
                continue
            if not (xmin < x1 < xmax and ymin < y1 < ymax):
                continue

            # Keep the tangential component so pedestrians can flow around a
            # bench / bin instead of deadlocking at its expanded AABB.
            candidates = []
            clearance = 0.005
            if x0 <= xmin:
                candidates.append((abs(x1 - xmin), xmin - clearance, y1))
            if x0 >= xmax:
                candidates.append((abs(x1 - xmax), xmax + clearance, y1))
            if y0 <= ymin:
                candidates.append((abs(y1 - ymin), x1, ymin - clearance))
            if y0 >= ymax:
                candidates.append((abs(y1 - ymax), x1, ymax + clearance))
            if candidates:
                _, x1, y1 = min(candidates, key=lambda item: item[0])
        return x1, y1


class SchoolHuNavBridge(Node):
    def __init__(self, scenario_path, sdf_path):
        super().__init__('school_hunav_gz8_bridge')

        self.declare_parameter('world_name', 'school_arena')
        self.declare_parameter('robot_name', 'robot')
        self.declare_parameter('update_hz', 10.0)
        # The proxy model origin is its ground contact point.  Keep all HuNav
        # motion planar; Gazebo must never feed physics-induced Z motion back
        # into the social-force simulation.
        self.declare_parameter('human_z', 0.0)
        self.declare_parameter('obstacle_range', 5.0)
        self.declare_parameter('max_yaw_rate', 2.5)
        # 0.45 m lets a pedestrian sidestep a bin or bench while the hard AABB
        # guard below still prevents crossing corridor walls.
        self.declare_parameter('route_half_width', 0.45)
        self.declare_parameter('target_topic', '/school_hunav/target_poses')

        self.world_name = self.get_parameter('world_name').value
        self.robot_name = self.get_parameter('robot_name').value
        self.update_hz = float(self.get_parameter('update_hz').value)
        self.human_z = float(self.get_parameter('human_z').value)
        self.obstacle_range = float(self.get_parameter('obstacle_range').value)
        self.max_yaw_rate = float(self.get_parameter('max_yaw_rate').value)
        self.route_half_width = float(self.get_parameter('route_half_width').value)
        self.target_topic = self.get_parameter('target_topic').value

        self.cfg = self._load_scenario(scenario_path)
        self.agent_names = list(self.cfg['agents'])
        self.agent_cfg = {name: self.cfg[name] for name in self.agent_names}
        self.goals = {int(k): v for k, v in self.cfg['global_goals'].items()}
        self.obstacles = StaticObstacleMap(sdf_path, set(self.agent_names))
        self.routes = {
            name: self._route_points(name)
            for name in self.agent_names
        }

        robot_initial = Transform()
        robot_initial.translation.x = -13.5
        robot_initial.translation.y = -8.0
        robot_initial.translation.z = 0.01
        robot_initial.rotation.w = 1.0
        self.live_tf = {self.robot_name: robot_initial}
        # ros_gz_bridge currently converts gz.msgs.Pose_V to TFMessage without
        # preserving Pose.name in child_frame_id. Keep a validated index map as
        # a fallback for this school world. Pose_V entity ordering is stable for
        # the lifetime of a Gazebo server.
        self.unnamed_pose_slots = {}
        self.reported_unnamed_pose_fallback = False
        self.prev_state = {}
        # Human state is advanced from the previous HuNav response, not from the
        # Gazebo pose echoed back across the Humble/Jazzy DDS boundary. That echo
        # can arrive late and alternate between old samples, creating a two-pose
        # feedback oscillation. Gazebo remains the visualization sink; the robot
        # pose is still read live so human-robot interaction remains effective.
        self.commanded_humans = {}
        self.agent_goals = {}
        self.behavior_states = {}
        self.last_missing_log = 0.0
        self.pending_compute = None

        pose_topic = f'/world/{self.world_name}/dynamic_pose/info'

        self.create_subscription(TFMessage, pose_topic, self._pose_cb, 10)
        self.target_pub = self.create_publisher(TFMessage, self.target_topic, 10)
        self.compute_cli = self.create_client(ComputeAgents, '/compute_agents')

        self.get_logger().info(f'Gazebo pose topic: {pose_topic}')
        self.get_logger().info(f'Target pose topic: {self.target_topic}')
        self.get_logger().info(f'HuNav service: /compute_agents')
        self.get_logger().info('Configured humans: ' + ', '.join(self.agent_names))
        self.get_logger().info(f'Loaded {len(self.obstacles.boxes)} static collision AABBs from SDF')

        self.timer = self.create_timer(1.0 / self.update_hz, self._tick)

    @staticmethod
    def _load_scenario(path):
        with open(path, 'r', encoding='utf-8') as f:
            doc = yaml.safe_load(f)
        return doc['hunav_loader']['ros__parameters']

    def _route_points(self, name):
        config = self.agent_cfg[name]
        initial = config['init_pose']
        points = [(float(initial['x']), float(initial['y']))]
        for goal_id in config.get('goals', []):
            goal = self.goals[int(goal_id)]
            points.append((float(goal['x']), float(goal['y'])))
        return points

    def _constrain_to_route(self, name, x, y):
        """Keep SFM avoidance inside the walkable tube around its route."""
        route = self.routes[name]
        if len(route) < 2:
            return x, y

        best_x = route[0][0]
        best_y = route[0][1]
        best_d2 = float('inf')
        for (ax, ay), (bx, by) in zip(route, route[1:]):
            sx = bx - ax
            sy = by - ay
            length2 = sx * sx + sy * sy
            if length2 < 1e-12:
                continue
            t = clamp(((x - ax) * sx + (y - ay) * sy) / length2, 0.0, 1.0)
            px = ax + t * sx
            py = ay + t * sy
            d2 = (x - px) ** 2 + (y - py) ** 2
            if d2 < best_d2:
                best_d2 = d2
                best_x = px
                best_y = py

        distance = math.sqrt(best_d2)
        if distance <= self.route_half_width or distance < 1e-12:
            return x, y
        scale = self.route_half_width / distance
        return (
            best_x + (x - best_x) * scale,
            best_y + (y - best_y) * scale,
        )

    def _pose_cb(self, msg):
        found_named_transform = False
        for tf in msg.transforms:
            key = tf.child_frame_id.strip('/')
            if key:
                self.live_tf[key] = tf.transform
                found_named_transform = True

        if found_named_transform:
            return

        # Gazebo Sim 8 -> ROS Jazzy maps Pose_V poses to TransformStamped, but
        # the current converter leaves child_frame_id empty. Discover the model
        # slots from the configured initial human poses, then keep using those
        # stable slots. Robot poses are detected afresh in every message because
        # dynamic_pose omits a stationary robot and shifts the later indices.
        if not self.unnamed_pose_slots:
            available = set(range(len(msg.transforms)))
            discovered = {}
            matched_initial_poses = True

            for name in self.agent_names:
                initial = self.agent_cfg[name]['init_pose']
                target_x = float(initial['x'])
                target_y = float(initial['y'])
                candidates = []
                for index in available:
                    tr = msg.transforms[index].transform
                    distance = math.hypot(
                        tr.translation.x - target_x,
                        tr.translation.y - target_y,
                    )
                    candidates.append((distance, index))

                if not candidates:
                    matched_initial_poses = False
                    break
                distance, index = min(candidates)
                if distance > 0.5:
                    matched_initial_poses = False
                    break
                discovered[name] = index
                available.remove(index)

            if not matched_initial_poses:
                # The adapter may be restarted while the simulation is already
                # running, after agents have left their configured start poses.
                # This school SDF declares its eight human models first and in
                # the same order as the YAML agent list, so recover those stable
                # Pose_V slots directly.
                if len(msg.transforms) < len(self.agent_names):
                    return
                discovered = {
                    name: index
                    for index, name in enumerate(self.agent_names)
                }
                available = set(
                    range(len(self.agent_names), len(msg.transforms))
                )

            self.unnamed_pose_slots = discovered

            if not self.reported_unnamed_pose_fallback:
                mapping = ', '.join(
                    f'{name}={index}'
                    for name, index in self.unnamed_pose_slots.items()
                )
                self.get_logger().warn(
                    'Gazebo TF messages have empty child_frame_id; '
                    f'using validated human pose slots: {mapping}; '
                    'robot pose is detected live with spawn-pose fallback'
                )
                self.reported_unnamed_pose_fallback = True

        largest_index = max(self.unnamed_pose_slots.values())
        if largest_index >= len(msg.transforms):
            self.unnamed_pose_slots = {}
            return

        for name, index in self.unnamed_pose_slots.items():
            self.live_tf[name] = msg.transforms[index].transform

        # A stationary model is absent from Gazebo's dynamic_pose message, so a
        # cached robot index is unsafe. Detect the robot model pose in each sample
        # and retain the last good pose when it is omitted. Human model poses are
        # at z=0.82; robot child-link transforms are local and close to (0, 0).
        human_indices = set(self.unnamed_pose_slots.values())
        robot_candidates = []
        for index, stamped in enumerate(msg.transforms):
            if index in human_indices:
                continue
            tr = stamped.transform
            horizontal_distance = math.hypot(
                tr.translation.x,
                tr.translation.y,
            )
            if -0.1 <= tr.translation.z <= 0.5 and horizontal_distance > 1.0:
                robot_candidates.append((horizontal_distance, tr))
        if robot_candidates:
            _, robot_transform = max(robot_candidates, key=lambda item: item[0])
            self.live_tf[self.robot_name] = robot_transform

    def _find_transform(self, model_name):
        # Prefer exact model frame. Avoid accidentally selecting model/base_link.
        if model_name in self.live_tf:
            return self.live_tf[model_name]
        candidates = []
        for key, tr in self.live_tf.items():
            if key.endswith('/' + model_name) or key.endswith('::' + model_name):
                candidates.append((len(key), tr))
        if candidates:
            candidates.sort(key=lambda x: x[0])
            return candidates[0][1]
        return None

    def _kinematics(self, name, tr, max_speed=None):
        now = time.monotonic()
        x = tr.translation.x
        y = tr.translation.y
        yaw = yaw_from_q(tr.rotation)
        vx = vy = wz = 0.0
        old = self.prev_state.get(name)
        if old is not None:
            ox, oy, oyaw, ot = old
            dt = now - ot
            if dt > 1e-4:
                vx = (x - ox) / dt
                vy = (y - oy) / dt
                dyaw = math.atan2(math.sin(yaw-oyaw), math.cos(yaw-oyaw))
                wz = dyaw / dt
        speed = math.hypot(vx, vy)
        if max_speed is not None and speed > max_speed and speed > 1e-9:
            scale = max_speed / speed
            vx *= scale
            vy *= scale
        self.prev_state[name] = (x, y, yaw, now)
        return x, y, yaw, vx, vy, wz

    def _build_human(self, name, tr):
        c = self.agent_cfg[name]
        desired_velocity = float(c.get('max_vel', 1.0))
        commanded = self.commanded_humans.get(name)
        if commanded is None:
            x, y, yaw, vx, vy, wz = self._kinematics(
                name, tr, max_speed=desired_velocity
            )
        else:
            x, y, yaw, vx, vy, wz = commanded

        a = Agent()
        a.id = int(c['id'])
        a.type = Agent.PERSON
        a.skin = int(c.get('skin', 0))
        a.name = name
        a.group_id = int(c.get('group_id', -1))
        a.position.position.x = x
        a.position.position.y = y
        a.position.position.z = self.human_z
        a.yaw = yaw
        set_q_from_yaw(a.position.orientation, yaw)
        a.velocity.linear.x = vx
        a.velocity.linear.y = vy
        a.velocity.angular.z = wz
        a.linear_vel = math.hypot(vx, vy)
        a.angular_vel = wz
        a.desired_velocity = desired_velocity
        a.radius = float(c.get('radius', 0.3))
        a.goal_radius = float(c.get('goal_radius', 0.35))
        a.cyclic_goals = bool(c.get('cyclic_goals', True))

        b = c.get('behavior', {})
        behavior_map = {
            'Regular': a.behavior.BEH_REGULAR,
            'Impassive': a.behavior.BEH_IMPASSIVE,
            'Surprised': a.behavior.BEH_SURPRISED,
            'Scared': a.behavior.BEH_SCARED,
            'Curious': a.behavior.BEH_CURIOUS,
            'Threatening': a.behavior.BEH_THREATENING,
        }
        a.behavior.type = behavior_map.get(str(b.get('type', 'Regular')), a.behavior.BEH_REGULAR)
        a.behavior.configuration = int(b.get('configuration', 0))
        a.behavior.duration = float(b.get('duration', 5.0))
        a.behavior.once = bool(b.get('once', False))
        a.behavior.vel = float(b.get('vel', a.desired_velocity))
        a.behavior.dist = float(b.get('dist', 3.0))
        a.behavior.goal_force_factor = float(b.get('goal_force_factor', 2.0))
        a.behavior.obstacle_force_factor = float(b.get('obstacle_force_factor', 10.0))
        a.behavior.social_force_factor = float(b.get('social_force_factor', 5.0))
        a.behavior.other_force_factor = float(b.get('other_force_factor', 20.0))

        if name in self.behavior_states:
            a.behavior.state = self.behavior_states[name]

        if name not in self.agent_goals:
            initial_goals = []
            for gid in c.get('goals', []):
                g = self.goals[int(gid)]
                p = Pose()
                p.position.x = float(g['x'])
                p.position.y = float(g['y'])
                p.orientation.w = 1.0
                initial_goals.append(p)
            self.agent_goals[name] = initial_goals
        a.goals = copy.deepcopy(self.agent_goals[name])

        # This mirrors the official Fortress wrapper: give HuNavSim nearby
        # collision points so obstacle force is computed from the real school SDF.
        a.closest_obs = self.obstacles.nearby_points(x, y, self.obstacle_range)
        return a

    def _build_robot(self, tr):
        x, y, yaw, vx, vy, wz = self._kinematics(self.robot_name, tr)
        r = Agent()
        r.id = -1
        r.type = Agent.ROBOT
        r.name = self.robot_name
        r.group_id = -1
        r.position.position.x = x
        r.position.position.y = y
        r.position.position.z = tr.translation.z
        r.yaw = yaw
        set_q_from_yaw(r.position.orientation, yaw)
        r.velocity.linear.x = vx
        r.velocity.linear.y = vy
        r.velocity.angular.z = wz
        r.linear_vel = math.hypot(vx, vy)
        r.angular_vel = wz
        r.radius = 0.35
        return r

    def _tick(self):
        if not self.compute_cli.service_is_ready():
            now = time.monotonic()
            if now - self.last_missing_log > 2.0:
                self.get_logger().warn('Waiting for /compute_agents...')
                self.last_missing_log = now
            return

        if self.pending_compute is not None:
            if not self.pending_compute.done():
                return
            try:
                res = self.pending_compute.result()
            except Exception as e:
                self.get_logger().error(f'/compute_agents failed: {e}')
                self.pending_compute = None
                return
            self.pending_compute = None
            if res is not None:
                self._apply_updated_agents(res.updated_agents)

        robot_tr = self._find_transform(self.robot_name)
        missing = [n for n in self.agent_names if self._find_transform(n) is None]
        if robot_tr is None or missing:
            now = time.monotonic()
            if now - self.last_missing_log > 2.0:
                self.get_logger().warn(
                    f'Waiting live Gazebo poses. robot={self.robot_name}: {robot_tr is not None}; '
                    f'missing humans={missing}'
                )
                self.last_missing_log = now
            return

        msg = Agents()
        msg.header.frame_id = 'world'
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.agents = [self._build_human(n, self._find_transform(n)) for n in self.agent_names]

        req = ComputeAgents.Request()
        req.current_agents = msg
        req.robot = self._build_robot(robot_tr)
        self.pending_compute = self.compute_cli.call_async(req)

    def _apply_updated_agents(self, agents_msg):
        targets = TFMessage()
        stamp = self.get_clock().now().to_msg()

        for a in agents_msg.agents:
            if a.name not in self.agent_cfg:
                continue

            # HuNav rotates / removes reached goals in the returned Agent. Keep
            # that state for the next request, as the official Gazebo wrapper
            # does; rebuilding the YAML goal list every tick makes pedestrians
            # oscillate around the first waypoint instead of following a route.
            self.agent_goals[a.name] = copy.deepcopy(list(a.goals))
            self.behavior_states[a.name] = a.behavior.state

            commanded = self.commanded_humans.get(a.name)
            current = self._find_transform(a.name)
            if commanded is not None:
                current_x, current_y, current_yaw, _, _, _ = commanded
            elif current is not None:
                current_x = current.translation.x
                current_y = current.translation.y
                current_yaw = yaw_from_q(current.rotation)
            else:
                continue

            target_x = a.position.position.x
            target_y = a.position.position.y
            dx = target_x - current_x
            dy = target_y - current_y
            distance = math.hypot(dx, dy)

            # Guard against a bad SFM timestep / velocity feedback causing a
            # single request to throw a model outside the school. Normal motion
            # remains untouched; only implausibly large steps are limited.
            desired_velocity = float(
                self.agent_cfg[a.name].get('max_vel', 1.0)
            )
            # HuNav normally returns one integration step.  A delayed service
            # response can occasionally contain a larger step, so enforce the
            # configured walking speed on the pose itself (not only on the
            # velocity reported in the next request).
            max_step = max(0.02, 1.05 * desired_velocity / self.update_hz)
            if distance > max_step and distance > 1e-9:
                scale = max_step / distance
                target_x = current_x + dx * scale
                target_y = current_y + dy * scale

            target_x, target_y = self._constrain_to_route(
                a.name, target_x, target_y
            )
            target_x, target_y = self.obstacles.constrain_motion(
                current_x,
                current_y,
                target_x,
                target_y,
                float(self.agent_cfg[a.name].get('radius', 0.3)),
            )

            vx = (target_x - current_x) * self.update_hz
            vy = (target_y - current_y) * self.update_hz
            dyaw = math.atan2(
                math.sin(a.yaw - current_yaw),
                math.cos(a.yaw - current_yaw),
            )
            max_yaw_step = self.max_yaw_rate / self.update_hz
            yaw_step = clamp(dyaw, -max_yaw_step, max_yaw_step)
            target_yaw = current_yaw + yaw_step
            target_yaw = math.atan2(math.sin(target_yaw), math.cos(target_yaw))
            wz = yaw_step * self.update_hz
            self.commanded_humans[a.name] = (
                target_x, target_y, target_yaw, vx, vy, wz
            )

            target = TransformStamped()
            target.header.stamp = stamp
            target.header.frame_id = 'world'
            target.child_frame_id = a.name
            target.transform.translation.x = target_x
            target.transform.translation.y = target_y
            target.transform.translation.z = self.human_z
            set_q_from_yaw(target.transform.rotation, target_yaw)
            targets.transforms.append(target)

        if targets.transforms:
            # Publish all targets in one DDS sample. A small Jazzy-side applier
            # performs the Gazebo service calls locally, avoiding delayed and
            # out-of-order Humble<->Jazzy service responses.
            self.target_pub.publish(targets)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--scenario', required=True)
    ap.add_argument('--sdf', required=True)
    args, ros_args = ap.parse_known_args()

    rclpy.init(args=ros_args)
    node = SchoolHuNavBridge(args.scenario, args.sdf)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
