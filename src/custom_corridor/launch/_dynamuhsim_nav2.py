"""Shared Nav2 actions; map, pose, timing and gate choices belong to each launch."""
import os

from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node


def nav2_parameters(package_share, controller, custom_params, robot_model):
    if robot_model not in ["burger", "tb3", "turtlebot3"]:
        robot_model = "mecanum"
    if custom_params and os.path.exists(custom_params):
        return custom_params
    if controller in ["mppi", "mppi_mecanum", "mppi_omni"]:
        filename = "nav2_mppi_mecanum.yaml" if robot_model == "mecanum" or controller in ["mppi_mecanum", "mppi_omni"] else "nav2_mppi.yaml"
    else:
        filename = "nav2_corridor.yaml"
    return os.path.join(package_share, "config", filename)


def navigation_nodes(nav2_params, map_server_params, amcl_params, cmd_vel_topic=None):
    bt_override = {}
    try:
        bt_share = get_package_share_directory("nav2_bt_navigator")
        default_bt_xml = os.path.join(bt_share, "behavior_trees", "navigate_to_pose_w_replanning_and_recovery.xml")
        if os.path.exists(default_bt_xml):
            bt_override["default_nav_to_pose_bt_xml"] = default_bt_xml
    except Exception:
        pass
    bt_params = [nav2_params, bt_override] if bt_override else [nav2_params]
    specs = [
        ("nav2_map_server", "map_server", map_server_params),
        ("nav2_amcl", "amcl", amcl_params),
        ("nav2_planner", "planner_server", [nav2_params]),
        ("nav2_controller", "controller_server", [nav2_params]),
        ("nav2_bt_navigator", "bt_navigator", bt_params),
        ("nav2_behaviors", "behavior_server", [nav2_params]),
        ("nav2_waypoint_follower", "waypoint_follower", [nav2_params]),
    ]
    return [Node(
        package=package, executable=name, name=name, output="screen", parameters=params,
        **({"remappings": [("cmd_vel", cmd_vel_topic)]}
           if cmd_vel_topic is not None and name in ["controller_server", "behavior_server"] else {}),
    ) for package, name, params in specs]


def lifecycle_manager(stage, node_names):
    return Node(
        package="nav2_lifecycle_manager", executable="lifecycle_manager",
        name="lifecycle_manager_" + stage, output="screen",
        parameters=[dict(use_sim_time=True, autostart=True, bond_timeout=30.0,
                         attempt_respawn_reconnection=True, node_names=node_names)],
    )


def goal_bridge():
    return Node(
        package="custom_corridor", executable="goal_pose_bridge.py", name="goal_pose_bridge",
        output="screen", parameters=[{"use_sim_time": True}],
    )
