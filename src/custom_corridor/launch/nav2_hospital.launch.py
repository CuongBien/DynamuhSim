import os
import sys

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, SetEnvironmentVariable, TimerAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

# Resolve from either the source tree or an installed package share.
sys.path.insert(0, os.path.dirname(os.path.realpath(__file__)))
from _dynamuhsim_nav2 import nav2_parameters, navigation_nodes, lifecycle_manager, goal_bridge

def launch_setup(context, *args, **kwargs):
    package_share = get_package_share_directory("custom_corridor")

    nav2_params = nav2_parameters(
        package_share,
        LaunchConfiguration("controller").perform(context).lower().strip(),
        LaunchConfiguration("params_file").perform(context).strip(),
        LaunchConfiguration("robot_model").perform(context).lower().strip(),
    )

    map_name = LaunchConfiguration("map").perform(context).strip()
    map_override = {}

    if map_name:
        if os.path.isabs(map_name):
            if not os.path.isfile(map_name):
                raise RuntimeError(f"Map YAML does not exist: {map_name}")
            resolved_map = map_name
        else:
            candidates = [
                os.path.join(package_share, "maps", f"{map_name}.yaml"),
                os.path.join(package_share, "maps", map_name),
                os.path.join(package_share, "maps", "hospital", f"{map_name}.yaml"),
                os.path.join(package_share, "maps", "hospital", map_name),
            ]
            clean_name = map_name.replace("hospital/", "").strip()
            if clean_name != map_name:
                candidates.extend([
                    os.path.join(package_share, "maps", "hospital", f"{clean_name}.yaml"),
                    os.path.join(package_share, "maps", "hospital", clean_name),
                ])
            # Fallback to hospital_easy.yaml if medium or hard specific map is requested but shares identical geometry
            if any(k in map_name for k in ("medium", "hard", "easy")):
                candidates.append(os.path.join(package_share, "maps", "hospital", "hospital_easy.yaml"))

            resolved_map = None
            for cand in candidates:
                if os.path.isfile(cand):
                    resolved_map = cand
                    break

            if resolved_map is None:
                raise RuntimeError(
                    f"Map '{map_name}' was not found. Checked:\n" +
                    "\n".join(f"  {c}" for c in candidates)
                )
        map_override["yaml_filename"] = resolved_map
        print(f"[nav2_corridor] Using map: {resolved_map}")
    else:
        resolved_map = os.path.join(
            package_share, "maps", "hospital", "hospital_easy.yaml")
        map_override["yaml_filename"] = resolved_map
        print(f"[nav2_corridor] Using default map: {resolved_map}")

    if not os.path.isfile(map_override["yaml_filename"]):
        raise RuntimeError(
            f"Resolved map YAML does not exist: {map_override['yaml_filename']}")

    map_server_params = [nav2_params, map_override]

    initial_x = float(LaunchConfiguration("initial_x").perform(context))
    initial_y = float(LaunchConfiguration("initial_y").perform(context))
    initial_yaw = float(LaunchConfiguration("initial_yaw").perform(context))

    amcl_override = {
        "initial_pose": {
            "x": initial_x,
            "y": initial_y,
            "z": 0.0,
            "yaw": initial_yaw,
        }
    }

    core_nodes = navigation_nodes(nav2_params, map_server_params, [nav2_params, amcl_override])

    amcl_lifecycle_manager = TimerAction(
        period=4.0,
        actions=[
            lifecycle_manager('localization', ['map_server', 'amcl'])
        ],
    )

    navigation_lifecycle_manager = TimerAction(
        period=7.0,
        actions=[
            lifecycle_manager('navigation', ['planner_server', 'controller_server', 'bt_navigator', 'behavior_server', 'waypoint_follower'])
        ],
    )

    goal_pose_bridge = goal_bridge()

    auto_route = Node(
        package="custom_corridor",
        executable="hospital_auto_route.py",
        name="hospital_auto_route",
        output="screen",
        parameters=[{"use_sim_time": True}],
    )

    nodes = [
        *core_nodes,
        amcl_lifecycle_manager,
        navigation_lifecycle_manager,
        goal_pose_bridge,
    ]
    if LaunchConfiguration("auto_route").perform(context).lower().strip() in ("true", "1", "yes"):
        nodes.append(auto_route)
    return nodes

def generate_launch_description():
    rmw_implementation = SetEnvironmentVariable(
        name="RMW_IMPLEMENTATION",
        value="rmw_fastrtps_cpp",
    )

    fastdds_transport = SetEnvironmentVariable(
        name="FASTDDS_BUILTIN_TRANSPORTS",
        value="UDPv4",
    )

    ros_domain = SetEnvironmentVariable(
        name="ROS_DOMAIN_ID",
        value=os.environ.get("ROS_DOMAIN_ID", "0"),
    )

    discovery_range = SetEnvironmentVariable(
        name="ROS_AUTOMATIC_DISCOVERY_RANGE",
        value=os.environ.get("ROS_AUTOMATIC_DISCOVERY_RANGE", "LOCALHOST"),
    )

    controller_arg = DeclareLaunchArgument(
        "controller",
        default_value="dwb",
        description="Nav2 local controller plugin: dwb or mppi",
    )

    auto_route_arg = DeclareLaunchArgument(
        "auto_route",
        default_value="true",
        description="Start the room 5 north/south route automatically",
    )

    params_file_arg = DeclareLaunchArgument(
        "params_file",
        default_value="",
        description="Full path to custom Nav2 params yaml file (overrides controller choice)",
    )

    map_arg = DeclareLaunchArgument(
        "map",
        default_value="hospital/hospital_easy",
        description="Name or path of map yaml file (e.g. arena_obstacle or corridor_090)",
    )

    initial_x_arg = DeclareLaunchArgument(
        "initial_x", default_value="-18.0",
        description="Initial AMCL x pose in map frame",
    )
    initial_y_arg = DeclareLaunchArgument(
        "initial_y", default_value="0.0",
        description="Initial AMCL y pose in map frame",
    )
    initial_yaw_arg = DeclareLaunchArgument(
        "initial_yaw", default_value="0.0",
        description="Initial AMCL yaw pose in map frame",
    )

    robot_model_arg = DeclareLaunchArgument(
        "robot_model",
        default_value="mecanum",
        description="Robot model: mecanum or burger",
        choices=["mecanum", "burger", "tb3", "turtlebot3"],
    )

    return LaunchDescription(
        [
            rmw_implementation,
            fastdds_transport,
            ros_domain,
            discovery_range,
            controller_arg,
            robot_model_arg,
            auto_route_arg,
            params_file_arg,
            map_arg,
            initial_x_arg,
            initial_y_arg,
            initial_yaw_arg,
            OpaqueFunction(function=launch_setup),
        ]
    )
