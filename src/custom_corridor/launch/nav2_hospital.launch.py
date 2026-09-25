import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, SetEnvironmentVariable, TimerAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def launch_setup(context, *args, **kwargs):
    package_share = get_package_share_directory("custom_corridor")

    controller = LaunchConfiguration("controller").perform(context).lower().strip()
    custom_params = LaunchConfiguration("params_file").perform(context).strip()

    if custom_params and os.path.exists(custom_params):
        nav2_params = custom_params
    elif controller == "mppi":
        nav2_params = os.path.join(package_share, "config", "nav2_mppi.yaml")
    else:
        nav2_params = os.path.join(package_share, "config", "nav2_corridor.yaml")

    map_name = LaunchConfiguration("map").perform(context).strip()
    map_override = {}

    if map_name:
        if os.path.isabs(map_name):
            if not os.path.isfile(map_name):
                raise RuntimeError(f"Map YAML does not exist: {map_name}")
            resolved_map = map_name
        else:
            cand1 = os.path.join(package_share, "maps", f"{map_name}.yaml")
            cand2 = os.path.join(package_share, "maps", map_name)
            if os.path.isfile(cand1):
                resolved_map = cand1
            elif os.path.isfile(cand2):
                resolved_map = cand2
            else:
                raise RuntimeError(
                    f"Map '{map_name}' was not found. Checked:\n"
                    f"  {cand1}\n"
                    f"  {cand2}"
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

    map_server = Node(
        package="nav2_map_server",
        executable="map_server",
        name="map_server",
        output="screen",
        parameters=map_server_params,
    )

    amcl = Node(
        package="nav2_amcl",
        executable="amcl",
        name="amcl",
        output="screen",
        parameters=[nav2_params, amcl_override],
    )

    planner_server = Node(
        package="nav2_planner",
        executable="planner_server",
        name="planner_server",
        output="screen",
        parameters=[nav2_params],
    )

    controller_server = Node(
        package="nav2_controller",
        executable="controller_server",
        name="controller_server",
        output="screen",
        parameters=[nav2_params],
    )

    bt_override = {}
    try:
        bt_share = get_package_share_directory("nav2_bt_navigator")
        default_bt_xml = os.path.join(
            bt_share, "behavior_trees", "navigate_to_pose_w_replanning_and_recovery.xml"
        )
        if os.path.exists(default_bt_xml):
            bt_override["default_nav_to_pose_bt_xml"] = default_bt_xml
    except Exception:
        pass

    bt_navigator_params = [nav2_params, bt_override] if bt_override else [nav2_params]

    bt_navigator = Node(
        package="nav2_bt_navigator",
        executable="bt_navigator",
        name="bt_navigator",
        output="screen",
        parameters=bt_navigator_params,
    )

    behavior_server = Node(
        package="nav2_behaviors",
        executable="behavior_server",
        name="behavior_server",
        output="screen",
        parameters=[nav2_params],
    )

    waypoint_follower = Node(
        package="nav2_waypoint_follower",
        executable="waypoint_follower",
        name="waypoint_follower",
        output="screen",
        parameters=[nav2_params],
    )

    amcl_lifecycle_manager = TimerAction(
        period=4.0,
        actions=[
            Node(
                package="nav2_lifecycle_manager",
                executable="lifecycle_manager",
                name="lifecycle_manager_localization",
                output="screen",
                parameters=[
                    {
                        "use_sim_time": True,
                        "autostart": True,
                        "bond_timeout": 30.0,
                        "attempt_respawn_reconnection": True,
                        "node_names": [
                            "map_server",
                            "amcl",
                        ],
                    }
                ],
            )
        ],
    )

    navigation_lifecycle_manager = TimerAction(
        period=7.0,
        actions=[
            Node(
                package="nav2_lifecycle_manager",
                executable="lifecycle_manager",
                name="lifecycle_manager_navigation",
                output="screen",
                parameters=[
                    {
                        "use_sim_time": True,
                        "autostart": True,
                        "bond_timeout": 30.0,
                        "attempt_respawn_reconnection": True,
                        "node_names": [
                            "planner_server",
                            "controller_server",
                            "bt_navigator",
                            "behavior_server",
                            "waypoint_follower",
                        ],
                    }
                ],
            )
        ],
    )

    goal_pose_bridge = Node(
        package="custom_corridor",
        executable="goal_pose_bridge.py",
        name="goal_pose_bridge",
        output="screen",
        parameters=[{"use_sim_time": True}],
    )

    auto_route = Node(
        package="custom_corridor",
        executable="hospital_auto_route.py",
        name="hospital_auto_route",
        output="screen",
        parameters=[{"use_sim_time": True}],
    )

    nodes = [
        map_server,
        amcl,
        planner_server,
        controller_server,
        bt_navigator,
        behavior_server,
        waypoint_follower,
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

    return LaunchDescription(
        [
            rmw_implementation,
            fastdds_transport,
            ros_domain,
            discovery_range,
            controller_arg,
            auto_route_arg,
            params_file_arg,
            map_arg,
            initial_x_arg,
            initial_y_arg,
            initial_yaw_arg,
            OpaqueFunction(function=launch_setup),
        ]
    )
