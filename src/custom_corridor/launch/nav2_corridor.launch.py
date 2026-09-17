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
        if os.path.isabs(map_name) and os.path.exists(map_name):
            map_override["yaml_filename"] = map_name
        else:
            cand1 = os.path.join(package_share, "maps", f"{map_name}.yaml")
            cand2 = os.path.join(package_share, "maps", map_name)
            if os.path.exists(cand1):
                map_override["yaml_filename"] = cand1
            elif os.path.exists(cand2):
                map_override["yaml_filename"] = cand2

    if "yaml_filename" not in map_override:
        map_override["yaml_filename"] = os.path.join(package_share, "maps", "corridor_090.yaml")

    map_server_params = [nav2_params, map_override]

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
        parameters=[nav2_params],
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
        period=2.0,
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
        period=5.0,
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

    return [
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

    params_file_arg = DeclareLaunchArgument(
        "params_file",
        default_value="",
        description="Full path to custom Nav2 params yaml file (overrides controller choice)",
    )

    map_arg = DeclareLaunchArgument(
        "map",
        default_value="corridor_090",
        description="Name or path of map yaml file (e.g. arena_obstacle or corridor_090)",
    )

    return LaunchDescription(
        [
            rmw_implementation,
            fastdds_transport,
            ros_domain,
            discovery_range,
            controller_arg,
            params_file_arg,
            map_arg,
            OpaqueFunction(function=launch_setup),
        ]
    )