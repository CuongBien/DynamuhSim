import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument, ExecuteProcess, OpaqueFunction, RegisterEventHandler,
    SetEnvironmentVariable, TimerAction,
)
from launch.event_handlers import OnProcessExit
from launch.substitutions import EnvironmentVariable, LaunchConfiguration
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
            cand_school = os.path.join(package_share, "maps", "school_arena.yaml")
            if os.path.exists(cand1):
                map_override["yaml_filename"] = cand1
            elif os.path.exists(cand2):
                map_override["yaml_filename"] = cand2
            elif map_name in ["school", "school_arena"] and os.path.exists(cand_school):
                map_override["yaml_filename"] = cand_school

    if "yaml_filename" not in map_override:
        map_override["yaml_filename"] = os.path.join(package_share, "maps", "corridor_090.yaml")

    map_server_params = [nav2_params, map_override]
    episode_mode = LaunchConfiguration("episode_initial_pose").perform(context).lower() == "true"

    map_server = Node(
        package="nav2_map_server",
        executable="map_server",
        name="map_server",
        output="screen",
        parameters=map_server_params,
    )

    amcl_params = [nav2_params]
    if episode_mode:
        amcl_params.append({"set_initial_pose": False, "transform_tolerance": 0.5})
    else:
        selected_map = map_override.get("yaml_filename", "")
        if "school" in selected_map or map_name in ["school", "school_arena"]:
            amcl_params.append({
                "set_initial_pose": True,
                "initial_pose": {"x": -13.5, "y": -8.15, "z": 0.0, "yaw": 0.0},
            })
        elif "arena" in selected_map or map_name in ["arena", "arena_obstacle", "dataset", "rvo2"]:
            amcl_params.append({
                "set_initial_pose": True,
                "initial_pose": {"x": -13.0, "y": 0.0, "z": 0.0, "yaw": 0.0},
            })
        else:
            amcl_params.append({
                "set_initial_pose": True,
                "initial_pose": {"x": 0.0, "y": 0.0, "z": 0.0, "yaw": 0.0},
            })

    amcl = Node(
        package="nav2_amcl",
        executable="amcl",
        name="amcl",
        output="screen",
        parameters=amcl_params,
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

    navigation_lifecycle_manager = Node(
                package="nav2_lifecycle_manager",
                executable="lifecycle_manager",
                name="lifecycle_manager_navigation",
                output="screen",
                parameters=[
                    {
                        # MPPI configuration can be CPU-heavy in headless trials.
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

    if episode_mode:
        pose_gate = ExecuteProcess(
            cmd=["python3", os.path.join(os.path.dirname(__file__),
                                         "publish_episode_initial_pose.py"),
                 "--params-file", nav2_params, "--ros-args",
                 "-p", "use_sim_time:=true"],
            output="screen",
        )
        navigation_actions = [
            RegisterEventHandler(OnProcessExit(
                target_action=pose_gate,
                on_exit=lambda event, context: [navigation_lifecycle_manager]
                if event.returncode == 0 else [],
            )),
            pose_gate,
        ]
    else:
        navigation_actions = [TimerAction(
            period=5.0, actions=[navigation_lifecycle_manager]
        )]

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
        *navigation_actions,
        goal_pose_bridge,
    ]


def generate_launch_description():
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

    episode_pose_arg = DeclareLaunchArgument(
        "episode_initial_pose", default_value="false",
        description="Publish episode AMCL pose and gate navigation on scan-time TF",
    )

    map_arg = DeclareLaunchArgument(
        "map",
        default_value="corridor_090",
        description="Name or path of map yaml file (e.g. arena_obstacle or corridor_090)",
    )

    return LaunchDescription(
        [
            ros_domain,
            discovery_range,
            controller_arg,
            params_file_arg,
            map_arg,
            episode_pose_arg,
            OpaqueFunction(function=launch_setup),
        ]
    )
