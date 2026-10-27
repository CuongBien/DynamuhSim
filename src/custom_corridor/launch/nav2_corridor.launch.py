import os
import sys

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument, ExecuteProcess, OpaqueFunction, RegisterEventHandler,
    SetEnvironmentVariable, TimerAction,
)
from launch.event_handlers import OnProcessExit
from launch.substitutions import EnvironmentVariable, LaunchConfiguration

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
                "initial_pose": {"x": -13.0, "y": 0.0, "z": 0.0, "yaw": 0.0},
            })

    cmd_vel_topic = LaunchConfiguration("cmd_vel_topic").perform(context).strip() or "cmd_vel"

    nodes = navigation_nodes(nav2_params, map_server_params, amcl_params, cmd_vel_topic)

    amcl_lifecycle_manager = TimerAction(
        period=2.0,
        actions=[
            lifecycle_manager('localization', ['map_server', 'amcl'])
        ],
    )

    navigation_lifecycle_manager = lifecycle_manager('navigation', ['planner_server', 'controller_server', 'bt_navigator', 'behavior_server', 'waypoint_follower'])

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

    goal_pose_bridge = goal_bridge()

    return [
        *nodes,
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

    robot_model_arg = DeclareLaunchArgument(
        "robot_model",
        default_value="mecanum",
        description="Robot model: mecanum (default) or burger",
    )

    cmd_vel_topic_arg = DeclareLaunchArgument(
        "cmd_vel_topic",
        default_value="cmd_vel",
        description="Target topic for controller_server and behavior_server cmd_vel",
    )

    return LaunchDescription(
        [
            ros_domain,
            discovery_range,
            robot_model_arg,
            controller_arg,
            params_file_arg,
            cmd_vel_topic_arg,
            map_arg,
            episode_pose_arg,
            OpaqueFunction(function=launch_setup),
        ]
    )
