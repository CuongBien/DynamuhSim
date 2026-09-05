import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import SetEnvironmentVariable, TimerAction
from launch_ros.actions import Node


def generate_launch_description():
    package_share = get_package_share_directory("custom_corridor")

    nav2_params = os.path.join(
        package_share,
        "config",
        "nav2_corridor.yaml",
    )

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
        value="0",
    )

    map_server = Node(
        package="nav2_map_server",
        executable="map_server",
        name="map_server",
        output="screen",
        parameters=[nav2_params],
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

    bt_navigator = Node(
        package="nav2_bt_navigator",
        executable="bt_navigator",
        name="bt_navigator",
        output="screen",
        parameters=[nav2_params],
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

    amcl_lifecycle_manager = Node(
        package="nav2_lifecycle_manager",
        executable="lifecycle_manager",
        name="lifecycle_manager_localization",
        output="screen",
        parameters=[
            {
                "use_sim_time": True,
                "autostart": True,
                "bond_timeout": 30.0,
                "node_names": [
                    "map_server",
                    "amcl",
                ],
            }
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

    return LaunchDescription(
        [
            rmw_implementation,
            fastdds_transport,
            ros_domain,

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
    )