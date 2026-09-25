#!/usr/bin/env python3
"""Launch Gazebo, TurtleBot3, bridges, robot state publisher and Nav2."""
import os

import xacro
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription,
    SetEnvironmentVariable, TimerAction,
)
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    here = os.path.dirname(os.path.abspath(__file__))
    school_dir = os.path.dirname(here)
    package_share = get_package_share_directory("custom_corridor")
    tb3_gazebo = get_package_share_directory("turtlebot3_gazebo")
    tb3_description = get_package_share_directory("turtlebot3_description")

    world = os.path.join(here, "school_floor.world")
    robot = os.path.join(here, "demo_robot.sdf")
    bridge = os.path.join(here, "demo_bridge.yaml")
    nav_params = os.path.join(here, "nav2_school.yaml")
    map_yaml = os.path.join(here, "map.yaml")
    pose_applier = os.path.join(here, "collision_safe_pose_applier.py")
    sync_monitor = os.path.join(here, "pose_sync_monitor.py")
    urdf = xacro.process_file(
        os.path.join(tb3_description, "urdf", "turtlebot3_burger.urdf")
    ).toxml()

    resource_path = (
        os.path.join(package_share, "models") + os.pathsep
        + os.path.join(tb3_gazebo, "models") + os.pathsep
        + os.environ.get("GZ_SIM_RESOURCE_PATH", "")
    )

    gazebo_gui = ExecuteProcess(
        cmd=["gz", "sim", "-r", world],
        condition=IfCondition(LaunchConfiguration("gui")),
        output="screen",
    )
    gazebo_headless = ExecuteProcess(
        cmd=["gz", "sim", "-r", "-s", world],
        condition=UnlessCondition(LaunchConfiguration("gui")),
        output="screen",
    )
    bridge_node = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        parameters=[{"config_file": bridge}],
        output="screen",
    )
    rsp = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[{"robot_description": urdf, "use_sim_time": True}],
        output="screen",
    )
    spawn = TimerAction(
        period=3.0,
        actions=[Node(
            package="ros_gz_sim",
            executable="create",
            arguments=[
                "-world", "school_arena", "-name", "robot", "-file", robot,
                "-x", "-13.5", "-y", "-8.15", "-z", "0.01", "-Y", "0.0",
            ],
            output="screen",
        )],
    )
    # Resolve the physical model by name through the Gazebo GUI service.
    camera_follow = TimerAction(
        period=5.0,
        actions=[ExecuteProcess(
            cmd=[
                "gz", "service", "-s", "/gui/follow",
                "--reqtype", "gz.msgs.StringMsg",
                "--reptype", "gz.msgs.Boolean",
                "--timeout", "3000", "--req", 'data: "robot"',
            ],
            condition=IfCondition(LaunchConfiguration("gui")),
            output="screen",
        )],
    )
    camera_follow_view = TimerAction(
        period=5.5,
        actions=[ExecuteProcess(
            cmd=[
                "gz", "topic", "-t", "/gui/track",
                "-m", "gz.msgs.CameraTrack", "-p",
                (
                    'track_mode: USE_LAST '
                    'follow_offset {x: -2.5 y: 0.0 z: 2.2} '
                    'follow_pgain: 0.08 track_pgain: 0.08'
                ),
            ],
            condition=IfCondition(LaunchConfiguration("gui")),
            output="screen",
        )],
    )

    nav2 = TimerAction(
        period=6.0,
        actions=[IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(package_share, "launch", "nav2_corridor.launch.py")
            ),
            launch_arguments={
                "controller": "dwb",
                "params_file": nav_params,
                "map": map_yaml,
            }.items(),
        )],
    )
    applier = ExecuteProcess(
        cmd=[
            "python3", pose_applier, "--ros-args",
            "-p", "world_name:=school_arena",
            "-p", "target_topic:=/school_hunav/target_poses",
            "-p", f"map_yaml:={map_yaml}", "-p", "agent_radius:=0.22",
            "-p", "apply_hz:=20.0", "-p", "ground_z:=0.0",
        ],
        output="screen",
    )
    sync = ExecuteProcess(
        cmd=[
            "python3", sync_monitor, "--ros-args",
            "-p", "use_sim_time:=true",
            "-p", "robot_name:=robot",
            "-p", "world_frame:=map",
        ],
        output="screen",
    )
    rviz = Node(
        package="rviz2",
        executable="rviz2",
        arguments=["-d", os.path.join(here, "demo_1.rviz")],
        condition=IfCondition(LaunchConfiguration("rviz")),
        output="screen",
    )

    return LaunchDescription([
        DeclareLaunchArgument("rviz", default_value="true"),
        DeclareLaunchArgument("gui", default_value="true"),
        SetEnvironmentVariable("ROS_DOMAIN_ID", "0"),
        SetEnvironmentVariable("RMW_IMPLEMENTATION", "rmw_fastrtps_cpp"),
        SetEnvironmentVariable("FASTDDS_BUILTIN_TRANSPORTS", "UDPv4"),
        SetEnvironmentVariable("GZ_PARTITION", "school_hunav"),
        SetEnvironmentVariable("GZ_SIM_RESOURCE_PATH", resource_path),
        gazebo_gui, gazebo_headless, bridge_node, rsp, spawn, camera_follow, camera_follow_view,
        nav2, applier, sync, rviz,
    ])

