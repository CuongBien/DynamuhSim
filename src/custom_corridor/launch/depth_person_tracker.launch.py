"""Explicit experimental launch; never included by the default robot launch."""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('depth_topic', default_value='/camera/depth/image_rect_raw'),
        DeclareLaunchArgument('camera_info_topic', default_value='/camera/camera_info'),
        DeclareLaunchArgument('odom_topic', default_value='/odom'),
        Node(package='custom_corridor', executable='depth_person_tracker_node.py',
             parameters=[dict(use_sim_time=ParameterValue(LaunchConfiguration('use_sim_time'), value_type=bool),
                              depth_topic=LaunchConfiguration('depth_topic'),
                              camera_info_topic=LaunchConfiguration('camera_info_topic'),
                              odom_topic=LaunchConfiguration('odom_topic'))], output='screen')])
