"""Opt-in warehouse adapter launch; existing launch and bridge files remain untouched."""
import os
from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    warehouse=Path(__file__).resolve().parent.parent/'warehouse'
    launches=Path(get_package_share_directory('custom_corridor'))/'launch'
    sim=IncludeLaunchDescription(PythonLaunchDescriptionSource(str(launches/'corridor_tb3.launch.py')),
        launch_arguments={'width':'arena','custom_world':os.environ['SOCNAVGYM_EXPERIMENT_WORLD'],
            'robot_model':'mecanum','obstacle':'human','x':'-27','y':'0','yaw':'0',
            'gui':LaunchConfiguration('gui'),'rviz':LaunchConfiguration('rviz')}.items())
    nav=IncludeLaunchDescription(PythonLaunchDescriptionSource(str(launches/'nav2_corridor.launch.py')),
        condition=IfCondition(LaunchConfiguration('nav')),
        launch_arguments={'map':str(warehouse/'warehouse.yaml'),'params_file':str(warehouse/'nav2_warehouse.yaml'),
            'robot_model':'mecanum','controller':'mppi','episode_initial_pose':'true'}.items())
    bridge=Node(package='ros_gz_bridge',executable='parameter_bridge',name='socnavgym_ground_truth_bridge',
        arguments=['/socnavgym/ground_truth@std_msgs/msg/String[gz.msgs.StringMsg'],output='screen')
    return LaunchDescription([DeclareLaunchArgument('gui',default_value='false'),
        DeclareLaunchArgument('rviz',default_value='false'),DeclareLaunchArgument('nav',default_value='false'),
        sim,bridge,TimerAction(period=6.,actions=[nav])])
