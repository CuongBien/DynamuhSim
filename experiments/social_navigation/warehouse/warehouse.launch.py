"""Warehouse wrapper: reuse existing ROS launch files without modifying them."""
from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    here=Path(__file__).resolve().parent
    launch_dir=Path(get_package_share_directory('custom_corridor'))/'launch'
    sim=IncludeLaunchDescription(PythonLaunchDescriptionSource(str(launch_dir/'corridor_tb3.launch.py')),
        launch_arguments={'width':'arena','custom_world':str(here/'warehouse.sdf'),
            'robot_model':'mecanum','obstacle':'human','x':'-27','y':'0','yaw':'0',
            'gui':LaunchConfiguration('gui'),'rviz':LaunchConfiguration('rviz')}.items())
    nav=IncludeLaunchDescription(PythonLaunchDescriptionSource(str(launch_dir/'nav2_corridor.launch.py')),
        condition=IfCondition(LaunchConfiguration('nav')),
        launch_arguments={'map':str(here/'warehouse.yaml'),'params_file':str(here/'nav2_warehouse.yaml'),
            'robot_model':'mecanum','controller':'mppi','episode_initial_pose':'true'}.items())
    return LaunchDescription([DeclareLaunchArgument('gui',default_value='true'),
        DeclareLaunchArgument('rviz',default_value='true'),DeclareLaunchArgument('nav',default_value='true'),
        sim,TimerAction(period=6.,actions=[nav])])

