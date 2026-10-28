"""Existing robot/map/MPPI; only own generated world/runtime initial pose YAML."""
import os
from pathlib import Path
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument,IncludeLaunchDescription,TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    social=Path(__file__).resolve().parents[2];base=Path(get_package_share_directory('custom_corridor'))/'launch'
    sim=IncludeLaunchDescription(PythonLaunchDescriptionSource(os.environ['DATASET_CAMERA_LAUNCH']),launch_arguments=dict(width='arena',custom_world=os.environ['NAV2_PILOT_WORLD'],robot_model='mecanum',custom_robot_model=os.environ['DATASET_ROBOT_SDF'],obstacle='human',x='-28',y='0',yaw='0',gui=LaunchConfiguration('gui'),rviz='false').items())
    nav=IncludeLaunchDescription(PythonLaunchDescriptionSource(str(base/'nav2_corridor.launch.py')),launch_arguments=dict(map=str(social/'warehouse/warehouse.yaml'),params_file=os.environ['NAV2_PILOT_PARAMS'],robot_model='mecanum',controller='mppi',episode_initial_pose='true').items())
    bridge=Node(package='ros_gz_bridge',executable='parameter_bridge',name='nav2_pilot_ground_truth_bridge',arguments=['/common/ground_truth@std_msgs/msg/String[gz.msgs.StringMsg'],output='screen')
    return LaunchDescription([DeclareLaunchArgument('gui',default_value='false'),sim,bridge,TimerAction(period=6.,actions=[nav])])
