import os
from pathlib import Path
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument,IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    base=Path(get_package_share_directory('custom_corridor'))/'launch/corridor_tb3.launch.py'
    sim=IncludeLaunchDescription(PythonLaunchDescriptionSource(str(base)),launch_arguments=dict(width='arena',custom_world=os.environ['COMPLEXITYNAV_EXPERIMENT_WORLD'],robot_model='mecanum',obstacle='human',x='-27',y='0',yaw='0',gui=LaunchConfiguration('gui'),rviz='false').items())
    bridge=Node(package='ros_gz_bridge',executable='parameter_bridge',name='complexitynav_ground_truth_bridge',arguments=['/complexitynav/ground_truth@std_msgs/msg/String[gz.msgs.StringMsg'],output='screen')
    return LaunchDescription([DeclareLaunchArgument('gui',default_value='false'),sim,bridge])
