"""Same warehouse, robot, map and MPPI parameters for every benchmark backend."""
import os,json
from pathlib import Path
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node

def generate_launch_description():
    root=Path(__file__).resolve().parent.parent
    base=Path(get_package_share_directory('custom_corridor'))/'launch'
    start=json.loads(os.environ.get('SOCIAL_BENCHMARK_START','[-27,0,0]'))
    sim=IncludeLaunchDescription(PythonLaunchDescriptionSource(str(base/'corridor_tb3.launch.py')),
        launch_arguments=dict(width='arena',custom_world=os.environ['SOCIAL_BENCHMARK_WORLD'],
            robot_model='mecanum',obstacle='human',x=str(start[0]),y=str(start[1]),yaw=str(start[2]),
            gui=LaunchConfiguration('gui'),rviz='false').items())
    nav=IncludeLaunchDescription(PythonLaunchDescriptionSource(str(base/'nav2_corridor.launch.py')),
        launch_arguments=dict(map=str(root/'warehouse/warehouse.yaml'),params_file=os.environ.get('SOCIAL_BENCHMARK_NAV_PARAMS',str(root/'warehouse/nav2_warehouse.yaml')),
            robot_model='mecanum',controller='mppi',episode_initial_pose='true').items())
    bridge=Node(package='ros_gz_bridge',executable='parameter_bridge',name='social_benchmark_bridge',
        arguments=['/socnavgym/ground_truth@std_msgs/msg/String[gz.msgs.StringMsg','/crowdsimplus/ground_truth@std_msgs/msg/String[gz.msgs.StringMsg'],output='screen')
    return LaunchDescription([DeclareLaunchArgument('gui',default_value='false'),sim,bridge,
        TimerAction(period=6.,actions=[nav])])
