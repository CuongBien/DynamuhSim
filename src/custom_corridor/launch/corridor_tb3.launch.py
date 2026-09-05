import os
import xacro

from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    OpaqueFunction,
    SetEnvironmentVariable,
)
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def launch_setup(context):

    # ---------------------------------------------------------
    # Corridor width
    # ---------------------------------------------------------
    width = LaunchConfiguration('width').perform(context)

    try:
        width = f"{float(width):.2f}"
    except ValueError:
        raise RuntimeError(
            f"Invalid width '{width}'. "
            "Allowed values: 0.70, 0.90, 1.20"
        )

    world_map = {
        '0.70': 'corridor_070.sdf',
        '0.90': 'corridor_090.sdf',
        '1.20': 'corridor_120.sdf',
    }

    if width not in world_map:
        raise RuntimeError(
            f"Invalid width '{width}'. "
            "Allowed values: 0.70, 0.90, 1.20"
        )

    # ---------------------------------------------------------
    # Package paths
    # ---------------------------------------------------------
    corridor_share = get_package_share_directory(
        'custom_corridor'
    )

    turtlebot_share = get_package_share_directory(
        'turtlebot3_gazebo'
    )
    description_share = get_package_share_directory(
        'turtlebot3_description'
    )

    urdf_file = os.path.join(
        description_share,
        'urdf',
        'turtlebot3_burger.urdf'
    )

    doc = xacro.process_file(urdf_file)
    robot_description = doc.toxml()
    # ---------------------------------------------------------
    # World
    # ---------------------------------------------------------
    world_file = os.path.join(
        corridor_share,
        'worlds',
        world_map[width]
    )

    # ---------------------------------------------------------
    # LOCAL TurtleBot3 Burger model
    #
    # This is the copy where we changed:
    #
    #     gpu_lidar -> lidar
    #
    # ---------------------------------------------------------
    burger_model = os.path.join(
        corridor_share,
        'models',
        'turtlebot3_burger',
        'model.sdf'
    )

    # ---------------------------------------------------------
    # Bridge configuration
    # ---------------------------------------------------------
    bridge_config = os.path.join(
        turtlebot_share,
        'params',
        'turtlebot3_burger_bridge.yaml'
    )

    # ---------------------------------------------------------
    # Gazebo
    # ---------------------------------------------------------
    gazebo = ExecuteProcess(
        cmd=[
            'gz',
            'sim',
            '-r',
            world_file,
        ],
        output='screen',
    )

    # ---------------------------------------------------------
    # Spawn TurtleBot3 Burger
    #
    # Corridor:
    #   x = -15 ... +15
    #
    # Robot:
    #   x = -13
    #   y = 0
    #   z = 0.01
    #   yaw = 0 -> +X
    # ---------------------------------------------------------
    spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-name', 'burger',
            '-file', burger_model,

            '-x', '-13.0',
            '-y', '0.0',
            '-z', '0.01',

            '-Y', '0.0',
        ],
        output='screen',
    )

    # ---------------------------------------------------------
    # ROS <-> Gazebo bridge
    # ---------------------------------------------------------
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '--ros-args',
            '-p',
            f'config_file:={bridge_config}',
        ],
        output='screen',
    )

    # ---------------------------------------------------------
    # RViz
    # ---------------------------------------------------------
    rviz_config = os.path.join(
    	corridor_share,
    	'rviz',
    	'corridor.rviz',
    )

    rviz = Node(
    	package='rviz2',
    	executable='rviz2',
    	name='rviz2',
    	arguments=[
          '-d',
          rviz_config,
    	],
    output='screen',
    )
    
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        parameters=[
            {
                'robot_description': robot_description,
                'use_sim_time': True,
            }
        ],
        output='screen',
    )
    return [
        gazebo,
        spawn_robot,
        bridge,
	robot_state_publisher,
        rviz,
    ]


def generate_launch_description():

    # ---------------------------------------------------------
    # Package paths
    #
    # This function has its own scope, so we must define it here.
    # ---------------------------------------------------------
    corridor_share = get_package_share_directory(
        'custom_corridor'
    )

    turtlebot_share = get_package_share_directory(
        'turtlebot3_gazebo'
    )

    # ---------------------------------------------------------
    # Corridor width argument
    # ---------------------------------------------------------
    width_arg = DeclareLaunchArgument(
        'width',
        default_value='0.90',
        description=(
            'Corridor width. '
            'Allowed values: 0.70, 0.90, 1.20'
        ),
    )

    # ---------------------------------------------------------
    # Gazebo resource path
    #
    # Local models:
    #   ~/nav_ws/src/custom_corridor/models
    #
    # TurtleBot3 common models:
    #   /opt/ros/jazzy/share/turtlebot3_gazebo/models
    #
    # This allows:
    #
    # model://turtlebot3_common/...
    #
    # to be resolved.
    # ---------------------------------------------------------
    local_models = os.path.join(
        corridor_share,
        'models',
    )

    gazebo_models = os.path.join(
        turtlebot_share,
        'models',
    )

    resource_path = SetEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH',
        value=(
            local_models
            + os.pathsep
            + gazebo_models
            + os.pathsep
            + os.environ.get(
                'GZ_SIM_RESOURCE_PATH',
                '',
            )
        ),
    )

    plugin_share = get_package_share_directory(
        'custom_corridor_plugins'
    )

    system_plugin_path = SetEnvironmentVariable(
        name='GZ_SIM_SYSTEM_PLUGIN_PATH',
        value=(
            os.path.join(os.path.dirname(os.path.dirname(plugin_share)), 'lib')
            + os.pathsep
            + os.environ.get('GZ_SIM_SYSTEM_PLUGIN_PATH', '')
        ),
    )
    
    rmw_implementation = SetEnvironmentVariable(
     name='RMW_IMPLEMENTATION',
     value='rmw_fastrtps_cpp'
    )

    fastdds_transport = SetEnvironmentVariable(
     name='FASTDDS_BUILTIN_TRANSPORTS',
     value='UDPv4'
    )

    ros_domain = SetEnvironmentVariable(
     name='ROS_DOMAIN_ID',
     value='0'
    )
    # ---------------------------------------------------------
    # Launch description
    # ---------------------------------------------------------
    return LaunchDescription([
     rmw_implementation,
     fastdds_transport,
     ros_domain,

     width_arg,
     resource_path,
    system_plugin_path,
     OpaqueFunction(function=launch_setup),
    ])
