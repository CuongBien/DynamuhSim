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
from launch.conditions import IfCondition
from launch.substitutions import EnvironmentVariable, LaunchConfiguration


def launch_setup(context):

    # ---------------------------------------------------------
    # Corridor width
    # ---------------------------------------------------------
    width = LaunchConfiguration('width').perform(context)
    obstacle = LaunchConfiguration('obstacle').perform(context).lower().strip()

    world_map = {
        '0.70': 'corridor_070.sdf',
        '0.90': 'corridor_090.sdf',
        '1.20': 'corridor_120.sdf',
        'arena': 'arena_obstacle.sdf',
    }

    if width.lower() in ['arena', 'open', 'big', 'wide']:
        width = 'arena'
    else:
        try:
            width = f"{float(width):.2f}"
        except ValueError:
            raise RuntimeError(
                f"Invalid width '{width}'. "
                "Allowed values: 0.70, 0.90, 1.20, arena"
            )

    if width not in world_map:
        raise RuntimeError(
            f"Invalid width '{width}'. "
            "Allowed values: 0.70, 0.90, 1.20, arena"
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
    raw_world_file = os.path.join(
        corridor_share,
        'worlds',
        world_map[width]
    )

    world_file = raw_world_file
    if obstacle in ['object', 'cylinder', 'box', 'none', 'clean', 'empty']:
        with open(raw_world_file, 'r') as f:
            content = f.read()
        import re
        filtered_content = re.sub(r'<actor name="human_actor">.*?</actor>', '', content, flags=re.DOTALL)
        if obstacle in ['object', 'cylinder', 'box']:
            filtered_content = filtered_content.replace(
                '<pose>5.0 0.0 -50.0 0 0 3.14159</pose>',
                '<pose>5.0 0.0 0.85 0 0 3.14159</pose>'
            )
            filtered_content = filtered_content.replace(
                '<default_mode>human</default_mode>',
                '<default_mode>object</default_mode>'
            )
        elif obstacle in ['none', 'clean', 'empty']:
            filtered_content = filtered_content.replace(
                '<pose>5.0 0.0 0.85 0 0 3.14159</pose>',
                '<pose>5.0 0.0 -50.0 0 0 3.14159</pose>'
            )
            filtered_content = filtered_content.replace(
                '<default_mode>human</default_mode>',
                '<default_mode>none</default_mode>'
            )
        active_world = f"/tmp/corridor_{width}_{obstacle}.sdf"
        with open(active_world, 'w') as f:
            f.write(filtered_content)
        world_file = active_world

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
        corridor_share,
        'config',
        'turtlebot3_burger_bridge.yaml'
    )

    # ---------------------------------------------------------
    # Gazebo
    # ---------------------------------------------------------
    gui = LaunchConfiguration('gui').perform(context).lower().strip()
    gz_cmd = ['gz', 'sim', '-r']
    if gui in ['false', '0', 'no', 'headless']:
        gz_cmd.append('-s')
    gz_cmd.append(world_file)

    gazebo = ExecuteProcess(
        cmd=gz_cmd,
        output='screen',
        additional_env={
            'GZ_OBSTACLE_TYPE': obstacle,
        },
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
        parameters=[{
            'config_file': bridge_config,
        }],
        output='screen',
    )

    # ros_gz_image provides efficient RGB and float depth image conversion.
    # Names follow the RealSense ROS convention and both images are aligned.
    image_bridge = Node(
        package='ros_gz_image',
        executable='image_bridge',
        name='camera_image_bridge',
        arguments=['/camera/image', '/camera/depth_image'],
        remappings=[
            ('/camera/image', '/camera/color/image_raw'),
            ('/camera/depth_image', '/camera/depth/image_rect_raw'),
        ],
        parameters=[{'use_sim_time': True}],
        output='screen',
    )

    # The installed Burger URDF has no camera frames, so publish the fixed
    # transforms here. The optical frame follows the ROS camera convention:
    # +Z forward, +X right, +Y down.
    camera_link_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='camera_link_tf',
        arguments=[
            '--x', '0.08', '--y', '0.0', '--z', '0.20',
            '--roll', '0.0', '--pitch', '0.0', '--yaw', '0.0',
            '--frame-id', 'base_link',
            '--child-frame-id', 'camera_link',
        ],
        output='screen',
    )

    camera_optical_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='camera_optical_tf',
        arguments=[
            '--x', '0.0', '--y', '0.0', '--z', '0.0',
            '--roll', '-1.57079632679', '--pitch', '0.0',
            '--yaw', '-1.57079632679',
            '--frame-id', 'camera_link',
            '--child-frame-id', 'camera_optical_frame',
        ],
        output='screen',
    )

    camera_imu_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='camera_imu_tf',
        arguments=[
            '--x', '0.0', '--y', '0.0', '--z', '0.0',
            '--roll', '0.0', '--pitch', '0.0', '--yaw', '0.0',
            '--frame-id', 'camera_link',
            '--child-frame-id', 'camera_imu_frame',
        ],
        parameters=[{'use_sim_time': True}],
        output='screen',
    )

    obstacle_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='obstacle_type_bridge',
        arguments=[
            '/obstacle_type@std_msgs/msg/String]gz.msgs.StringMsg',
            '/obstacle_type/status@std_msgs/msg/String[gz.msgs.StringMsg',
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
        condition=IfCondition(LaunchConfiguration('rviz')),
    )

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        parameters=[
            {
                'robot_description': robot_description,
                'use_sim_time': True,
                'ignore_timestamp': True,
            }
        ],
        output='screen',
    )
    return [
        gazebo,
        spawn_robot,
        bridge,
        image_bridge,
        camera_link_tf,
        camera_optical_tf,
        camera_imu_tf,
        obstacle_bridge,
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
            'Corridor width or environment type. '
            'Allowed values: 0.70, 0.90, 1.20, arena'
        ),
    )

    obstacle_arg = DeclareLaunchArgument(
        'obstacle',
        default_value='human',
        description=(
            'Dynamic obstacle type. '
            'Allowed values: human (people/pedestrian), object (industrial cylinder/box), none'
        ),
        choices=['human', 'object', 'none', 'people', 'pedestrian', 'box', 'cylinder'],
    )

    gui_arg = DeclareLaunchArgument(
        'gui',
        default_value='true',
        description='Set to false to run Gazebo in headless mode (server only, much faster)',
    )

    rviz_arg = DeclareLaunchArgument(
        'rviz',
        default_value='true',
        description='Set to false to disable RViz2',
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

    ros_domain = SetEnvironmentVariable(
        name='ROS_DOMAIN_ID',
        value=EnvironmentVariable('ROS_DOMAIN_ID', default_value='42'),
    )

    # ---------------------------------------------------------
    # Launch description
    # ---------------------------------------------------------
    return LaunchDescription([
        width_arg,
        obstacle_arg,
        gui_arg,
        rviz_arg,
        resource_path,
        system_plugin_path,
        ros_domain,
        OpaqueFunction(function=launch_setup),
    ])
