import os
import sys
import tempfile
import xacro


# Also installed beside these launch files and copied with legacy camera launches.
sys.path.insert(0, os.path.dirname(os.path.realpath(__file__)))
from _dynamuhsim_simulation import (
    robot_assets, gazebo_command, spawn_node, bridge_node, state_publisher_node, rviz_node,
    image_bridge_node, camera_tf_nodes, obstacle_bridge_node,
)
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
        'dataset': 'arena_dataset.sdf',
        'rvo2': 'arena_rvo2.sdf',
        'school': os.path.join('school_arena', 'school_arena.sdf'),
        'school_arena': os.path.join('school_arena', 'school_arena.sdf'),
    }

    if width.lower() in ['arena', 'open', 'big', 'wide', 'dataset', 'rvo2', 'social', 'school', 'school_arena']:
        if width.lower() == 'dataset':
            width = 'dataset'
        elif width.lower() in ['rvo2', 'social']:
            width = 'rvo2'
        elif width.lower() in ['school', 'school_arena']:
            width = 'school'
        elif width.lower() in ['arena', 'open', 'big', 'wide']:
            width = 'arena'
    else:
        try:
            width = f"{float(width):.2f}"
        except ValueError:
            raise RuntimeError(
                f"Invalid width '{width}'. "
                "Allowed values: 0.70, 0.90, 1.20, arena, dataset, rvo2, school"
            )

    if width not in world_map:
        raise RuntimeError(
            f"Invalid width '{width}'. "
            "Allowed values: 0.70, 0.90, 1.20, arena, dataset, rvo2, school"
        )

    # ---------------------------------------------------------
    # Robot model selection (mecanum or burger)
    # ---------------------------------------------------------
    robot_model = LaunchConfiguration('robot_model').perform(context).lower().strip()
    if robot_model not in ['burger', 'tb3', 'turtlebot3']:
        robot_model = 'mecanum'

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

    custom_model = (LaunchConfiguration('custom_robot_model').perform(context).strip()
                    if robot_model != 'burger' else '')
    robot = robot_assets(robot_model, corridor_share, description_share,
                         os.path.join(corridor_share, 'config', 'turtlebot3_burger_bridge.yaml'),
                         custom_model)

    doc = xacro.process_file(robot.urdf)
    robot_description = doc.toxml()
    # ---------------------------------------------------------
    # World
    # ---------------------------------------------------------
    custom_world = LaunchConfiguration('custom_world').perform(context).strip()
    if custom_world and os.path.isfile(custom_world):
        raw_world_file = custom_world
    else:
        raw_world_file = os.path.join(
            corridor_share,
            'worlds',
            world_map[width]
        )
        if not os.path.isfile(raw_world_file):
            alt_world = os.path.join(corridor_share, 'worlds', 'arena_dataset', world_map[width])
            if os.path.isfile(alt_world):
                raw_world_file = alt_world

    world_file = raw_world_file
    if obstacle in ['object', 'cylinder', 'box', 'none', 'clean', 'empty']:
        with open(raw_world_file, 'r') as f:
            content = f.read()
        import re
        filtered_content = re.sub(r'<actor name="(?:human_actor|human_\w+_visual)">.*?</actor>', '', content, flags=re.DOTALL)
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
            filtered_content = re.sub(r'<model name="human_\w+_proxy">.*?</model>', '', filtered_content, flags=re.DOTALL)
            filtered_content = re.sub(r'<plugin\s+filename="librvo2_human_system\.so".*?</plugin>', '', filtered_content, flags=re.DOTALL)
            filtered_content = filtered_content.replace(
                '<pose>5.0 0.0 0.85 0 0 3.14159</pose>',
                '<pose>5.0 0.0 -50.0 0 0 3.14159</pose>'
            )
            filtered_content = filtered_content.replace(
                '<default_mode>human</default_mode>',
                '<default_mode>none</default_mode>'
            )
        uid = getattr(os, 'getuid', lambda: 1000)()
        active_world = os.path.join(tempfile.gettempdir(), f"corridor_{uid}_{width}_{obstacle}.sdf")
        with open(active_world, 'w') as f:
            f.write(filtered_content)
        world_file = active_world

    gui = LaunchConfiguration('gui').perform(context).lower().strip()
    paused = LaunchConfiguration('paused').perform(context).lower().strip()
    gz_cmd = gazebo_command(world_file, gui, paused)

    local_models = os.path.join(corridor_share, 'models')
    gazebo_models = os.path.join(turtlebot_share, 'models')
    plugin_share = get_package_share_directory('custom_corridor_plugins')
    plugin_lib = os.path.join(os.path.dirname(os.path.dirname(plugin_share)), 'lib')

    gz_resource_path = os.pathsep.join(
        p for p in [local_models, gazebo_models, os.environ.get('GZ_SIM_RESOURCE_PATH', '')] if p
    )
    gz_plugin_path = os.pathsep.join(
        p for p in [plugin_lib, os.environ.get('GZ_SIM_SYSTEM_PLUGIN_PATH', '')] if p
    )

    gazebo = ExecuteProcess(
        cmd=gz_cmd,
        output='screen',
        additional_env={
            'GZ_OBSTACLE_TYPE': obstacle,
            'GZ_SIM_RESOURCE_PATH': gz_resource_path,
            'GZ_SIM_SYSTEM_PLUGIN_PATH': gz_plugin_path,
            'GALLIUM_DRIVER': 'd3d12',
            'MESA_D3D12_DEFAULT_ADAPTER_NAME': 'NVIDIA',
        },
    )

    # ---------------------------------------------------------
    # Spawn Robot
    #
    # Corridor / Arena: (-13.0, 0.0)
    # School: (-13.5, -8.15)
    # ---------------------------------------------------------
    x_val = LaunchConfiguration('x').perform(context).strip()
    y_val = LaunchConfiguration('y').perform(context).strip()
    yaw_val = LaunchConfiguration('yaw').perform(context).strip()

    if width == 'school':
        spawn_x = x_val if x_val else '-13.5'
        spawn_y = y_val if y_val else '-8.15'
        spawn_yaw = yaw_val if yaw_val else '0.0'
    else:
        spawn_x = x_val if x_val else '-13.0'
        spawn_y = y_val if y_val else '0.0'
        spawn_yaw = yaw_val if yaw_val else '0.0'

    spawn_robot = spawn_node(robot, spawn_x, spawn_y, spawn_yaw)

    # ---------------------------------------------------------
    # ROS <-> Gazebo bridge
    # ---------------------------------------------------------
    bridge = bridge_node(robot.bridge)

    # ros_gz_image provides efficient RGB and float depth image conversion.
    # Names follow the RealSense ROS convention and both images are aligned.
    image_bridge = image_bridge_node()
    camera_link_tf, camera_optical_tf, camera_imu_tf = camera_tf_nodes()

    obstacle_bridge = obstacle_bridge_node()

    # ---------------------------------------------------------
    # RViz
    # ---------------------------------------------------------
    rviz_config = os.path.join(
        corridor_share,
        'rviz',
        'corridor.rviz',
    )

    rviz = rviz_node(rviz_config)

    robot_state_publisher = state_publisher_node(robot_description, ignore_timestamp=True)
    rviz_enabled = LaunchConfiguration('rviz').perform(context).lower().strip()
    nodes = [
        gazebo,
        spawn_robot,
        bridge,
        image_bridge,
        camera_link_tf,
        camera_optical_tf,
        camera_imu_tf,
        obstacle_bridge,
        robot_state_publisher,
    ]
    if rviz_enabled not in ['false', '0', 'no']:
        nodes.append(rviz)
    return nodes

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
            'Allowed values: 0.70, 0.90, 1.20, arena, dataset'
        ),
    )

    obstacle_arg = DeclareLaunchArgument(
        'obstacle',
        default_value='human',
        description=(
            'Dynamic obstacle type. '
            'Allowed values: human (people/pedestrian), object (industrial cylinder/box), none'
        ),
        choices=['human', 'object', 'none', 'clean', 'empty', 'people', 'pedestrian', 'box', 'cylinder'],
    )

    gui_arg = DeclareLaunchArgument(
        'gui',
        default_value='true',
        description='Set to true to show Gazebo GUI window, false for headless Gazebo (server only)',
    )

    rviz_arg = DeclareLaunchArgument(
        'rviz',
        default_value='true',
        description='Set to true to launch RViz2 (default true), false to disable',
    )

    x_arg = DeclareLaunchArgument(
        'x',
        default_value='',
        description='Initial robot spawn X coordinate (defaults to -13.5 for school, -13.0 for corridor/arena)',
    )

    y_arg = DeclareLaunchArgument(
        'y',
        default_value='',
        description='Initial robot spawn Y coordinate (defaults to -8.15 for school, 0.0 for corridor/arena)',
    )

    yaw_arg = DeclareLaunchArgument(
        'yaw',
        default_value='',
        description='Initial robot spawn Yaw angle in radians (default 0.0)',
    )

    robot_model_arg = DeclareLaunchArgument(
        'robot_model',
        default_value='mecanum',
        description='Robot model to spawn: mecanum (default) or burger',
    )

    custom_world_arg = DeclareLaunchArgument(
        'custom_world',
        default_value='',
        description='Optional direct path to custom SDF world file',
    )

    custom_robot_model_arg = DeclareLaunchArgument(
        'custom_robot_model',
        default_value='',
        description='Optional direct path to custom SDF robot model file',
    )

    paused_arg = DeclareLaunchArgument(
        'paused',
        default_value='false',
        description='Start Gazebo in paused mode (true or false)',
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
     value=os.environ.get('ROS_DOMAIN_ID', '0')
    )

    discovery_range = SetEnvironmentVariable(
     name='ROS_AUTOMATIC_DISCOVERY_RANGE',
     value=os.environ.get('ROS_AUTOMATIC_DISCOVERY_RANGE', 'LOCALHOST')
    )

    gallium_driver = SetEnvironmentVariable(
        name='GALLIUM_DRIVER',
        value='d3d12'
    )

    mesa_adapter = SetEnvironmentVariable(
        name='MESA_D3D12_DEFAULT_ADAPTER_NAME',
        value='NVIDIA'
    )

    # ---------------------------------------------------------
    # Launch description
    # ---------------------------------------------------------
    return LaunchDescription([
     gallium_driver,
     mesa_adapter,
     rmw_implementation,
     fastdds_transport,
     ros_domain,
     discovery_range,

     width_arg,
     obstacle_arg,
     gui_arg,
     rviz_arg,
     robot_model_arg,
     custom_world_arg,
     custom_robot_model_arg,
     paused_arg,
     x_arg,

     y_arg,
     yaw_arg,
     resource_path,
     system_plugin_path,
     OpaqueFunction(function=launch_setup),
    ])
