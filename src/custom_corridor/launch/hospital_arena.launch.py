import os

from ament_index_python.packages import (
    get_package_prefix,
    get_package_share_directory,
)

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def launch_setup(context):
    package_share = get_package_share_directory("custom_corridor")
    turtlebot_share = get_package_share_directory("turtlebot3_gazebo")
    description_share = get_package_share_directory("turtlebot3_description")

    difficulty = LaunchConfiguration("difficulty").perform(context).lower().strip()
    gui = LaunchConfiguration("gui").perform(context).lower().strip()
    rviz_enabled = LaunchConfiguration("rviz").perform(context).lower().strip()

    world_map = {
        "easy": "hospital_easy.sdf",
    }

    if difficulty not in world_map:
        raise RuntimeError(
            f"Invalid difficulty '{difficulty}'. "
            "Allowed values: easy, medium, hard"
        )

    # ---------------------------------------------------------
    # Hospital world
    # ---------------------------------------------------------
    world_file = os.path.join(
        package_share,
        "worlds",
        "hospital",
        world_map[difficulty],
    )

    if not os.path.isfile(world_file):
        raise RuntimeError(
            f"Hospital world does not exist: {world_file}"
        )

    plugin_prefix = get_package_prefix("custom_corridor_plugins")
    plugin_path = os.path.join(plugin_prefix, "lib")
    model_path = os.path.join(package_share, "models")
    turtlebot_model_path = os.path.join(turtlebot_share, "models")
    existing_plugin_path = os.environ.get("GZ_SIM_SYSTEM_PLUGIN_PATH", "")
    existing_resource_path = os.environ.get("GZ_SIM_RESOURCE_PATH", "")

    # ---------------------------------------------------------
    # TurtleBot3 Burger model
    # ---------------------------------------------------------
    burger_model = os.path.join(
        package_share,
        "models",
        "turtlebot3_burger",
        "model.sdf",
    )

    if not os.path.isfile(burger_model):
        raise RuntimeError(
            f"TurtleBot3 Burger model does not exist: {burger_model}"
        )

    # ---------------------------------------------------------
    # Bridge configuration
    # ---------------------------------------------------------
    bridge_config = os.path.join(
        turtlebot_share,
        "params",
        "turtlebot3_burger_bridge.yaml",
    )

    # ---------------------------------------------------------
    # Gazebo
    # ---------------------------------------------------------
    gz_cmd = ["gz", "sim", "-r"]

    if gui in ["false", "0", "no", "headless"]:
        gz_cmd.append("-s")

    gz_cmd.append(world_file)

    gazebo = ExecuteProcess(
        cmd=gz_cmd,
        output="screen",
    )

    # ---------------------------------------------------------
    # Spawn TurtleBot3 Burger
    #
    # Hospital world:
    # x = -18
    # y = 0
    # yaw = 0 -> +X
    # ---------------------------------------------------------
    spawn_robot = Node(
        package="ros_gz_sim",
        executable="create",
        arguments=[
            "-name",
            "burger",
            "-file",
            burger_model,
            "-x",
            "-18.0",
            "-y",
            "0.0",
            "-z",
            "0.01",
            "-Y",
            "0.0",
        ],
        output="screen",
    )

    # ---------------------------------------------------------
    # ROS <-> Gazebo bridge
    # ---------------------------------------------------------
    bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        parameters=[
            {
                "config_file": bridge_config,
            }
        ],
        output="screen",
    )

    # ---------------------------------------------------------
    # Robot State Publisher
    # ---------------------------------------------------------
    urdf_file = os.path.join(
        description_share,
        "urdf",
        "turtlebot3_burger.urdf",
    )

    if not os.path.isfile(urdf_file):
        raise RuntimeError(
            f"TurtleBot3 URDF does not exist: {urdf_file}"
        )

    import xacro

    doc = xacro.process_file(urdf_file)
    robot_description = doc.toxml()

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        parameters=[
            {
                "robot_description": robot_description,
                "use_sim_time": True,
            }
        ],
        output="screen",
    )

    # ---------------------------------------------------------
    # RViz
    # ---------------------------------------------------------
    rviz_config = os.path.join(
        package_share,
        "rviz",
        "corridor.rviz",
    )

    rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        arguments=[
            "-d",
            rviz_config,
        ],
        parameters=[
            {
                "use_sim_time": True,
            }
        ],
        output="screen",
    )

    nodes = [
        SetEnvironmentVariable(
            name="GZ_SIM_SYSTEM_PLUGIN_PATH",
            value=os.pathsep.join(
                path for path in [plugin_path, existing_plugin_path] if path
            ),
        ),
        SetEnvironmentVariable(
            name="GZ_SIM_RESOURCE_PATH",
            value=os.pathsep.join(
                path for path in [
                    model_path,
                    turtlebot_model_path,
                    existing_resource_path,
                ] if path
            ),
        ),
        gazebo,
        spawn_robot,
        bridge,
        robot_state_publisher,
    ]

    if rviz_enabled not in ["false", "0", "no"]:
        nodes.append(rviz)

    return nodes


def generate_launch_description():

    difficulty_arg = DeclareLaunchArgument(
        "difficulty",
        default_value="easy",
        description=(
            "Hospital difficulty: easy, medium, or hard"
        ),
    )

    gui_arg = DeclareLaunchArgument(
        "gui",
        default_value="true",
        description=(
            "Set to true to show Gazebo GUI, "
            "false for headless Gazebo"
        ),
    )

    rviz_arg = DeclareLaunchArgument(
        "rviz",
        default_value="true",
        description=(
            "Set to true to launch RViz2, "
            "false to disable RViz2"
        ),
    )

    return LaunchDescription(
        [
            difficulty_arg,
            gui_arg,
            rviz_arg,
            OpaqueFunction(function=launch_setup),
        ]
    )