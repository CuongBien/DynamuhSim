import os
import sys
import uuid
import xml.etree.ElementTree as ET

from ament_index_python.packages import (
    get_package_prefix,
    get_package_share_directory,
)

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, LogInfo, OpaqueFunction, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration

# Also installed beside these launch files and copied with legacy camera launches.
sys.path.insert(0, os.path.dirname(os.path.realpath(__file__)))
from _dynamuhsim_simulation import (
    robot_assets, gazebo_command, spawn_node, bridge_node, state_publisher_node, rviz_node,
)

def launch_setup(context):
    package_share = get_package_share_directory("custom_corridor")
    turtlebot_share = get_package_share_directory("turtlebot3_gazebo")
    description_share = get_package_share_directory("turtlebot3_description")

    difficulty = LaunchConfiguration("difficulty").perform(context).lower().strip()
    gui = LaunchConfiguration("gui").perform(context).lower().strip()
    rviz_enabled = LaunchConfiguration("rviz").perform(context).lower().strip()

    world_map = {
        "easy": "hospital_easy.sdf",
        "medium": "hospital_medium.sdf",
        "hard": "hospital_hard.sdf",
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

    # Validate the actual installed SDF before opening any processes.
    world = ET.parse(world_file).getroot().find("world")
    expected_world = f"hospital_{difficulty}"
    if world is None or world.get("name") != expected_world:
        raise RuntimeError(
            f"Selected {difficulty}, but SDF world name does not match "
            f"{expected_world}: {world_file}. Regenerate worlds and rebuild custom_corridor."
        )
    actors = world.findall("actor")
    expected_actor_count = {"easy": 3, "medium": 8, "hard": 16}[difficulty]
    if len(actors) != expected_actor_count:
        raise RuntimeError(
            f"{expected_world} has {len(actors)} actor tags; expected "
            f"{expected_actor_count} for this version (carts included): {world_file}"
        )
    if difficulty != "hard" and any(
        actor.get("name", "").startswith("hard_") for actor in actors
    ):
        raise RuntimeError(f"Hard actors found in {difficulty}: {world_file}")

    # All children share this partition. A fresh partition prevents this GUI,
    # spawn request and bridge discovering an older Gazebo hospital server.
    gz_partition = f"hospital_{difficulty}_{uuid.uuid4().hex}"

    plugin_prefix = get_package_prefix("custom_corridor_plugins")
    plugin_path = os.path.join(plugin_prefix, "lib")
    model_path = os.path.join(package_share, "models")
    turtlebot_model_path = os.path.join(turtlebot_share, "models")
    existing_plugin_path = os.environ.get("GZ_SIM_SYSTEM_PLUGIN_PATH", "")
    existing_resource_path = os.environ.get("GZ_SIM_RESOURCE_PATH", "")

    # ---------------------------------------------------------
    # Robot model selection (mecanum or burger)
    # ---------------------------------------------------------
    robot_model = LaunchConfiguration("robot_model").perform(context).lower().strip()
    if robot_model not in ["burger", "tb3", "turtlebot3"]:
        robot_model = "mecanum"

    robot = robot_assets(robot_model, package_share, description_share,
                         os.path.join(turtlebot_share, 'params', 'turtlebot3_burger_bridge.yaml'))

    if not os.path.isfile(robot.sdf):
        raise RuntimeError(
            f"Robot model does not exist: {robot.sdf}"
        )

    # ---------------------------------------------------------
    # Gazebo
    # ---------------------------------------------------------
    gz_cmd = gazebo_command(world_file, gui)

    gz_resource_path = os.pathsep.join(
        p for p in [model_path, turtlebot_model_path, existing_resource_path] if p
    )
    gz_plugin_path = os.pathsep.join(
        p for p in [plugin_path, existing_plugin_path] if p
    )

    gazebo = ExecuteProcess(
        cmd=gz_cmd,
        output="screen",
        additional_env={
            "GZ_PARTITION": gz_partition,
            "GZ_SIM_RESOURCE_PATH": gz_resource_path,
            "GZ_SIM_SYSTEM_PLUGIN_PATH": gz_plugin_path,
            "GALLIUM_DRIVER": "d3d12",
            "MESA_D3D12_DEFAULT_ADAPTER_NAME": "NVIDIA",
        },
    )

    # ---------------------------------------------------------
    # ---------------------------------------------------------
    # Spawn Robot
    #
    # Hospital world:
    # x = -18
    # y = 0
    # yaw = 0 -> +X
    # ---------------------------------------------------------
    spawn_robot = spawn_node(robot, "-18.0", "0.0", "0.0")

    # ---------------------------------------------------------
    # ROS <-> Gazebo bridge
    # ---------------------------------------------------------
    bridge = bridge_node(robot.bridge)

    # ---------------------------------------------------------
    # Robot State Publisher
    # ---------------------------------------------------------
    if not os.path.isfile(robot.urdf):
        raise RuntimeError(
            f"Robot URDF does not exist: {robot.urdf}"
        )

    import xacro

    doc = xacro.process_file(robot.urdf)
    robot_description = doc.toxml()

    robot_state_publisher = state_publisher_node(robot_description)

    # ---------------------------------------------------------
    # RViz
    # ---------------------------------------------------------
    rviz_config = os.path.join(
        package_share,
        "rviz",
        "corridor.rviz",
    )

    rviz = rviz_node(rviz_config)

    nodes = [
        LogInfo(msg=f"[Hospital] difficulty={difficulty}; world={expected_world}; actors={len(actors)}"),
        LogInfo(msg=f"[Hospital] SDF={os.path.realpath(world_file)}"),
        LogInfo(msg=f"[Hospital] GZ_PARTITION={gz_partition}"),
        SetEnvironmentVariable(name="GZ_PARTITION", value=gz_partition),
        SetEnvironmentVariable(
            name="GZ_SIM_SYSTEM_PLUGIN_PATH",
            value=gz_plugin_path,
        ),
        SetEnvironmentVariable(
            name="GZ_SIM_RESOURCE_PATH",
            value=gz_resource_path,
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
            "Hospital difficulty: easy, medium or hard"
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

    robot_model_arg = DeclareLaunchArgument(
        "robot_model",
        default_value="mecanum",
        description=(
            "Robot model: mecanum (small omni) or burger (TurtleBot3)"
        ),
        choices=["mecanum", "burger", "tb3", "turtlebot3"],
    )

    return LaunchDescription(
        [
            difficulty_arg,
            robot_model_arg,
            gui_arg,
            rviz_arg,
            OpaqueFunction(function=launch_setup),
        ]
    )
