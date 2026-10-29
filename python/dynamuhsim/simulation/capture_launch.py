"""Native Mecanum warehouse launch with an explicit camera model API.

The URDF owns camera_link. Only optical/IMU static transforms are added here.
Historical launch files and generated source drivers are not imported.
"""
import importlib.util
import os
from pathlib import Path


def generate_launch_description():
    import xacro
    from launch import LaunchDescription
    from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription, TimerAction
    from launch.launch_description_sources import PythonLaunchDescriptionSource
    from launch.substitutions import LaunchConfiguration
    from launch_ros.actions import Node
    from ament_index_python.packages import get_package_share_directory

    repo = Path(__file__).resolve().parents[3]
    share = Path(get_package_share_directory("custom_corridor"))
    turtle = Path(get_package_share_directory("turtlebot3_gazebo"))
    plugin_share = Path(get_package_share_directory("custom_corridor_plugins"))
    spec = importlib.util.spec_from_file_location("_capture_simulation_nodes", share / "launch/_dynamuhsim_simulation.py")
    simulation_nodes = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(simulation_nodes)
    resource = os.pathsep.join(filter(None, [str(share / "models"), str(turtle / "models"), os.getenv("GZ_SIM_RESOURCE_PATH")]))
    plugins = os.pathsep.join(filter(None, [str(plugin_share.parents[1] / "lib"), os.getenv("GZ_SIM_SYSTEM_PLUGIN_PATH")]))
    # GUI is a separately owned process after readiness, as in the group runner.
    sim = ExecuteProcess(cmd=simulation_nodes.gazebo_command(os.environ["NAV2_PILOT_WORLD"], "false"), output="screen",
                         additional_env=dict(GZ_OBSTACLE_TYPE="human", GZ_SIM_RESOURCE_PATH=resource,
                                             GZ_SIM_SYSTEM_PLUGIN_PATH=plugins, GALLIUM_DRIVER="d3d12",
                                             MESA_D3D12_DEFAULT_ADAPTER_NAME="NVIDIA"))
    robot = simulation_nodes.RobotAssets("mecanum_robot", os.environ["DATASET_ROBOT_SDF"],
                                    str(share / "config/mecanum_bridge.yaml"),
                                    os.environ.get("DATASET_ROBOT_URDF", str(share / "urdf/mecanum_robot.urdf")), "0.04")
    spawn = simulation_nodes.spawn_node(robot, "-28", "0", "0")
    bridge = simulation_nodes.bridge_node(robot.bridge)
    image = simulation_nodes.image_bridge_node()
    optical, imu = simulation_nodes.camera_tf_nodes(mount_tf=False, zero="0", optical_sim_time=True)
    state = simulation_nodes.state_publisher_node(
        xacro.process_file(robot.urdf).toxml(), ignore_timestamp=True)
    obstacle = simulation_nodes.obstacle_bridge_node()
    truth = Node(package="ros_gz_bridge", executable="parameter_bridge", name="nav2_pilot_ground_truth_bridge",
                 arguments=["/common/ground_truth@std_msgs/msg/String[gz.msgs.StringMsg"], output="screen")
    nav = IncludeLaunchDescription(PythonLaunchDescriptionSource(str(share / "launch/nav2_corridor.launch.py")),
             launch_arguments=dict(map=os.environ.get("DYNAMUHSIM_CAPTURE_MAP", str(repo / "experiments/social_navigation/warehouse/warehouse.yaml")),
                                   params_file=os.environ["NAV2_PILOT_PARAMS"], robot_model="mecanum",
                                   controller="mppi", episode_initial_pose="true").items())
    return LaunchDescription([DeclareLaunchArgument("gui", default_value="false"), sim, spawn, bridge, image,
                              optical, imu, state, obstacle, truth, TimerAction(period=6., actions=[nav])])
