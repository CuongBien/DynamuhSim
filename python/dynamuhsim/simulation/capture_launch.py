"""Native Mecanum warehouse launch with an explicit camera model API.

The URDF owns camera_link. Only optical/IMU static transforms are added here.
Historical launch files and generated source drivers are not imported.
"""
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
    resource = os.pathsep.join(filter(None, [str(share / "models"), str(turtle / "models"), os.getenv("GZ_SIM_RESOURCE_PATH")]))
    plugins = os.pathsep.join(filter(None, [str(plugin_share.parents[1] / "lib"), os.getenv("GZ_SIM_SYSTEM_PLUGIN_PATH")]))
    # GUI is a separately owned process after readiness, as in the group runner.
    sim = ExecuteProcess(cmd=["gz", "sim", "-r", "-s", os.environ["NAV2_PILOT_WORLD"]], output="screen",
                         additional_env=dict(GZ_OBSTACLE_TYPE="human", GZ_SIM_RESOURCE_PATH=resource,
                                             GZ_SIM_SYSTEM_PLUGIN_PATH=plugins, GALLIUM_DRIVER="d3d12",
                                             MESA_D3D12_DEFAULT_ADAPTER_NAME="NVIDIA"))
    spawn = Node(package="ros_gz_sim", executable="create", arguments=["-name", "mecanum_robot", "-file",
                 os.environ["DATASET_ROBOT_SDF"], "-x", "-28", "-y", "0", "-z", "0.04", "-Y", "0"], output="screen")
    bridge = Node(package="ros_gz_bridge", executable="parameter_bridge",
                  parameters=[dict(config_file=str(share / "config/mecanum_bridge.yaml"))], output="screen")
    image = Node(package="ros_gz_image", executable="image_bridge", name="camera_image_bridge",
                 arguments=["/camera/image", "/camera/depth_image"],
                 remappings=[("/camera/image", "/camera/color/image_raw"),
                             ("/camera/depth_image", "/camera/depth/image_rect_raw")],
                 parameters=[dict(use_sim_time=True)], output="screen")
    def transform(name, xyz, angles, parent, child):
        args = ["--x", xyz[0], "--y", xyz[1], "--z", xyz[2], "--roll", angles[0], "--pitch", angles[1],
                "--yaw", angles[2], "--frame-id", parent, "--child-frame-id", child]
        return Node(package="tf2_ros", executable="static_transform_publisher", name=name,
                    arguments=args, parameters=[dict(use_sim_time=True)], output="screen")
    optical = transform("camera_optical_tf", ["0", "0", "0"], ["-1.57079632679", "0", "-1.57079632679"], "camera_link", "camera_optical_frame")
    imu = transform("camera_imu_tf", ["0", "0", "0"], ["0", "0", "0"], "camera_link", "camera_imu_frame")
    state = Node(package="robot_state_publisher", executable="robot_state_publisher", name="robot_state_publisher",
                 parameters=[dict(robot_description=xacro.process_file(str(share / "urdf/mecanum_robot.urdf")).toxml(),
                                  use_sim_time=True, ignore_timestamp=True)], output="screen")
    obstacle = Node(package="ros_gz_bridge", executable="parameter_bridge", name="obstacle_type_bridge",
                    arguments=["/obstacle_type@std_msgs/msg/String]gz.msgs.StringMsg",
                               "/obstacle_type/status@std_msgs/msg/String[gz.msgs.StringMsg"], output="screen")
    truth = Node(package="ros_gz_bridge", executable="parameter_bridge", name="nav2_pilot_ground_truth_bridge",
                 arguments=["/common/ground_truth@std_msgs/msg/String[gz.msgs.StringMsg"], output="screen")
    nav = IncludeLaunchDescription(PythonLaunchDescriptionSource(str(share / "launch/nav2_corridor.launch.py")),
             launch_arguments=dict(map=str(repo / "experiments/social_navigation/warehouse/warehouse.yaml"),
                                   params_file=os.environ["NAV2_PILOT_PARAMS"], robot_model="mecanum",
                                   controller="mppi", episode_initial_pose="true").items())
    return LaunchDescription([DeclareLaunchArgument("gui", default_value="false"), sim, spawn, bridge, image,
                              optical, imu, state, obstacle, truth, TimerAction(period=6., actions=[nav])])
