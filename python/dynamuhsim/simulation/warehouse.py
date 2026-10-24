"""Common world/launch setup for existing warehouse Nav2 pilots.

The camera/dataset launch and its frozen source are deliberately separate.
"""
from pathlib import Path
import copy
import os
import xml.etree.ElementTree as ET


def configure_world(tree, adapter_library, condition, planning_dt, *, planning_dt_text=None):
    world = tree.getroot().find("world")
    plugin = world.find("plugin[@name='social_experiment::Adapter']")
    if condition == "empty":
        world.remove(plugin)
    else:
        plugin.set("filename", str(adapter_library))
        plugin.set("name", "response_experiment::Adapter")
        ET.SubElement(plugin, "population").text = "6"
        ET.SubElement(plugin, "planning_dt").text = str(planning_dt) if planning_dt_text is None else planning_dt_text
    world.find("physics/max_step_size").text = ".0025"
    world.find("physics/real_time_factor").text = "1.0"
    for item in list(world):
        name = item.get("name", "")
        if name.startswith("socnavgym_"):
            item.set("name", name.replace("socnavgym_", "common_", 1))
    observer = world.find("model[@name='common_observer']")
    observer.find("pose").text = "-24 -5 12 0 1.17 1.57079632679"
    for tag, value in [("topic", "/common/observer"), ("update_rate", "1"),
                       ("image/width", "640"), ("image/height", "480")]:
        observer.find(".//" + tag).text = value
    return tree


def runtime_nav2_config(config):
    result = copy.deepcopy(config)
    result["amcl"]["ros__parameters"]["initial_pose"].update(x=-28., y=0., yaw=0.)
    return result


def generate_launch_description(entrypoint):
    # Keep ROS imports lazy: configuration/help/tests work without a ROS install.
    from launch import LaunchDescription
    from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
    from launch.launch_description_sources import PythonLaunchDescriptionSource
    from launch.substitutions import LaunchConfiguration
    from launch_ros.actions import Node
    from ament_index_python.packages import get_package_share_directory

    social = Path(entrypoint).resolve().parents[2]
    base = Path(get_package_share_directory("custom_corridor")) / "launch"
    sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(str(base / "corridor_tb3.launch.py")),
        launch_arguments=dict(width="arena", custom_world=os.environ["NAV2_PILOT_WORLD"],
                              robot_model="mecanum", obstacle="human", x="-28", y="0", yaw="0",
                              gui=LaunchConfiguration("gui"), rviz="false").items())
    nav = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(str(base / "nav2_corridor.launch.py")),
        launch_arguments=dict(map=str(social / "warehouse/warehouse.yaml"),
                              params_file=os.environ["NAV2_PILOT_PARAMS"], robot_model="mecanum",
                              controller="mppi", episode_initial_pose="true").items())
    bridge = Node(package="ros_gz_bridge", executable="parameter_bridge", name="nav2_pilot_ground_truth_bridge",
                  arguments=["/common/ground_truth@std_msgs/msg/String[gz.msgs.StringMsg"], output="screen")
    return LaunchDescription([DeclareLaunchArgument("gui", default_value="false"), sim, bridge,
                              TimerAction(period=6., actions=[nav])])
