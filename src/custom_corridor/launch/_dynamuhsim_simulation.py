"""Robot assets and launch actions shared by the corridor and hospital worlds."""
import os
from typing import NamedTuple

from launch_ros.actions import Node


class RobotAssets(NamedTuple):
    entity: str
    sdf: str
    bridge: str
    urdf: str
    spawn_z: str


def robot_assets(model, package_share, description_share, burger_bridge, custom_model=""):
    if model == "burger":
        return RobotAssets(
            "burger", os.path.join(package_share, "models", "turtlebot3_burger", "model.sdf"),
            burger_bridge, os.path.join(description_share, "urdf", "turtlebot3_burger.urdf"), "0.01",
        )
    selected = custom_model if custom_model and os.path.isfile(custom_model) else os.path.join(package_share, "models", "mecanum_robot", "model.sdf")
    return RobotAssets(
        "mecanum_robot", selected, os.path.join(package_share, "config", "mecanum_bridge.yaml"),
        os.path.join(package_share, "urdf", "mecanum_robot.urdf"), "0.04",
    )


def gazebo_command(world_file, gui, paused="false"):
    command = ["gz", "sim"]
    if paused not in ["true", "1", "yes"]:
        command.append("-r")
    if gui in ["false", "0", "no", "headless"]:
        command.append("-s")
    return command + [world_file]


def spawn_node(robot, x, y, yaw):
    return Node(
        package="ros_gz_sim", executable="create", output="screen",
        arguments=["-name", robot.entity, "-file", robot.sdf,
                   "-x", x, "-y", y, "-z", robot.spawn_z, "-Y", yaw],
    )


def bridge_node(config):
    return Node(package="ros_gz_bridge", executable="parameter_bridge",
                parameters=[{"config_file": config}], output="screen")


def state_publisher_node(description, *, ignore_timestamp=False):
    parameters = dict(robot_description=description, use_sim_time=True)
    if ignore_timestamp:
        parameters["ignore_timestamp"] = True
    return Node(package="robot_state_publisher", executable="robot_state_publisher",
                name="robot_state_publisher", parameters=[parameters], output="screen")


def rviz_node(config):
    return Node(package="rviz2", executable="rviz2", name="rviz2",
                arguments=["-d", config], parameters=[{"use_sim_time": True}], output="screen")
