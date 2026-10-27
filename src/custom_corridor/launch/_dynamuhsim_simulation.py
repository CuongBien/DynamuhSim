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


def image_bridge_node():
    return Node(
        package="ros_gz_image", executable="image_bridge", name="camera_image_bridge",
        arguments=["/camera/image", "/camera/depth_image"],
        remappings=[("/camera/image", "/camera/color/image_raw"),
                    ("/camera/depth_image", "/camera/depth/image_rect_raw")],
        parameters=[{"use_sim_time": True}], output="screen",
    )


def _camera_tf_node(name, xyz, angles, parent, child, use_sim_time=False):
    arguments = ["--x", xyz[0], "--y", xyz[1], "--z", xyz[2],
                 "--roll", angles[0], "--pitch", angles[1], "--yaw", angles[2],
                 "--frame-id", parent, "--child-frame-id", child]
    return Node(package="tf2_ros", executable="static_transform_publisher", name=name,
                arguments=arguments, output="screen",
                **({"parameters": [{"use_sim_time": True}]} if use_sim_time else {}))


def camera_tf_nodes(*, mount_tf=True, zero="0.0", optical_sim_time=False):
    """Legacy launch owns the mount TF; native capture obtains it from the URDF."""
    nodes = []
    if mount_tf:
        nodes.append(_camera_tf_node("camera_link_tf", ["0.08", "0.0", "0.20"],
                                    ["0.0"] * 3, "base_link", "camera_link"))
    nodes.append(_camera_tf_node("camera_optical_tf", [zero] * 3,
                                ["-1.57079632679", zero, "-1.57079632679"],
                                "camera_link", "camera_optical_frame", optical_sim_time))
    nodes.append(_camera_tf_node("camera_imu_tf", [zero] * 3, [zero] * 3,
                                "camera_link", "camera_imu_frame", True))
    return nodes


def obstacle_bridge_node():
    return Node(package="ros_gz_bridge", executable="parameter_bridge", name="obstacle_type_bridge",
                arguments=["/obstacle_type@std_msgs/msg/String]gz.msgs.StringMsg",
                           "/obstacle_type/status@std_msgs/msg/String[gz.msgs.StringMsg"], output="screen")
