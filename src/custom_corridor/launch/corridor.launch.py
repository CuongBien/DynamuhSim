from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, ExecuteProcess
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory

import os


def launch_gazebo(context):
    width = LaunchConfiguration('width').perform(context)

    # Normalize input: 0.9 -> 0.90
    try:
        width = f"{float(width):.2f}"
    except ValueError:
        raise RuntimeError(
            f"Invalid width '{width}'. "
            "Allowed values: 0.70, 0.90, 1.20"
        )

    worlds = {
        '0.70': 'corridor_070.sdf',
        '0.90': 'corridor_090.sdf',
        '1.20': 'corridor_120.sdf',
    }

    if width not in worlds:
        raise RuntimeError(
            f"Invalid corridor width: {width}. "
            "Allowed values: 0.70, 0.90, 1.20"
        )

    package_dir = get_package_share_directory('custom_corridor')
    world_file = os.path.join(
        package_dir,
        'worlds',
        worlds[width]
    )

    return [
        ExecuteProcess(
            cmd=['gz', 'sim', '-r', world_file],
            output='screen'
        )
    ]


def generate_launch_description():
    width_arg = DeclareLaunchArgument(
        'width',
        default_value='0.90',
        description='Corridor width. Allowed: 0.70, 0.90, 1.20'
    )

    return LaunchDescription([
        width_arg,
        OpaqueFunction(function=launch_gazebo),
    ])
