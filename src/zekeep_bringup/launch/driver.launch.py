from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.substitutions import FindPackageShare

from zekeep_bringup.launch_config import (
    create_common_arguments,
    create_configurations,
    create_controller_node,
)


def generate_launch_description():
    """Launch the hardware controller without visualization nodes."""
    bringup_share = FindPackageShare("zekeep_bringup")
    configurations = create_configurations()

    return LaunchDescription(
        [
            *create_common_arguments(bringup_share),
            DeclareLaunchArgument("python_executable", default_value="",
                                  description="Optional SDK Python interpreter"),
            create_controller_node(configurations,
                                   python_executable=LaunchConfiguration("python_executable")),
        ]
    )
