from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution


def generate_launch_description():
    config = PathJoinSubstitution(
        [FindPackageShare("zekeep_joystick"), "config", "zikway.yaml"]
    )
    return LaunchDescription(
        [
            DeclareLaunchArgument("config", default_value=config),
            Node(
                package="joy",
                executable="joy_node",
                name="joy_node",
                parameters=[{"deadzone": 0.05, "autorepeat_rate": 20.0}],
            ),
            Node(
                package="zekeep_joystick",
                executable="joystick_control",
                name="zekeep_joystick",
                parameters=[LaunchConfiguration("config")],
                output="screen",
            ),
        ]
    )
