from launch import LaunchDescription
from launch.conditions import IfCondition
from launch.substitutions import (
    Command,
    PathJoinSubstitution,
)
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare

from zekeep_bringup.launch_config import (
    create_common_arguments,
    create_configurations,
    create_controller_node,
)


def generate_launch_description():
    """Launch the controller, robot state publisher, and optional RViz."""
    bringup_share = FindPackageShare("zekeep_bringup")
    configurations = create_configurations(include_visualization_options=True)

    urdf_file = PathJoinSubstitution(
        [bringup_share, "description", "urdf", "sixaxis.urdf"]
    )
    rviz_config = PathJoinSubstitution([bringup_share, "rviz", "zekeep.rviz"])
    robot_description = ParameterValue(Command(["cat ", urdf_file]), value_type=str)

    return LaunchDescription(
        [
            *create_common_arguments(
                bringup_share,
                include_visualization_options=True,
            ),
            create_controller_node(configurations, shutdown_on_exit=True),
            Node(
                package="robot_state_publisher",
                executable="robot_state_publisher",
                name="robot_state_publisher",
                output="screen",
                parameters=[{"robot_description": robot_description}],
                remappings=[
                    ("/joint_states", ["/", configurations["arm_namespace"], "/joint_states"])
                ],
            ),
            Node(
                package="rviz2",
                executable="rviz2",
                name="rviz2",
                output="screen",
                arguments=["-d", rviz_config],
                condition=IfCondition(configurations["use_rviz"]),
            ),
        ]
    )
