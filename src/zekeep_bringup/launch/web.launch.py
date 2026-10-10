from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import AnyLaunchDescriptionSource
from launch.substitutions import FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    """Launch rosbridge and the local model web server."""
    bringup_share = FindPackageShare("zekeep_bringup")
    web_server = PathJoinSubstitution([bringup_share, "web", "server.js"])
    rosbridge_launch = PathJoinSubstitution(
        [FindPackageShare("rosbridge_server"), "launch", "rosbridge_websocket_launch.xml"]
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("start_ai", default_value="true"),
            IncludeLaunchDescription(
                AnyLaunchDescriptionSource(rosbridge_launch),
                # Quote bracketed lists so XML passes STRING parameters to the glob parsers.
                launch_arguments={
                    "port": "9090", "address": "127.0.0.1",
                    "call_services_in_new_thread": "true",
                    "send_action_goals_in_new_thread": "true",
                    "topics_glob": '"[/zekeep/*,/joint_states,/tf,/tf_static,/camera/color/*,/gemini305g/color/*]"',
                    "services_glob": '"[/zekeep/*,/rosapi/*,/ZekeepController/get_parameters,/compute_ik]"',
                    "params_glob": '"[/__web_parameters_disabled__]"',
                }.items(),
            ),
            IncludeLaunchDescription(
                AnyLaunchDescriptionSource(PathJoinSubstitution([bringup_share, "launch", "web_ai.launch.py"])),
                condition=IfCondition(LaunchConfiguration("start_ai")),
            ),
            ExecuteProcess(
                cmd=[FindExecutable(name="node"), web_server],
                output="screen",
            ),
        ]
    )
