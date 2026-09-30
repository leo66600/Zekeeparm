from launch import LaunchDescription
from launch.actions import ExecuteProcess, IncludeLaunchDescription
from launch.launch_description_sources import AnyLaunchDescriptionSource
from launch.substitutions import FindExecutable, PathJoinSubstitution
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
            IncludeLaunchDescription(
                AnyLaunchDescriptionSource(rosbridge_launch),
                launch_arguments={
                    "port": "9090", "address": "127.0.0.1",
                    "topics_glob": "[/zekeep/*,/joint_states,/tf,/tf_static]",
                    "services_glob": "[/zekeep/*,/rosapi/*]",
                    "params_glob": "[/__web_parameters_disabled__]",
                }.items(),
            ),
            ExecuteProcess(
                cmd=[FindExecutable(name="node"), web_server],
                output="screen",
            ),
        ]
    )
