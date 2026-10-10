"""Start the web grasp stack against an independently running hardware driver."""
import os
import shlex
from pathlib import Path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import EnvironmentVariable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    miniforge_dir = Path(os.environ.get('ZKEEP_MINIFORGE_DIR') or Path.home() / 'miniforge3').expanduser()
    defaults = {
        "start_ai": "true",
        "start_moveit": "true",
        "start_vision": "true",
        "use_rviz": "false",
        "motion_authorized": "false",
        "ai_motion_authorized": "false",
        "grasp_geometry_confirmed": "false",
        "grasp_backend": "yolo-graspnet",
        "perception_root": PathJoinSubstitution([
            EnvironmentVariable("ZKEEP_WS", default_value=str(Path.cwd())),
            "src", "zekeep_grasp",
        ]),
        "perception_config": "config/default.yaml",
        "python_executable": EnvironmentVariable("ZKEEP_VISION_PYTHON", default_value=
            shlex.quote(str(miniforge_dir / 'envs/rebotarm/bin/python'))),
        "grasp_rpy_rad": "[0.0, 0.0, 0.0]",
        "object_size_m": "[0.03, 0.03, 0.03]",
    }
    launches = [
        ("zekeep_bringup", "web.launch.py", {
            "start_ai": LaunchConfiguration("start_ai"),
            # AI gets its own explicit grant and always uses system Python.
            "motion_authorized": LaunchConfiguration("ai_motion_authorized"),
            "python_executable": "/usr/bin/python3",
        }, None),
        ("zekeep_moveit_config", "hardware.launch.py", {
            "arm_namespace": "zekeep",
            "use_rviz": LaunchConfiguration("use_rviz"),
            # Initializes the existing per-gripper padding and base/table ACM.
            "publish_worktable": "true",
            "shutdown_on_moveit_exit": "false",
        }, IfCondition(LaunchConfiguration("start_moveit"))),
        ("zekeep_bringup", "web_tasks.launch.py", {
            name: LaunchConfiguration(name) for name in (
                "start_vision", "motion_authorized", "grasp_geometry_confirmed", "grasp_backend",
                "perception_root", "perception_config", "python_executable",
                "grasp_rpy_rad", "object_size_m",
            )
        }, None),
    ]
    return LaunchDescription([
        *(DeclareLaunchArgument(name, default_value=value) for name, value in defaults.items()),
        *(GroupAction([
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(PathJoinSubstitution([
                    FindPackageShare(package), "launch", filename,
                ])),
                launch_arguments=arguments.items(), condition=condition,
            ),
        ], scoped=True) for package, filename, arguments, condition in launches),
    ])
