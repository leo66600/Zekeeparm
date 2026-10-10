"""Start optional web task services; this launch never starts the motor controller."""
import os
import shlex
from pathlib import Path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import EnvironmentVariable, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    miniforge_dir = Path(os.environ.get('ZKEEP_MINIFORGE_DIR') or Path.home() / 'miniforge3').expanduser()
    defaults = {
        "motion_authorized": "false",
        "start_vision": "false",
        "grasp_geometry_confirmed": "false",
        "grasp_backend": "fixed",
        "perception_root": str(Path(__file__).resolve().parents[2] / "zekeep_grasp"),
        "perception_config": "config/default.yaml",
        "python_executable": EnvironmentVariable("ZKEEP_VISION_PYTHON", default_value=
            shlex.quote(str(miniforge_dir / 'envs/rebotarm/bin/python'))),
        "grasp_rpy_rad": "[0.0, 0.0, 0.0]",
        "object_size_m": "[0.03, 0.03, 0.03]",
    }
    return LaunchDescription([
        *(DeclareLaunchArgument(name, default_value=value) for name, value in defaults.items()),
        Node(package="zekeep_teach", executable="web_tasks", output="screen",
             prefix=LaunchConfiguration("python_executable"), parameters=[{
                 "motion_authorized": ParameterValue(LaunchConfiguration("motion_authorized"), value_type=bool),
                 "grasp_geometry_confirmed": ParameterValue(LaunchConfiguration("grasp_geometry_confirmed"), value_type=bool),
                 "grasp_backend": LaunchConfiguration("grasp_backend"),
                 "perception_root": LaunchConfiguration("perception_root"),
                 "perception_config": LaunchConfiguration("perception_config"),
                 "grasp_rpy_rad": LaunchConfiguration("grasp_rpy_rad"),
                 "object_size_m": LaunchConfiguration("object_size_m"),
             }]),
        Node(package="zekeep_shadow", executable="web_vision", output="screen",
             condition=IfCondition(LaunchConfiguration("start_vision")),
             prefix=LaunchConfiguration("python_executable"), parameters=[{
                 "perception_root": LaunchConfiguration("perception_root"),
                 "config": LaunchConfiguration("perception_config"),
                 "grasp_backend": LaunchConfiguration("grasp_backend"),
             }]),
    ])
