"""Shared launch configuration for Zekeep controller entry points."""

from __future__ import annotations

from collections.abc import Mapping
from typing import Any

from launch.actions import DeclareLaunchArgument, Shutdown
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def create_configurations(
    *,
    include_visualization_options: bool = False,
) -> dict[str, LaunchConfiguration]:
    """Create LaunchConfiguration objects used by controller launch files."""
    names = [
        "hardware_config",
        "model",
        "channel",
        "auto_enable",
        "joint_state_rate",
        "cmd_arbitration",
        "arm_namespace",
        "disable_after_safe_home",
    ]
    if include_visualization_options:
        names.extend(("use_rviz", "frame_id", "ee_frame_id"))
    return {name: LaunchConfiguration(name) for name in names}


def create_common_arguments(
    bringup_share: Any,
    *,
    include_visualization_options: bool = False,
) -> list[DeclareLaunchArgument]:
    """Declare controller arguments shared by driver and full bringup."""
    arguments = [
        DeclareLaunchArgument(
            "hardware_config",
            default_value=PathJoinSubstitution(
                [bringup_share, "config", "zekeep_hardware.yaml"]
            ),
        ),
        DeclareLaunchArgument("model", default_value="sixaxis"),
        DeclareLaunchArgument("channel", default_value=""),
        DeclareLaunchArgument(
            "auto_enable",
            default_value="false",
            description="Enable motor control automatically after connecting",
        ),
        DeclareLaunchArgument("joint_state_rate", default_value="100.0"),
        DeclareLaunchArgument("cmd_arbitration", default_value="reject"),
        DeclareLaunchArgument("arm_namespace", default_value="zekeep"),
        DeclareLaunchArgument("disable_after_safe_home", default_value="true"),
    ]
    if include_visualization_options:
        arguments.extend(
            [
                DeclareLaunchArgument("use_rviz", default_value="false"),
                DeclareLaunchArgument("frame_id", default_value="base_link"),
                DeclareLaunchArgument("ee_frame_id", default_value="link6"),
            ]
        )
    return arguments


def create_controller_node(
    configurations: Mapping[str, LaunchConfiguration],
    *,
    shutdown_on_exit: bool = False,
    python_executable: Any = None,
) -> Node:
    """Create the Zekeep controller node from shared launch values."""
    parameters = {
        "hardware_config": configurations["hardware_config"],
        "model": configurations["model"],
        "channel": configurations["channel"],
        "auto_enable": ParameterValue(
            configurations["auto_enable"],
            value_type=bool,
        ),
        "joint_state_rate": configurations["joint_state_rate"],
        "cmd_arbitration": configurations["cmd_arbitration"],
        "arm_namespace": configurations["arm_namespace"],
        "disable_after_safe_home": ParameterValue(
            configurations["disable_after_safe_home"],
            value_type=bool,
        ),
    }
    for name in ("frame_id", "ee_frame_id"):
        if name in configurations:
            parameters[name] = configurations[name]

    node_arguments: dict[str, Any] = {
        "package": "zekeepcontroller",
        "executable": "ZekeepController",
        "name": "ZekeepController",
        "output": "screen",
        # Infinity: failed homing must retain holding torque, never force-kill.
        "sigterm_timeout": "1e309",
        "sigkill_timeout": "1e309",
        "parameters": [parameters],
    }
    if shutdown_on_exit:
        node_arguments["on_exit"] = Shutdown(
            reason="ZekeepController exited"
        )
    if python_executable is not None:
        node_arguments["prefix"] = python_executable
    return Node(**node_arguments)
