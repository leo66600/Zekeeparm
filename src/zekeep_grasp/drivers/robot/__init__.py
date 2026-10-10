"""Robot drivers with optional SDK-backed imports."""

from typing import Any

from .grasp_driver import (
    GRIPPER_MAX_DISTANCE_M,
    ArmJointMapping,
    GraspDriver,
    SelectedArmConfig,
    ensure_rebot_sdk_in_syspath,
    find_rebot_repo_root,
    gripper_distance_to_motor_position,
    install_arm_joint_mapping,
    selected_arm_config,
    selected_hardware_yaml,
)
__all__ = [
    "GRIPPER_MAX_DISTANCE_M",
    "ArmJointMapping",
    "GraspDriver",
    "SelectedArmConfig",
    "ensure_rebot_sdk_in_syspath",
    "find_rebot_repo_root",
    "gripper_distance_to_motor_position",
    "install_arm_joint_mapping",
    "selected_arm_config",
    "selected_hardware_yaml",
    "OfficialSdkRobot",
]


def __getattr__(name: str) -> Any:
    """Load optional SDK-backed objects only when callers request them."""
    if name == "OfficialSdkRobot":
        from .official_sdk_robot import OfficialSdkRobot

        return OfficialSdkRobot
    raise AttributeError(f"module {__name__!r} has no attribute {name!r}")
