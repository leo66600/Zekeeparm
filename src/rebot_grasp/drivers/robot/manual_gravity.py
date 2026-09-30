"""Manual calibration adapter backed by the ROS hardware manager."""
from __future__ import annotations

from typing import Any

import numpy as np

from utils.transforms import quat_to_mat4


class ManualGravityRobot:
    """Expose gravity compensation and TCP pose for hand-eye collection."""

    def __init__(self, hardware: Any) -> None:
        self._hardware = hardware

    def start(self) -> None:
        try:
            self._hardware.connect()
            self._hardware.start_gravity_compensation()
            if not self._hardware.gravity_compensation_active():
                raise RuntimeError("gravity compensation did not become active")
            if not self._hardware.control_loop_active:
                raise RuntimeError("gravity compensation control loop is not running")
        except Exception:
            self._hardware.shutdown()
            raise

    def get_tcp_pose(self) -> np.ndarray:
        pose = self._hardware.current_pose()
        return quat_to_mat4(
            pose.position.x,
            pose.position.y,
            pose.position.z,
            pose.orientation.x,
            pose.orientation.y,
            pose.orientation.z,
            pose.orientation.w,
        )

    def safe_home(self) -> None:
        self._hardware.stop_gravity_compensation()
        self._hardware.shutdown()


def make_manual_gravity_robot(
    hardware_config: str | None = None,
    model: str = "",
    channel: str = "",
    manager_cls=None,
) -> ManualGravityRobot:
    if manager_cls is None:
        from zekeepcontroller.hardware_manager import HardwareManager

        manager_cls = HardwareManager
    hardware = manager_cls(
        hardware_config=hardware_config,
        model=model,
        channel=channel,
    )
    return ManualGravityRobot(hardware)
