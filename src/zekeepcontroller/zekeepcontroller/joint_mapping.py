from __future__ import annotations

from dataclasses import dataclass
from typing import Iterable

import numpy as np


_LIMIT_EPSILON_RAD = 1e-3


@dataclass(frozen=True)
class JointCalibration:
    """Motor-to-ROS calibration and safety limits for one joint."""
    name: str
    direction: float = 1.0
    gear_ratio: float = 1.0
    encoder_zero: float = 0.0
    ros_zero: float = 0.0
    hard_lower: float = -np.inf
    hard_upper: float = np.inf
    max_velocity: float = np.inf

    def __post_init__(self) -> None:
        if self.direction not in (-1.0, 1.0):
            raise ValueError(f"{self.name} direction must be -1 or 1")
        if self.gear_ratio <= 0.0:
            raise ValueError(f"{self.name} gear_ratio must be positive")
        if self.hard_lower > self.hard_upper:
            raise ValueError(f"{self.name} hard_lower must not exceed hard_upper")
        if self.max_velocity <= 0.0:
            raise ValueError(f"{self.name} max_velocity must be positive")


class JointMapping:
    """Convert positions, velocities, and efforts between ROS and motor space."""
    def __init__(self, calibrations: Iterable[JointCalibration]) -> None:
        self.calibrations = tuple(calibrations)
        names = [calibration.name for calibration in self.calibrations]
        if not names:
            raise ValueError("joint mapping requires at least one calibration")
        if len(names) != len(set(names)):
            raise ValueError("joint mapping contains duplicate joint names")

        self._directions = np.array(
            [calibration.direction for calibration in self.calibrations],
            dtype=np.float64,
        )
        self._ratios = np.array(
            [calibration.gear_ratio for calibration in self.calibrations],
            dtype=np.float64,
        )
        self._encoder_zeros = np.array(
            [calibration.encoder_zero for calibration in self.calibrations],
            dtype=np.float64,
        )
        self._ros_zeros = np.array(
            [calibration.ros_zero for calibration in self.calibrations],
            dtype=np.float64,
        )
        self._max_velocities = np.array(
            [calibration.max_velocity for calibration in self.calibrations],
            dtype=np.float64,
        )

    @property
    def names(self) -> list[str]:
        """Return calibrated joint names in controller order."""
        return [calibration.name for calibration in self.calibrations]

    def positions_to_ros(self, motor_positions) -> np.ndarray:
        """Convert motor positions to ROS coordinates."""
        motor = self._vector(motor_positions, "motor positions")
        return (
            self._directions * self._ratios * (motor - self._encoder_zeros)
            + self._ros_zeros
        )

    def positions_to_motor(self, ros_positions) -> np.ndarray:
        """Validate and convert ROS positions to motor coordinates."""
        ros = self._vector(ros_positions, "ROS positions")
        self.validate_positions(ros)
        for index, calibration in enumerate(self.calibrations):
            ros[index] = np.clip(ros[index], calibration.hard_lower, calibration.hard_upper)
        return (
            (ros - self._ros_zeros) / (self._directions * self._ratios)
            + self._encoder_zeros
        )

    def velocities_to_ros(self, motor_velocities) -> np.ndarray:
        """Convert motor velocities to ROS coordinates."""
        motor = self._vector(motor_velocities, "motor velocities")
        return self._directions * self._ratios * motor

    def velocities_to_motor(self, ros_velocities) -> np.ndarray:
        """Convert ROS velocities to motor coordinates."""
        ros = self._vector(ros_velocities, "ROS velocities")
        return ros / (self._directions * self._ratios)

    def clamp_velocities(self, ros_velocities) -> np.ndarray:
        """Clamp ROS velocities to configured joint limits."""
        velocities = self._vector(ros_velocities, "ROS velocities")
        return np.clip(velocities, -self._max_velocities, self._max_velocities)

    @property
    def motor_velocity_limits(self) -> np.ndarray:
        """Return equivalent motor-space velocity limits."""
        return np.abs(self._max_velocities / self._ratios)

    @property
    def ros_velocity_limits(self) -> np.ndarray:
        """Return configured ROS-space velocity limits."""
        return self._max_velocities.copy()

    def efforts_to_ros(self, motor_efforts) -> np.ndarray:
        """Convert motor efforts to ROS coordinates."""
        motor = self._vector(motor_efforts, "motor efforts")
        return motor / (self._directions * self._ratios)

    def efforts_to_motor(self, ros_efforts) -> np.ndarray:
        """Convert ROS efforts to motor coordinates."""
        ros = self._vector(ros_efforts, "ROS efforts")
        return ros * self._directions * self._ratios

    def validate_positions(self, ros_positions) -> None:
        """Raise ``ValueError`` when ROS positions exceed hard limits."""
        positions = self._vector(ros_positions, "ROS positions")
        for calibration, position in zip(self.calibrations, positions):
            if (
                position < calibration.hard_lower - _LIMIT_EPSILON_RAD
                or position > calibration.hard_upper + _LIMIT_EPSILON_RAD
            ):
                raise ValueError(
                    f"{calibration.name} target {position:g} outside hard limits "
                    f"[{calibration.hard_lower:g}, {calibration.hard_upper:g}]"
                )

    def _vector(self, values, label: str) -> np.ndarray:
        vector = np.asarray(values, dtype=np.float64).reshape(-1)
        if vector.size != len(self.calibrations):
            raise ValueError(
                f"{label} must contain {len(self.calibrations)} values, "
                f"got {vector.size}"
            )
        if not np.all(np.isfinite(vector)):
            raise ValueError(f"{label} must contain only finite values")
        return vector


def joint_mapping_from_config(config: dict, joint_names: Iterable[str]) -> JointMapping:
    """Create a joint mapping from hardware configuration data."""
    configured = config.get("joint_calibration", {}) or {}
    calibrations = []
    for name in joint_names:
        if name not in configured:
            raise ValueError(f"missing calibration for {name}")
        values = configured[name] or {}
        calibrations.append(
            JointCalibration(
                name=name,
                direction=float(values.get("direction", 1.0)),
                gear_ratio=float(values.get("gear_ratio", 1.0)),
                encoder_zero=float(values.get("encoder_zero", 0.0)),
                ros_zero=float(values.get("ros_zero", 0.0)),
                hard_lower=float(values.get("hard_lower", -np.inf)),
                hard_upper=float(values.get("hard_upper", np.inf)),
                max_velocity=float(values.get("max_velocity", np.inf)),
            )
        )
    return JointMapping(calibrations)
