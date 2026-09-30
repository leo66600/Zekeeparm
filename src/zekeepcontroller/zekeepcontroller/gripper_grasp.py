"""Deterministic gripper contact detection independent of ROS and hardware I/O."""

from __future__ import annotations

from dataclasses import dataclass
import math

import numpy as np


@dataclass(frozen=True)
class GripperGraspConfig:
    """Limits and gains used by deterministic gripper contact detection."""
    open_position: float
    close_position: float
    closing_torque: float
    hold_torque: float
    max_torque: float
    stall_velocity_rad_s: float
    contact_torque: float
    minimum_motion_rad: float
    hard_stop_margin_rad: float
    kp_move: float
    kd_move: float
    kd_close: float

    def __post_init__(self) -> None:
        values = {
            field: float(getattr(self, field))
            for field in (
                "open_position",
                "close_position",
                "closing_torque",
                "hold_torque",
                "max_torque",
                "stall_velocity_rad_s",
                "contact_torque",
                "minimum_motion_rad",
                "hard_stop_margin_rad",
                "kp_move",
                "kd_move",
                "kd_close",
            )
        }
        invalid = [name for name, value in values.items() if not math.isfinite(value)]
        if invalid:
            raise ValueError(
                "gripper grasp config must be finite: " + ", ".join(invalid)
            )
        if values["max_torque"] <= 0.0:
            raise ValueError("gripper grasp max_torque must be positive")
        if values["open_position"] == values["close_position"]:
            raise ValueError(
                "gripper grasp open_position and close_position must differ"
            )
        nonnegative = (
            "closing_torque",
            "hold_torque",
            "stall_velocity_rad_s",
            "contact_torque",
            "minimum_motion_rad",
            "hard_stop_margin_rad",
            "kp_move",
            "kd_move",
            "kd_close",
        )
        if any(values[name] < 0.0 for name in nonnegative):
            raise ValueError("gripper grasp gains and thresholds must be nonnegative")
        if (
            values["closing_torque"] > values["max_torque"]
            or values["hold_torque"] > values["max_torque"]
        ):
            raise ValueError(
                "gripper grasp closing/hold torque cannot exceed max_torque"
            )


@dataclass(frozen=True)
class GripperCommand:
    """One low-level command produced by the gripper state machine."""
    position: float
    velocity: float
    kp: float
    kd: float
    torque: float
    state: str


class GripperGraspState:
    """Detect contact while closing a gripper and produce safe hold commands."""
    def __init__(self, config: GripperGraspConfig) -> None:
        self.config = config
        self.state = "idle"
        self.start_position = 0.0
        self.contact_position = 0.0
        self.object_detected = False
        self._closing_sign = math.copysign(
            1.0,
            config.close_position - config.open_position,
        )
        self._closing_torque = self._signed_closing(config.closing_torque)
        self._hold_torque = self._signed_closing(config.hold_torque)

    @property
    def done(self) -> bool:
        """Return whether grasp execution reached a terminal state."""
        return self.state in ("holding", "empty")

    def start(
        self,
        position: float,
        *,
        closing_torque: float | None = None,
        hold_torque: float | None = None,
    ) -> None:
        """Start a new grasp from the latest measured gripper position."""
        if not math.isfinite(float(position)):
            raise ValueError("gripper start position must be finite")
        self.start_position = float(position)
        self.contact_position = float(position)
        self.object_detected = False
        self._closing_torque = self._signed_closing(
            self.config.closing_torque
            if closing_torque is None or closing_torque <= 0.0
            else closing_torque
        )
        self._hold_torque = self._signed_closing(
            self.config.hold_torque
            if hold_torque is None or hold_torque <= 0.0
            else hold_torque
        )
        self.state = "closing"

    def update(
        self,
        *,
        position: float,
        velocity: float,
        torque: float,
    ) -> GripperCommand:
        """Consume feedback and return the next bounded gripper command."""
        position = float(position)
        velocity = float(velocity)
        torque = float(torque)
        if not all(math.isfinite(value) for value in (position, velocity, torque)):
            raise ValueError("gripper feedback must be finite")
        if self.state == "closing":
            self.contact_position = position
            moved = abs(position - self.start_position) >= self.config.minimum_motion_rad
            at_hard_stop = (
                abs(position - self.config.close_position)
                <= self.config.hard_stop_margin_rad
            )
            started_at_hard_stop = (
                abs(self.start_position - self.config.close_position)
                <= self.config.hard_stop_margin_rad
            )
            stalled = (
                abs(velocity) <= self.config.stall_velocity_rad_s
                and abs(torque) >= self.config.contact_torque
            )
            if at_hard_stop and (moved or started_at_hard_stop):
                self.state = "empty"
                return self._position_command(self.config.close_position, "empty")
            if moved and stalled:
                self.state = "holding"
                self.object_detected = True
                return self._hold_command(position)
            return GripperCommand(
                position=self.config.close_position,
                velocity=0.0,
                kp=0.0,
                kd=self.config.kd_close,
                torque=self._closing_torque,
                state="closing",
            )
        if self.state == "holding":
            return self._hold_command(self.contact_position)
        if self.state == "empty":
            return self._position_command(self.config.close_position, "empty")
        return self._position_command(position, "idle")

    def _limited(self, value: float) -> float:
        if not math.isfinite(float(value)):
            raise ValueError("requested gripper torque must be finite")
        return float(np.clip(abs(float(value)), 0.0, self.config.max_torque))

    def _signed_closing(self, value: float) -> float:
        return self._closing_sign * self._limited(value)

    def _position_command(self, position: float, state: str) -> GripperCommand:
        return GripperCommand(
            position=float(position),
            velocity=0.0,
            kp=self.config.kp_move,
            kd=self.config.kd_move,
            torque=0.0,
            state=state,
        )

    def _hold_command(self, position: float) -> GripperCommand:
        return GripperCommand(
            position=float(position),
            velocity=0.0,
            kp=self.config.kp_move,
            kd=self.config.kd_move,
            torque=self._hold_torque,
            state="holding",
        )
