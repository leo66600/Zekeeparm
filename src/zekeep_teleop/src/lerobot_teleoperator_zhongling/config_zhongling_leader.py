#!/usr/bin/env python

# Copyright 2026 The HuggingFace Inc. team. All rights reserved.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0

import math
from dataclasses import dataclass, field

from lerobot.teleoperators.config import TeleoperatorConfig

DEFAULT_JOINT_NAMES: tuple[str, ...] = (
    "shoulder_pan",
    "shoulder_lift",
    "elbow_flex",
    "wrist_flex",
    "wrist_yaw",
    "wrist_roll",
    "gripper",
)
DEFAULT_SOURCE_RANGES_RAD: tuple[tuple[float, float], ...] = ((-3.14, 3.14),) * 7
DEFAULT_PWM_RANGES: tuple[tuple[int, int, int], ...] = (
    (500, 1500, 2500),
    (500, 1500, 2500),
    (500, 1500, 2500),
    (500, 1500, 2500),
    (500, 1500, 2500),
    (500, 1500, 2500),
    (1300, 1500, 2500),
)
DEFAULT_JOINT_RANGES_DEG: tuple[tuple[float, float], ...] = tuple(
    (math.degrees(lower), math.degrees(upper))
    for lower, upper in (
        (-2.61, 2.61),
        (0.0, 3.70),
        (0.0, 3.70),
        (-1.57, 1.57),
        (-1.57, 1.57),
        (-1.57, 1.57),
        (-1.35, 0.0),
    )
)


@TeleoperatorConfig.register_subclass("zhongling_leader")
@dataclass
class ZhonglingLeaderConfig(TeleoperatorConfig):
    """Configuration for a seven-axis Zhongling serial leader arm."""

    port: str
    baudrate: int = 115200
    servo_ids: list[int] = field(default_factory=lambda: list(range(1, 8)))
    joint_names: list[str] = field(default_factory=lambda: list(DEFAULT_JOINT_NAMES))
    source_ranges_rad: list[tuple[float, float]] = field(
        default_factory=lambda: list(DEFAULT_SOURCE_RANGES_RAD)
    )
    joint_ranges_deg: list[tuple[float, float]] = field(
        default_factory=lambda: list(DEFAULT_JOINT_RANGES_DEG)
    )
    joint_directions: list[int] = field(default_factory=lambda: [1] * 7)
    joint_degrees_per_pwm: list[float | None] = field(default_factory=lambda: [None] * 7)
    pwm_ranges: list[tuple[int, int, int]] = field(default_factory=lambda: list(DEFAULT_PWM_RANGES))
    zero_deadband_pwm: int = 10
    serial_timeout_s: float = 0.02
    response_wait_s: float = 0.0
    torque_release_wait_s: float = 0.008
    command_terminator: str = "\r\n"
    release_torque_on_connect: bool = True
    startup_zero_check_ids: list[int] = field(default_factory=list)
    startup_zero_tolerance_pwm: int = 50
    use_startup_pose_as_zero: bool = False
    fixed_pwm_center_ids: list[int] = field(default_factory=list)
    use_background_read: bool = False
    background_read_interval_s: float = 0.0
    start_from_zero: bool = True
    max_step_deg: float | None = 5.0
    max_velocity_deg_s: float | None = None
    max_consecutive_read_failures: int = 5

    def __post_init__(self) -> None:
        expected = 7
        fields = {
            "servo_ids": self.servo_ids,
            "joint_names": self.joint_names,
            "source_ranges_rad": self.source_ranges_rad,
            "joint_ranges_deg": self.joint_ranges_deg,
            "joint_directions": self.joint_directions,
            "joint_degrees_per_pwm": self.joint_degrees_per_pwm,
            "pwm_ranges": self.pwm_ranges,
        }
        for name, values in fields.items():
            if len(values) != expected:
                raise ValueError(f"{name} must contain exactly {expected} values, got {len(values)}")

        if len(set(self.servo_ids)) != expected:
            raise ValueError("servo_ids must be unique")
        if any(servo_id < 0 or servo_id > 999 for servo_id in self.servo_ids):
            raise ValueError("servo_ids must be between 0 and 999")
        if len(set(self.startup_zero_check_ids)) != len(self.startup_zero_check_ids):
            raise ValueError("startup_zero_check_ids must be unique")
        if any(servo_id not in self.servo_ids for servo_id in self.startup_zero_check_ids):
            raise ValueError("startup_zero_check_ids must be present in servo_ids")
        if len(set(self.fixed_pwm_center_ids)) != len(self.fixed_pwm_center_ids):
            raise ValueError("fixed_pwm_center_ids must be unique")
        if any(servo_id not in self.servo_ids for servo_id in self.fixed_pwm_center_ids):
            raise ValueError("fixed_pwm_center_ids must be present in servo_ids")
        if len(set(self.joint_names)) != expected:
            raise ValueError("joint_names must be unique")
        if any(direction not in (-1, 1) for direction in self.joint_directions):
            raise ValueError("joint_directions values must be either -1 or 1")
        if any(scale is not None and scale <= 0 for scale in self.joint_degrees_per_pwm):
            raise ValueError("joint_degrees_per_pwm values must be positive or None")

        for index, (lower, upper) in enumerate(self.source_ranges_rad):
            if not lower < 0 < upper:
                raise ValueError(f"source_ranges_rad[{index}] must straddle zero")
        for index, (lower, upper) in enumerate(self.joint_ranges_deg):
            if lower >= upper or not lower <= 0 <= upper:
                raise ValueError(f"joint_ranges_deg[{index}] must be increasing and contain joint zero")

        if self.zero_deadband_pwm < 0:
            raise ValueError("zero_deadband_pwm cannot be negative")
        for index, pwm_range in enumerate(self.pwm_ranges):
            if len(pwm_range) != 3:
                raise ValueError(f"pwm_ranges[{index}] must contain exactly (min, center, max)")
            pwm_min, pwm_center, pwm_max = pwm_range
            if not pwm_min < pwm_center < pwm_max:
                raise ValueError(f"pwm_ranges[{index}] must satisfy min < center < max")
            if self.zero_deadband_pwm >= min(pwm_center - pwm_min, pwm_max - pwm_center):
                raise ValueError(
                    f"zero_deadband_pwm must be smaller than both half-ranges of pwm_ranges[{index}]"
                )
        if self.startup_zero_tolerance_pwm < 0:
            raise ValueError("startup_zero_tolerance_pwm cannot be negative")
        if self.baudrate <= 0:
            raise ValueError("baudrate must be positive")
        if (
            self.serial_timeout_s <= 0
            or self.response_wait_s < 0
            or self.torque_release_wait_s < 0
            or self.background_read_interval_s < 0
        ):
            raise ValueError("serial_timeout_s must be positive and wait values cannot be negative")
        if self.max_step_deg is not None and self.max_step_deg <= 0:
            raise ValueError("max_step_deg must be positive or None")
        if self.max_velocity_deg_s is not None and self.max_velocity_deg_s <= 0:
            raise ValueError("max_velocity_deg_s must be positive or None")
        if self.max_consecutive_read_failures < 1:
            raise ValueError("max_consecutive_read_failures must be at least 1")

