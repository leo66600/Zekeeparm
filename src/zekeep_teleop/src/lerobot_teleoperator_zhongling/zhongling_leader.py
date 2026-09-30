#!/usr/bin/env python

# Copyright 2026 The HuggingFace Inc. team. All rights reserved.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0

import logging
import re
import threading
import time
from typing import Any

import serial

from lerobot.processor import RobotAction
from lerobot.teleoperators.teleoperator import Teleoperator
from lerobot.utils.decorators import check_if_already_connected, check_if_not_connected

from .config_zhongling_leader import ZhonglingLeaderConfig

logger = logging.getLogger(__name__)
PWM_PATTERN = re.compile(r"#(?P<id>\d{3})P(?P<pwm>\d{4})!")


class ZhonglingLeader(Teleoperator):
    """LeRobot teleoperator for a seven-axis Zhongling serial leader arm."""

    config_class = ZhonglingLeaderConfig
    name = "zhongling_leader"

    def __init__(self, config: ZhonglingLeaderConfig):
        super().__init__(config)
        self.config = config
        self._serial: serial.Serial | None = None
        self._last_action: list[float] | None = None
        self._last_action_time: float | None = None
        self._read_failures = [0] * len(config.servo_ids)
        self._startup_pwm_centers: list[int] | None = None
        self._latest_targets: list[float] | None = None
        self._background_error: Exception | None = None
        self._background_lock = threading.Lock()
        self._background_stop = threading.Event()
        self._background_thread: threading.Thread | None = None

    @property
    def action_features(self) -> dict[str, type]:
        return {f"{joint}.pos": float for joint in self.config.joint_names}

    @property
    def feedback_features(self) -> dict[str, type]:
        return {}

    @property
    def is_connected(self) -> bool:
        return self._serial is not None and self._serial.is_open

    @property
    def is_calibrated(self) -> bool:
        return True

    @check_if_already_connected
    def connect(self, calibrate: bool = True) -> None:
        del calibrate
        self._serial = serial.Serial(
            self.config.port,
            self.config.baudrate,
            timeout=self.config.serial_timeout_s,
        )
        try:
            self.configure()
            startup_pwms = self._read_startup_frame()
            if self.config.use_startup_pose_as_zero:
                self._set_startup_pwm_centers(startup_pwms)
            initial = [self._pwm_to_target(pwm, index) for index, pwm in enumerate(startup_pwms)]
        except Exception:
            self._serial.close()
            self._serial = None
            self._startup_pwm_centers = None
            raise

        self._last_action = [0.0] * len(initial) if self.config.start_from_zero else initial
        self._last_action_time = None
        self._latest_targets = initial
        self._read_failures = [0] * len(self.config.servo_ids)
        if self.config.use_background_read:
            self._start_background_reader()
        logger.info(f"{self} connected on {self.config.port} @ {self.config.baudrate}.")

    def _start_background_reader(self) -> None:
        self._background_stop.clear()
        self._background_error = None
        self._background_thread = threading.Thread(
            target=self._background_read_loop,
            name=f"{self.id}-prad-reader",
            daemon=True,
        )
        self._background_thread.start()

    def _background_read_loop(self) -> None:
        while not self._background_stop.is_set():
            try:
                targets = self._read_targets(require_all=False)
            except Exception as exc:
                if not self._background_stop.is_set():
                    with self._background_lock:
                        self._background_error = exc
                return
            with self._background_lock:
                self._latest_targets = targets
            if self.config.background_read_interval_s > 0:
                self._background_stop.wait(self.config.background_read_interval_s)

    def _get_latest_targets(self) -> list[float]:
        with self._background_lock:
            if self._background_error is not None:
                raise ConnectionError("Zhongling background reader stopped") from self._background_error
            if self._latest_targets is None:
                raise ConnectionError("Zhongling background reader has no valid frame")
            return self._latest_targets.copy()

    def calibrate(self) -> None:
        logger.info(
            "Zhongling midpoint calibration is stored in each servo; no additional "
            "LeRobot calibration file is required."
        )

    @check_if_not_connected
    def configure(self) -> None:
        if not self.config.release_torque_on_connect:
            return
        for servo_id in self.config.servo_ids:
            self._send_command(f"#{servo_id:03d}PULK!", expect_response=False)
            time.sleep(self.config.torque_release_wait_s)

    @staticmethod
    def parse_pwm(response: str, expected_servo_id: int | None = None) -> int | None:
        for match in PWM_PATTERN.finditer(response):
            if expected_servo_id is None or int(match.group("id")) == expected_servo_id:
                return int(match.group("pwm"))
        return None

    def _send_command(self, command: str, *, expect_response: bool = True) -> str:
        if self._serial is None:
            raise RuntimeError("Serial port is not open")
        self._serial.reset_input_buffer()
        payload = f"{command}{self.config.command_terminator}".encode("ascii")
        self._serial.write(payload)
        self._serial.flush()
        time.sleep(self.config.response_wait_s)
        if not expect_response:
            return ""
        response = self._serial.read_until(b"!") + self._serial.read_all()
        return response.decode("ascii", errors="ignore")

    def _read_pwm(self, servo_id: int) -> int | None:
        response = self._send_command(f"#{servo_id:03d}PRAD!")
        return self.parse_pwm(response, expected_servo_id=servo_id)

    def _read_startup_frame(self) -> list[int]:
        pwms: list[int] = []
        for index, servo_id in enumerate(self.config.servo_ids):
            pwm = self._read_pwm(servo_id)
            if pwm is None:
                raise ConnectionError(f"No valid PRAD response from Zhongling servo ID {servo_id:03d}")
            if servo_id in self.config.startup_zero_check_ids:
                configured_center = self.config.pwm_ranges[index][1]
                error = pwm - configured_center
                if abs(error) > self.config.startup_zero_tolerance_pwm:
                    raise ConnectionError(
                        f"Zhongling servo ID {servo_id:03d} is not at startup zero: "
                        f"PWM {pwm}, expected {configured_center} "
                        f"(tolerance ±{self.config.startup_zero_tolerance_pwm})"
                    )
            self._read_failures[index] = 0
            pwms.append(pwm)
        return pwms

    def _set_startup_pwm_centers(self, pwms: list[int]) -> None:
        for index, (servo_id, pwm) in enumerate(zip(self.config.servo_ids, pwms, strict=True)):
            pwm_min, _, pwm_max = self.config.pwm_ranges[index]
            minimum_margin = self.config.zero_deadband_pwm + 1
            if not pwm_min + minimum_margin <= pwm <= pwm_max - minimum_margin:
                raise ConnectionError(
                    f"Zhongling servo ID {servo_id:03d} startup PWM {pwm} is too close "
                    f"to configured endpoint [{pwm_min}, {pwm_max}] for a "
                    f"±{self.config.zero_deadband_pwm} zero deadband"
                )
        self._startup_pwm_centers = pwms.copy()
        captured = ", ".join(
            f"{servo_id:03d}={pwm}"
            for servo_id, pwm in zip(self.config.servo_ids, pwms, strict=True)
        )
        logger.info(f"{self} captured startup pose as session zero: {captured}")

    def _pwm_to_target(self, pwm: int, index: int) -> float:
        pwm_min, configured_center, pwm_max = self.config.pwm_ranges[index]
        pwm_center = (
            self._startup_pwm_centers[index]
            if self._startup_pwm_centers is not None
            else configured_center
        )
        pwm = min(pwm_max, max(pwm_min, pwm))
        offset = pwm - pwm_center
        if abs(offset) <= self.config.zero_deadband_pwm:
            return 0.0

        degrees_per_pwm = self.config.joint_degrees_per_pwm[index]
        if degrees_per_pwm is not None:
            effective_offset = abs(offset) - self.config.zero_deadband_pwm
            target = (
                effective_offset
                * degrees_per_pwm
                * (1 if offset > 0 else -1)
                * self.config.joint_directions[index]
            )
            target_min, target_max = self.config.joint_ranges_deg[index]
            return min(target_max, max(target_min, target))

        source_min, source_max = self.config.source_ranges_rad[index]
        if offset < 0:
            pwm_fraction = (-offset - self.config.zero_deadband_pwm) / (
                pwm_center - pwm_min - self.config.zero_deadband_pwm
            )
            source_value = source_min * pwm_fraction
        else:
            pwm_fraction = (offset - self.config.zero_deadband_pwm) / (
                pwm_max - pwm_center - self.config.zero_deadband_pwm
            )
            source_value = source_max * pwm_fraction

        source_value *= self.config.joint_directions[index]
        target_min, target_max = self.config.joint_ranges_deg[index]
        if source_value < 0:
            negative_fraction = min(1.0, source_value / source_min)
            return target_min * negative_fraction
        positive_fraction = min(1.0, source_value / source_max)
        return target_max * positive_fraction

    def _read_targets(self, require_all: bool) -> list[float]:
        targets: list[float] = []
        for index, servo_id in enumerate(self.config.servo_ids):
            pwm = self._read_pwm(servo_id)
            if pwm is None:
                self._read_failures[index] += 1
                if require_all or self._last_action is None:
                    raise ConnectionError(f"No valid PRAD response from Zhongling servo ID {servo_id:03d}")
                if self._read_failures[index] >= self.config.max_consecutive_read_failures:
                    raise ConnectionError(
                        f"Zhongling servo ID {servo_id:03d} failed "
                        f"{self._read_failures[index]} consecutive reads"
                    )
                if self._read_failures[index] == 1:
                    logger.warning(
                        f"Invalid PRAD response from Zhongling servo ID {servo_id:03d}; "
                        "holding its previous command"
                    )
                targets.append(self._last_action[index])
                continue
            self._read_failures[index] = 0
            targets.append(self._pwm_to_target(pwm, index))
        return targets

    @check_if_not_connected
    def get_action(self) -> RobotAction:
        action_time = time.perf_counter()
        targets = (
            self._get_latest_targets()
            if self.config.use_background_read
            else self._read_targets(require_all=False)
        )
        limit = self.config.max_step_deg
        if self.config.max_velocity_deg_s is not None:
            elapsed_s = (
                1 / 30
                if self._last_action_time is None
                else max(0.0, action_time - self._last_action_time)
            )
            velocity_limit = self.config.max_velocity_deg_s * elapsed_s
            limit = velocity_limit if limit is None else min(limit, velocity_limit)
        if self._last_action is not None and limit is not None:
            targets = [
                previous + min(limit, max(-limit, target - previous))
                for target, previous in zip(targets, self._last_action, strict=True)
            ]
        self._last_action = targets
        self._last_action_time = action_time
        return {
            f"{joint}.pos": target
            for joint, target in zip(self.config.joint_names, targets, strict=True)
        }

    def send_feedback(self, feedback: dict[str, Any]) -> None:
        del feedback
        raise NotImplementedError("Zhongling leader force feedback is not supported")

    @check_if_not_connected
    def disconnect(self) -> None:
        assert self._serial is not None
        self._background_stop.set()
        if self._background_thread is not None:
            self._background_thread.join()
            self._background_thread = None
        self._serial.close()
        self._serial = None
        self._startup_pwm_centers = None
        self._latest_targets = None
        self._last_action_time = None
        logger.info(f"{self} disconnected.")

