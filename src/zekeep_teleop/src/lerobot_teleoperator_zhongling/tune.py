#!/usr/bin/env python

"""Inspect one Zhongling axis and calculate an independent output range."""

import argparse
import time
from dataclasses import dataclass, replace
from pathlib import Path

import serial
import yaml

from . import ZhonglingLeader, ZhonglingLeaderConfig


@dataclass(frozen=True)
class AxisRecommendation:
    servo_id: int
    joint_name: str
    observed_pwm: int
    current_target_deg: float
    desired_target_deg: float
    current_range_deg: tuple[float, float]
    recommended_range_deg: tuple[float, float]


def load_leader_config(config_path: Path) -> ZhonglingLeaderConfig:
    with config_path.open() as config_file:
        document = yaml.safe_load(config_file)
    if not isinstance(document, dict) or not isinstance(document.get("teleop"), dict):
        raise ValueError(f"{config_path} does not contain a teleop mapping")

    values = document["teleop"].copy()
    teleop_type = values.pop("type", None)
    if teleop_type != "zhongling_leader":
        raise ValueError(f"expected teleop.type=zhongling_leader, got {teleop_type!r}")
    values["calibration_dir"] = Path("/tmp/lerobot-zhongling-tune")
    return ZhonglingLeaderConfig(**values)


def calculate_recommendation(
    config: ZhonglingLeaderConfig,
    servo_id: int,
    observed_pwm: int,
    desired_target_deg: float,
) -> AxisRecommendation:
    try:
        index = config.servo_ids.index(servo_id)
    except ValueError as exc:
        configured = ", ".join(f"{item:03d}" for item in config.servo_ids)
        raise ValueError(
            f"servo ID {servo_id:03d} is not configured; available IDs: {configured}"
        ) from exc

    mapper = ZhonglingLeader(replace(config, use_background_read=False))
    current_target = mapper._pwm_to_target(observed_pwm, index)
    if abs(current_target) < 1e-9:
        raise ValueError("the observed PWM maps to zero; use a point outside the zero deadband")
    if current_target * desired_target_deg <= 0:
        raise ValueError(
            f"desired angle {desired_target_deg:g} must have the same sign as the "
            f"current mapped angle {current_target:.3f}"
        )

    scale = desired_target_deg / current_target
    lower, upper = config.joint_ranges_deg[index]
    recommended = (lower * scale, upper) if current_target < 0 else (lower, upper * scale)
    return AxisRecommendation(
        servo_id=servo_id,
        joint_name=config.joint_names[index],
        observed_pwm=observed_pwm,
        current_target_deg=current_target,
        desired_target_deg=desired_target_deg,
        current_range_deg=(lower, upper),
        recommended_range_deg=recommended,
    )


def query_pwm(port: serial.Serial, config: ZhonglingLeaderConfig, servo_id: int) -> int | None:
    port.reset_input_buffer()
    port.write(f"#{servo_id:03d}PRAD!{config.command_terminator}".encode("ascii"))
    port.flush()
    time.sleep(config.response_wait_s)
    response = port.read_until(b"!") + port.read_all()
    return ZhonglingLeader.parse_pwm(response.decode("ascii", errors="ignore"), servo_id)


def monitor_axis(config: ZhonglingLeaderConfig, servo_id: int, interval_s: float) -> None:
    index = config.servo_ids.index(servo_id)
    mapper = ZhonglingLeader(replace(config, use_background_read=False))
    pwm_min, _, pwm_max = config.pwm_ranges[index]
    with serial.Serial(config.port, config.baudrate, timeout=config.serial_timeout_s) as port:
        print(
            f"Read-only monitoring ID {servo_id:03d} ({config.joint_names[index]}) "
            f"on {config.port}. Press Ctrl+C to stop."
        )
        while True:
            pwm = query_pwm(port, config, servo_id)
            if pwm is None:
                line = f"ID {servo_id:03d} | PWM ---- | NO DATA"
            else:
                target = mapper._pwm_to_target(pwm, index)
                saturation = ""
                if pwm <= pwm_min:
                    saturation = " | LOW LIMIT"
                elif pwm >= pwm_max:
                    saturation = " | HIGH LIMIT"
                line = f"ID {servo_id:03d} | PWM {pwm:4d} | target {target:8.3f} deg{saturation}"
            print(f"\r{line:<100}", end="", flush=True)
            time.sleep(interval_s)


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Read one Zhongling servo or calculate its output-range calibration."
    )
    parser.add_argument(
        "--config-path",
        type=Path,
        default=Path("configs/zhongling_b601_teleop.yaml"),
    )
    parser.add_argument("--servo-id", type=int, required=True)
    parser.add_argument("--observed-pwm", type=int)
    parser.add_argument("--desired-deg", type=float)
    parser.add_argument("--monitor", action="store_true")
    parser.add_argument("--interval", type=float, default=0.1)
    return parser


def main() -> None:
    parser = build_parser()
    args = parser.parse_args()
    if not 0 <= args.servo_id <= 999:
        parser.error("--servo-id must be between 0 and 999")
    if (args.observed_pwm is None) != (args.desired_deg is None):
        parser.error("--observed-pwm and --desired-deg must be provided together")
    if not args.monitor and args.observed_pwm is None:
        parser.error("select --monitor or provide --observed-pwm and --desired-deg")
    if args.interval < 0:
        parser.error("--interval cannot be negative")

    try:
        config = load_leader_config(args.config_path)
        if args.observed_pwm is not None:
            result = calculate_recommendation(
                config, args.servo_id, args.observed_pwm, args.desired_deg
            )
            print(f"Servo ID:         {result.servo_id:03d}")
            print(f"Joint:            {result.joint_name}")
            print(f"Observed PWM:      {result.observed_pwm}")
            print(f"Current target:    {result.current_target_deg:.3f} deg")
            print(f"Desired target:    {result.desired_target_deg:.3f} deg")
            print(f"Current range:     {list(result.current_range_deg)}")
            print(f"Recommended range: {list(result.recommended_range_deg)}")
        if args.monitor:
            monitor_axis(config, args.servo_id, args.interval)
    except KeyboardInterrupt:
        print("\nStopped.")
    except (OSError, ValueError, serial.SerialException) as exc:
        parser.error(str(exc))


if __name__ == "__main__":
    main()

