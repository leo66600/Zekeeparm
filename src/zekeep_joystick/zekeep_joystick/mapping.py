from __future__ import annotations

import math


def apply_deadzone(value: float, deadzone: float) -> float:
    """Remove stick drift while preserving full-scale output."""
    if not math.isfinite(value):
        return 0.0
    value = max(-1.0, min(1.0, value))
    if abs(value) <= deadzone:
        return 0.0
    return (value - deadzone if value > 0.0 else value + deadzone) / (1.0 - deadzone)


def joint_velocities(
    axes: list[float],
    axis_indices: list[int],
    axis_signs: list[float],
    max_speeds: list[float],
    deadzone: float,
) -> list[float]:
    return [
        apply_deadzone(float(axes[index]), deadzone) * sign * speed
        for index, sign, speed in zip(axis_indices, axis_signs, max_speeds)
    ]


def integrate_positions(
    positions: list[float],
    velocities: list[float],
    lower_limits: list[float],
    upper_limits: list[float],
    dt: float,
) -> tuple[list[float], list[float]]:
    targets, accepted_velocities = [], []
    for position, velocity, lower, upper in zip(
        positions, velocities, lower_limits, upper_limits
    ):
        target = max(lower, min(upper, position + velocity * dt))
        targets.append(target)
        accepted_velocities.append(velocity if lower < target < upper else 0.0)
    return targets, accepted_velocities


def integrate_cartesian_pose(
    pose: list[float], velocities: list[float], dt: float
) -> list[float]:
    """Integrate XYZ and base-frame RPY velocities, wrapping angles."""
    result = [value + velocity * dt for value, velocity in zip(pose, velocities)]
    result[3:] = [math.atan2(math.sin(value), math.cos(value)) for value in result[3:]]
    return result
