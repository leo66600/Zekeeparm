"""Package-wide constants for the six-axis arm."""

ARM_JOINT_NAMES: tuple[str, ...] = tuple(
    f"joint{index}" for index in range(1, 7)
)
