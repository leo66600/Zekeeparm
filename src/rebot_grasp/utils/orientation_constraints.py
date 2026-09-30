"""Validated grasp-orientation policies and frame-independent filtering."""

from __future__ import annotations

from dataclasses import dataclass
import math
from typing import Iterable

import numpy as np


_MODES = frozenset(("top_down", "any"))


@dataclass(frozen=True)
class OrientationConstraint:
    mode: str
    max_tilt_deg: float


@dataclass(frozen=True)
class OrientationFilterResult:
    accepted_indices: tuple[int, ...]
    tilt_degrees: tuple[float, ...]


@dataclass(frozen=True)
class OrientationPolicy:
    default: OrientationConstraint
    by_class: dict[str, OrientationConstraint]

    def for_class(self, class_name: str | None) -> OrientationConstraint:
        normalized = normalize_class_name(class_name)
        return self.by_class.get(normalized, self.default)


def normalize_class_name(value: str | None) -> str:
    return "".join(character for character in str(value or "").lower() if character.isalnum())


def constrain_rotation(
    rotation: np.ndarray,
    constraint: OrientationConstraint,
) -> np.ndarray:
    """Apply an orientation policy while preserving useful in-plane yaw."""

    matrix = np.asarray(rotation, dtype=np.float64)
    if matrix.shape != (3, 3):
        raise ValueError(f"rotation must have shape (3, 3), got {matrix.shape}")
    if not np.all(np.isfinite(matrix)):
        raise ValueError("rotation contains non-finite values")
    if constraint.mode == "any":
        return matrix.copy()
    if constraint.mode != "top_down":
        raise ValueError(f"unsupported orientation mode: {constraint.mode}")

    approach = np.array([0.0, 0.0, -1.0], dtype=np.float64)
    opening = None
    for preferred in (
        matrix[:, 1],
        matrix[:, 2],
        np.array([0.0, 1.0, 0.0]),
        np.array([1.0, 0.0, 0.0]),
    ):
        horizontal = preferred - float(np.dot(preferred, approach)) * approach
        norm = float(np.linalg.norm(horizontal))
        if norm >= 1e-9:
            opening = horizontal / norm
            break
    if opening is None:  # Defensive: fixed horizontal fallbacks make this unreachable.
        raise ValueError("cannot construct a horizontal gripper opening axis")

    completion = np.cross(approach, opening)
    completion /= np.linalg.norm(completion)
    opening = np.cross(completion, approach)
    return np.column_stack((approach, opening, completion))


def _parse_constraint(
    value: object,
    *,
    label: str,
    fallback_tilt_deg: float | None = None,
) -> OrientationConstraint:
    if not isinstance(value, dict):
        raise ValueError(f"{label} must be a mapping")
    mode = str(value.get("mode", "")).strip().lower()
    if mode not in _MODES:
        raise ValueError(f"{label}.mode must be one of {sorted(_MODES)}")
    raw_tilt = value.get("max_tilt_deg", fallback_tilt_deg)
    if raw_tilt is None:
        raise ValueError(f"{label}.max_tilt_deg is required")
    max_tilt_deg = float(raw_tilt)
    if not math.isfinite(max_tilt_deg) or not 0.0 <= max_tilt_deg <= 90.0:
        raise ValueError(f"{label}.max_tilt_deg must be between 0 and 90")
    return OrientationConstraint(mode=mode, max_tilt_deg=max_tilt_deg)


def load_orientation_constraints(value: object) -> OrientationPolicy:
    if not isinstance(value, dict):
        raise ValueError("orientation_constraints must be a mapping")
    default = _parse_constraint(
        value.get("default"),
        label="orientation_constraints.default",
    )
    raw_overrides = value.get("by_class", {})
    if not isinstance(raw_overrides, dict):
        raise ValueError("orientation_constraints.by_class must be a mapping")
    overrides: dict[str, OrientationConstraint] = {}
    for class_name, constraint in raw_overrides.items():
        normalized = normalize_class_name(str(class_name))
        if not normalized:
            raise ValueError("orientation_constraints.by_class contains an empty class")
        if normalized in overrides:
            raise ValueError(
                "orientation_constraints.by_class contains duplicate normalized "
                f"class {normalized!r}"
            )
        overrides[normalized] = _parse_constraint(
            constraint,
            label=f"orientation_constraints.by_class.{class_name}",
            fallback_tilt_deg=default.max_tilt_deg,
        )
    return OrientationPolicy(default=default, by_class=overrides)


def _tilt_degrees(rotation: np.ndarray) -> float:
    matrix = np.asarray(rotation, dtype=np.float64)
    if matrix.shape != (3, 3) or not np.all(np.isfinite(matrix)):
        return math.inf
    approach = matrix[:, 0]
    norm = float(np.linalg.norm(approach))
    if norm < 1e-9:
        return math.inf
    alignment = float(np.dot(approach / norm, [0.0, 0.0, -1.0]))
    return math.degrees(math.acos(float(np.clip(alignment, -1.0, 1.0))))


def filter_rotations(
    rotations: Iterable[np.ndarray],
    constraint: OrientationConstraint,
) -> OrientationFilterResult:
    tilts = tuple(_tilt_degrees(rotation) for rotation in rotations)
    if constraint.mode == "any":
        accepted = tuple(range(len(tilts)))
    elif constraint.mode == "top_down":
        accepted = tuple(
            index
            for index, tilt in enumerate(tilts)
            if tilt <= constraint.max_tilt_deg + 1e-9
        )
    else:
        raise ValueError(f"unsupported orientation mode: {constraint.mode}")
    return OrientationFilterResult(
        accepted_indices=accepted,
        tilt_degrees=tilts,
    )
