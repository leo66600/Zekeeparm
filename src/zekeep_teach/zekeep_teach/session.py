"""Pure data model and persistence for hand-guided teaching sessions."""

from __future__ import annotations

from dataclasses import dataclass, field
from datetime import datetime
import json
import math
import os
from pathlib import Path
import tempfile
from typing import Any, Sequence

import numpy as np

from .constants import ARM_JOINT_NAMES

SCHEMA_VERSION = 1


def _validate_joint_positions(values: Sequence[float]) -> np.ndarray:
    positions = np.asarray(values, dtype=np.float64).reshape(-1)
    if positions.size != len(ARM_JOINT_NAMES) or not np.all(np.isfinite(positions)):
        raise ValueError("teaching sample must contain six finite joint positions")
    return positions.copy()


def _validate_capture_timestamp(value: float) -> float:
    captured = float(value)
    if not math.isfinite(captured) or captured < 0.0:
        raise ValueError("capture timestamp must be finite and nonnegative")
    return captured


@dataclass(frozen=True)
class TeachingSample:
    """One timestamped six-joint sample captured during teaching."""
    captured_s: float
    positions: np.ndarray

    @classmethod
    def create(
        cls,
        positions: Sequence[float],
        *,
        captured_s: float,
    ) -> "TeachingSample":
        """Validate and construct a sample from raw joint positions."""
        return cls(
            _validate_capture_timestamp(captured_s),
            _validate_joint_positions(positions),
        )

    def to_dict(self) -> dict[str, Any]:
        return {
            "captured_s": self.captured_s,
            "positions": [float(value) for value in self.positions],
        }


@dataclass(frozen=True)
class RetimedWaypoint:
    """A replay waypoint with joint positions, velocities, and timing."""
    time_from_start_s: float
    positions: np.ndarray
    velocities: np.ndarray


@dataclass
class TeachingSession:
    """Validated point and continuous-path data for one teaching session."""
    created_at: str = field(default_factory=lambda: datetime.now().astimezone().isoformat())
    points: list[TeachingSample] = field(default_factory=list)
    path: list[TeachingSample] = field(default_factory=list)
    joint_names: tuple[str, ...] = ARM_JOINT_NAMES
    schema_version: int = SCHEMA_VERSION

    def __post_init__(self) -> None:
        if self.schema_version != SCHEMA_VERSION:
            raise ValueError(f"unsupported teaching schema version: {self.schema_version}")
        self.joint_names = tuple(self.joint_names)
        if self.joint_names != ARM_JOINT_NAMES:
            raise ValueError("teaching session joint names do not match the arm")
        if not isinstance(self.created_at, str) or not self.created_at.strip():
            raise ValueError("teaching session created_at must be a nonempty string")

    def add_point(
        self,
        positions: Sequence[float],
        *,
        captured_s: float,
        minimum_distance_rad: float,
    ) -> bool:
        """Append a discrete point unless it is within the duplicate threshold."""
        sample = TeachingSample.create(positions, captured_s=captured_s)
        threshold = float(minimum_distance_rad)
        if not math.isfinite(threshold) or threshold < 0.0:
            raise ValueError("minimum point distance must be finite and nonnegative")
        if self.points:
            distance = float(
                np.max(np.abs(sample.positions - self.points[-1].positions))
            )
            if distance < threshold:
                return False
        self.points.append(sample)
        return True

    def add_path_sample(
        self,
        positions: Sequence[float],
        *,
        captured_s: float,
    ) -> None:
        """Append a continuous sample with a strictly increasing timestamp."""
        sample = TeachingSample.create(positions, captured_s=captured_s)
        if self.path and sample.captured_s <= self.path[-1].captured_s:
            raise ValueError("continuous path timestamps must be strictly increasing")
        self.path.append(sample)

    def to_dict(self) -> dict[str, Any]:
        return {
            "schema_version": self.schema_version,
            "joint_names": list(self.joint_names),
            "created_at": self.created_at,
            "points": [sample.to_dict() for sample in self.points],
            "path": [sample.to_dict() for sample in self.path],
        }


def _samples_from_data(values: Any, *, path_samples: bool) -> list[TeachingSample]:
    if not isinstance(values, list):
        raise ValueError("teaching samples must be a list")
    samples: list[TeachingSample] = []
    for value in values:
        if not isinstance(value, dict):
            raise ValueError("teaching sample must be an object")
        try:
            sample = TeachingSample.create(
                value["positions"],
                captured_s=value["captured_s"],
            )
        except (KeyError, TypeError) as exc:
            raise ValueError("teaching sample is missing required values") from exc
        if path_samples and samples and sample.captured_s <= samples[-1].captured_s:
            raise ValueError("continuous path timestamps must be strictly increasing")
        samples.append(sample)
    return samples


def load_teaching_session(path: str | Path) -> TeachingSession:
    """Load and validate a teaching session from a JSON file."""
    source = Path(path)
    try:
        data = json.loads(source.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError) as exc:
        raise ValueError(f"failed to load teaching session: {source}") from exc
    if not isinstance(data, dict):
        raise ValueError("teaching session root must be an object")
    try:
        schema_version = data["schema_version"]
        if isinstance(schema_version, bool) or not isinstance(schema_version, int):
            raise ValueError("teaching schema version must be an integer")
        session = TeachingSession(
            schema_version=schema_version,
            joint_names=tuple(data["joint_names"]),
            created_at=data["created_at"],
        )
        session.points = _samples_from_data(data["points"], path_samples=False)
        session.path = _samples_from_data(data["path"], path_samples=True)
    except (KeyError, TypeError, ValueError) as exc:
        if isinstance(exc, ValueError):
            raise
        raise ValueError("teaching session is missing required values") from exc
    return session


def save_teaching_session(session: TeachingSession, path: str | Path) -> Path:
    """Atomically serialize a teaching session and return its output path."""
    destination = Path(path)
    destination.parent.mkdir(parents=True, exist_ok=True)
    payload = json.dumps(
        session.to_dict(),
        allow_nan=False,
        indent=2,
        sort_keys=True,
    )
    temporary_path: Path | None = None
    try:
        with tempfile.NamedTemporaryFile(
            mode="w",
            encoding="utf-8",
            dir=destination.parent,
            prefix=f".{destination.name}.",
            suffix=".tmp",
            delete=False,
        ) as temporary:
            temporary.write(payload)
            temporary.write("\n")
            temporary.flush()
            os.fsync(temporary.fileno())
            temporary_path = Path(temporary.name)
        os.replace(temporary_path, destination)
    finally:
        if temporary_path is not None and temporary_path.exists():
            temporary_path.unlink()
    return destination


def simplify_path(
    samples: Sequence[TeachingSample],
    *,
    minimum_distance_rad: float,
) -> list[TeachingSample]:
    """Remove adjacent path samples below the configured joint-distance threshold."""
    threshold = float(minimum_distance_rad)
    if not math.isfinite(threshold) or threshold < 0.0:
        raise ValueError("minimum path distance must be finite and nonnegative")
    if not samples:
        return []
    simplified = [samples[0]]
    for sample in samples[1:]:
        distance = float(
            np.max(np.abs(sample.positions - simplified[-1].positions))
        )
        if distance >= threshold:
            simplified.append(sample)
    return simplified


def retime_path(
    samples: Sequence[TeachingSample],
    *,
    max_velocity_rad_s: float,
    minimum_segment_s: float,
) -> list[RetimedWaypoint]:
    """Create a bounded-velocity Hermite replay path from captured samples."""
    maximum_velocity = float(max_velocity_rad_s)
    minimum_duration = float(minimum_segment_s)
    if not math.isfinite(maximum_velocity) or maximum_velocity <= 0.0:
        raise ValueError("maximum replay velocity must be finite and positive")
    if not math.isfinite(minimum_duration) or minimum_duration <= 0.0:
        raise ValueError("minimum segment duration must be finite and positive")
    if len(samples) < 2:
        raise ValueError("continuous replay requires at least two path samples")

    positions = [_validate_joint_positions(sample.positions) for sample in samples]
    times = [0.0]
    for start, target in zip(positions, positions[1:]):
        displacement = float(np.max(np.abs(target - start)))
        duration = max(minimum_duration, displacement / maximum_velocity)
        times.append(times[-1] + duration)

    velocities = [np.zeros(len(ARM_JOINT_NAMES), dtype=np.float64)]
    for index in range(1, len(positions) - 1):
        span_s = times[index + 1] - times[index - 1]
        velocity = (positions[index + 1] - positions[index - 1]) / span_s
        velocities.append(
            np.clip(velocity, -maximum_velocity, maximum_velocity)
        )
    velocities.append(np.zeros(len(ARM_JOINT_NAMES), dtype=np.float64))

    peak_velocity = _peak_hermite_velocity(positions, velocities, times)
    if peak_velocity > maximum_velocity:
        scale = peak_velocity / maximum_velocity
        times = [timestamp * scale for timestamp in times]
        velocities = [velocity / scale for velocity in velocities]

    return [
        RetimedWaypoint(
            time_from_start_s=float(timestamp),
            positions=position.copy(),
            velocities=velocity.copy(),
        )
        for timestamp, position, velocity in zip(times, positions, velocities)
    ]


def _peak_hermite_velocity(
    positions: Sequence[np.ndarray],
    velocities: Sequence[np.ndarray],
    times: Sequence[float],
) -> float:
    peak = 0.0
    for index in range(len(positions) - 1):
        duration = float(times[index + 1] - times[index])
        start = positions[index]
        target = positions[index + 1]
        start_velocity = velocities[index]
        target_velocity = velocities[index + 1]
        coefficient_a = (
            6.0 * (start - target) / duration
            + 3.0 * start_velocity
            + 3.0 * target_velocity
        )
        coefficient_b = (
            6.0 * (target - start) / duration
            - 4.0 * start_velocity
            - 2.0 * target_velocity
        )
        coefficient_c = start_velocity
        candidates = [
            np.abs(coefficient_c),
            np.abs(coefficient_a + coefficient_b + coefficient_c),
        ]
        nonzero = np.abs(coefficient_a) > 1e-15
        critical = np.zeros_like(coefficient_a)
        critical[nonzero] = (
            -coefficient_b[nonzero] / (2.0 * coefficient_a[nonzero])
        )
        interior = nonzero & (critical > 0.0) & (critical < 1.0)
        if np.any(interior):
            derivative = (
                coefficient_a * critical**2
                + coefficient_b * critical
                + coefficient_c
            )
            candidates.append(np.abs(derivative[interior]))
        peak = max(peak, *(float(np.max(candidate)) for candidate in candidates))
    return peak
