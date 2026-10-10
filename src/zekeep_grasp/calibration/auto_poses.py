"""Validated Cartesian pose sets for automatic hand-eye collection."""
from __future__ import annotations

from collections.abc import Iterator, Sequence
from dataclasses import dataclass
import hashlib
import json
from pathlib import Path

import numpy as np
import yaml


@dataclass(frozen=True)
class AutoCalibrationPosePlan(Sequence[tuple[float, ...]]):
    """Pose sequence with the coordinate and validated-start contract."""

    poses: tuple[tuple[float, ...], ...]
    base_frame: str
    reference_frame: str
    position_unit: str
    angle_unit: str
    euler_convention: str
    validated_start_joint_positions: tuple[float, ...]
    start_tolerance_rad: float
    pose_sha256: str

    def __len__(self) -> int:
        return len(self.poses)

    def __getitem__(self, index):
        return self.poses[index]

    def __iter__(self) -> Iterator[tuple[float, ...]]:
        return iter(self.poses)

    def require_validated_start(self, current_joint_positions: Sequence[float]) -> None:
        """Refuse automatic traversal from an unvalidated initial branch."""
        current = np.asarray(current_joint_positions, dtype=np.float64).reshape(-1)
        validated = np.asarray(
            self.validated_start_joint_positions,
            dtype=np.float64,
        )
        if current.size < 6 or not np.all(np.isfinite(current[:6])):
            raise ValueError("current joint state must contain six finite values")
        delta = np.abs(current[:6] - validated)
        if float(np.max(delta)) > self.start_tolerance_rad:
            raise ValueError(
                "automatic calibration requires the validated start configuration; "
                f"max joint error is {float(np.max(delta)):.4f}rad, "
                f"limit is {self.start_tolerance_rad:.4f}rad"
            )


def load_auto_calibration_poses(
    path: str | Path,
    *,
    expected_reference_frame: str,
) -> AutoCalibrationPosePlan:
    """Load poses only when their declared end frame matches the robot model."""
    pose_path = Path(path)
    data = yaml.safe_load(pose_path.read_text(encoding="utf-8")) or {}
    declared_frame = str(data.get("reference_frame", "")).strip()
    expected_frame = str(expected_reference_frame).strip()
    if not declared_frame or declared_frame != expected_frame:
        raise ValueError(
            "automatic calibration pose reference frame mismatch: "
            f"file={declared_frame!r}, robot={expected_frame!r}"
        )

    required_metadata = {
        "base_frame": "base_link",
        "position_unit": "meter",
        "angle_unit": "radian",
        "euler_convention": "intrinsic_zyx_rpy",
    }
    for key, expected in required_metadata.items():
        actual = str(data.get(key, "")).strip()
        if actual != expected:
            raise ValueError(
                f"automatic calibration pose {key} must be {expected!r}, got {actual!r}"
            )

    validated_start = np.asarray(
        data.get("validated_start_joint_positions") or (),
        dtype=np.float64,
    ).reshape(-1)
    if validated_start.size != 6 or not np.all(np.isfinite(validated_start)):
        raise ValueError(
            "validated_start_joint_positions must contain six finite radians"
        )
    start_tolerance_rad = float(data.get("start_tolerance_rad", float("nan")))
    if not np.isfinite(start_tolerance_rad) or not 0.0 < start_tolerance_rad <= 0.1:
        raise ValueError("start_tolerance_rad must be finite and in (0, 0.1]")

    poses = []
    for index, raw_pose in enumerate(data.get("poses") or (), start=1):
        values = np.asarray(raw_pose, dtype=np.float64).reshape(-1)
        if values.size != 6 or not np.all(np.isfinite(values)):
            raise ValueError(
                f"automatic calibration pose {index} must contain six finite values"
            )
        poses.append(tuple(float(value) for value in values))
    if len(poses) < 5:
        raise ValueError("automatic calibration requires at least five poses")
    canonical_poses = json.dumps(
        poses,
        separators=(",", ":"),
        ensure_ascii=True,
    ).encode("ascii")
    actual_pose_sha256 = hashlib.sha256(canonical_poses).hexdigest()
    declared_pose_sha256 = str(data.get("pose_sha256", "")).strip().lower()
    if declared_pose_sha256 != actual_pose_sha256:
        raise ValueError(
            "automatic calibration pose content hash mismatch: "
            f"file={declared_pose_sha256!r}, actual={actual_pose_sha256!r}"
        )
    return AutoCalibrationPosePlan(
        poses=tuple(poses),
        base_frame=required_metadata["base_frame"],
        reference_frame=declared_frame,
        position_unit=required_metadata["position_unit"],
        angle_unit=required_metadata["angle_unit"],
        euler_convention=required_metadata["euler_convention"],
        validated_start_joint_positions=tuple(float(value) for value in validated_start),
        start_tolerance_rad=start_tolerance_rad,
        pose_sha256=actual_pose_sha256,
    )
