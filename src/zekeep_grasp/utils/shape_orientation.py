"""Estimate a collision-safe top-down gripper yaw from target depth points."""

from __future__ import annotations

from dataclasses import dataclass
import math
from typing import Sequence

import numpy as np


@dataclass(frozen=True)
class ShapeOrientationEstimate:
    reliable: bool
    major_axis_xy: np.ndarray
    opening_axis_xy: np.ndarray
    anisotropy: float
    point_count: int
    major_span_m: float
    minor_span_m: float


def _canonical_axis(axis_xy: np.ndarray) -> np.ndarray:
    axis = np.asarray(axis_xy, dtype=np.float64).reshape(-1)
    if axis.size != 2 or not np.all(np.isfinite(axis)):
        raise ValueError("horizontal axis must contain two finite values")
    norm = float(np.linalg.norm(axis))
    if norm < 1e-12:
        raise ValueError("horizontal axis must be nonzero")
    axis = axis / norm
    dominant = int(np.argmax(np.abs(axis)))
    if axis[dominant] < 0.0:
        axis = -axis
    return axis


def estimate_shape_opening_axis(
    points_base: np.ndarray,
    *,
    minimum_points: int = 80,
    minimum_anisotropy: float = 1.35,
    minimum_major_span_m: float = 0.012,
    minimum_minor_span_m: float = 0.004,
) -> ShapeOrientationEstimate:
    """Use horizontal PCA; jaws open across the object's short dimension."""

    points = np.asarray(points_base, dtype=np.float64)
    if points.ndim != 2 or points.shape[1] != 3:
        raise ValueError("points_base must have shape (N, 3)")
    points = points[np.all(np.isfinite(points), axis=1)]
    if int(minimum_points) < 3:
        raise ValueError("minimum_points must be at least three")
    anisotropy_threshold = float(minimum_anisotropy)
    if not math.isfinite(anisotropy_threshold) or anisotropy_threshold < 1.0:
        raise ValueError("minimum_anisotropy must be finite and at least one")
    span_threshold = float(minimum_major_span_m)
    if not math.isfinite(span_threshold) or span_threshold < 0.0:
        raise ValueError("minimum_major_span_m must be finite and nonnegative")
    minor_span_threshold = float(minimum_minor_span_m)
    if not math.isfinite(minor_span_threshold) or minor_span_threshold < 0.0:
        raise ValueError("minimum_minor_span_m must be finite and nonnegative")

    fallback_major = np.array([1.0, 0.0], dtype=np.float64)
    fallback_opening = np.array([0.0, 1.0], dtype=np.float64)
    if len(points) < 3:
        return ShapeOrientationEstimate(
            reliable=False,
            major_axis_xy=fallback_major,
            opening_axis_xy=fallback_opening,
            anisotropy=1.0,
            point_count=len(points),
            major_span_m=0.0,
            minor_span_m=0.0,
        )

    xy = points[:, :2]
    centered = xy - np.median(xy, axis=0)
    covariance = centered.T @ centered / max(len(centered) - 1, 1)
    eigenvalues, eigenvectors = np.linalg.eigh(covariance)
    order = np.argsort(eigenvalues)[::-1]
    major_axis = _canonical_axis(eigenvectors[:, order[0]])
    minor_axis = _canonical_axis(eigenvectors[:, order[1]])
    major_projection = centered @ major_axis
    minor_projection = centered @ minor_axis
    major_span = float(np.percentile(major_projection, 95) - np.percentile(major_projection, 5))
    minor_span = float(np.percentile(minor_projection, 95) - np.percentile(minor_projection, 5))
    anisotropy = (
        major_span / minor_span
        if minor_span > 1e-9
        else float("inf")
    )
    opening_axis = _canonical_axis(
        np.array([-major_axis[1], major_axis[0]], dtype=np.float64)
    )
    return ShapeOrientationEstimate(
        reliable=(
            len(points) >= int(minimum_points)
            and major_span >= span_threshold
            and minor_span >= minor_span_threshold
            and anisotropy >= anisotropy_threshold
        ),
        major_axis_xy=major_axis,
        opening_axis_xy=opening_axis,
        anisotropy=anisotropy,
        point_count=len(points),
        major_span_m=major_span,
        minor_span_m=minor_span,
    )


def masked_depth_points_base(
    depth_mm: np.ndarray,
    mask: np.ndarray,
    K: np.ndarray,
    T_cam2base: np.ndarray,
    *,
    min_depth_m: float,
    max_depth_m: float,
) -> np.ndarray:
    """Backproject target-mask depth pixels and transform them to base frame."""

    depth = np.asarray(depth_mm)
    target_mask = np.asarray(mask)
    intrinsic = np.asarray(K, dtype=np.float64)
    transform = np.asarray(T_cam2base, dtype=np.float64)
    if depth.ndim != 2:
        raise ValueError("depth_mm must be a 2D image")
    if target_mask.shape != depth.shape:
        raise ValueError("mask must match depth image shape")
    if intrinsic.shape != (3, 3) or not np.all(np.isfinite(intrinsic)):
        raise ValueError("K must have shape (3, 3) with finite values")
    if transform.shape != (4, 4) or not np.all(np.isfinite(transform)):
        raise ValueError("T_cam2base must have shape (4, 4) with finite values")
    minimum = float(min_depth_m)
    maximum = float(max_depth_m)
    if not 0.0 <= minimum < maximum or not math.isfinite(maximum):
        raise ValueError("depth limits must be finite and increasing")
    fx, fy = float(intrinsic[0, 0]), float(intrinsic[1, 1])
    cx, cy = float(intrinsic[0, 2]), float(intrinsic[1, 2])
    if fx <= 0.0 or fy <= 0.0:
        raise ValueError("camera focal lengths must be positive")

    z = depth.astype(np.float64) / 1000.0
    valid = (
        (target_mask > 0)
        & np.isfinite(z)
        & (z >= minimum)
        & (z <= maximum)
    )
    v, u = np.nonzero(valid)
    if len(u) == 0:
        return np.empty((0, 3), dtype=np.float64)
    z_values = z[v, u]
    points_cam = np.column_stack(
        (
            (u.astype(np.float64) - cx) * z_values / fx,
            (v.astype(np.float64) - cy) * z_values / fy,
            z_values,
        )
    )
    homogeneous = np.column_stack(
        (points_cam, np.ones(len(points_cam), dtype=np.float64))
    )
    return (homogeneous @ transform.T)[:, :3]


def top_down_rotations_for_opening_axis(
    opening_axis_xy: Sequence[float],
) -> tuple[np.ndarray, np.ndarray]:
    """Return the two 180-degree-equivalent vertical gripper rotations."""

    opening_xy = _canonical_axis(np.asarray(opening_axis_xy, dtype=np.float64))
    approach = np.array([0.0, 0.0, -1.0], dtype=np.float64)
    opening = np.array(
        [opening_xy[0], opening_xy[1], 0.0],
        dtype=np.float64,
    )
    completion = np.cross(approach, opening)
    completion /= np.linalg.norm(completion)
    first = np.column_stack((approach, opening, completion))
    second = np.column_stack((approach, -opening, -completion))
    return first, second


def shape_aligned_grasp_rotations(
    opening_axis_xy: Sequence[float],
    *,
    lean_axis_xy: Sequence[float] | None = None,
    oblique_tilt_deg: float = 60.0,
    lean_offset_deg: float = -45.0,
    include_vertical_fallback: bool = True,
) -> tuple[np.ndarray, ...]:
    """Return shape-aligned oblique grasps plus optional vertical fallbacks.

    The TCP X axis is the approach direction and TCP Y is the jaw opening
    direction.  Both lean directions and both 180-degree-equivalent jaw poses
    are emitted so IK can select the reachable wrist configuration.  Defaults
    match the live MoveIt reachability probe near the worktable's outer edge.
    """

    tilt = float(oblique_tilt_deg)
    if not math.isfinite(tilt) or not 0.0 < tilt < 90.0:
        raise ValueError("oblique_tilt_deg must be finite and between 0 and 90")
    lean_offset = float(lean_offset_deg)
    if not math.isfinite(lean_offset):
        raise ValueError("lean_offset_deg must be finite")

    opening_xy = _canonical_axis(np.asarray(opening_axis_xy, dtype=np.float64))
    opening = np.array([opening_xy[0], opening_xy[1], 0.0], dtype=np.float64)
    downward = np.array([0.0, 0.0, -1.0], dtype=np.float64)
    if lean_axis_xy is None:
        horizontal = np.cross(downward, opening)
        horizontal /= np.linalg.norm(horizontal)
    else:
        lean_xy = _canonical_axis(
            np.asarray(lean_axis_xy, dtype=np.float64)
        )
        horizontal = np.array([lean_xy[0], lean_xy[1], 0.0])
    offset = math.radians(lean_offset)
    horizontal = np.array(
        [
            math.cos(offset) * horizontal[0]
            - math.sin(offset) * horizontal[1],
            math.sin(offset) * horizontal[0]
            + math.cos(offset) * horizontal[1],
            0.0,
        ],
        dtype=np.float64,
    )
    angle = math.radians(tilt)

    rotations: list[np.ndarray] = []
    for lean_sign in (1.0, -1.0):
        approach = (
            math.cos(angle) * downward
            + lean_sign * math.sin(angle) * horizontal
        )
        projected_opening = opening - approach * float(
            np.dot(opening, approach)
        )
        projected_opening /= np.linalg.norm(projected_opening)
        completion = np.cross(approach, projected_opening)
        completion /= np.linalg.norm(completion)
        rotations.append(
            np.column_stack((approach, projected_opening, completion))
        )
        rotations.append(
            np.column_stack((approach, -projected_opening, -completion))
        )

    if include_vertical_fallback:
        rotations.extend(top_down_rotations_for_opening_axis(opening_xy))
    return tuple(rotations)
