"""Perception helpers for fixed-size tabletop cubes."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Sequence

import cv2
import numpy as np


@dataclass(frozen=True)
class CubeGeometry:
    valid: bool
    strict: bool
    length_m: float
    width_m: float
    height_m: float
    point_count: int
    center_base: np.ndarray | None = None


@dataclass(frozen=True)
class GraspCenter:
    pixel: tuple[int, int]
    position_cam: np.ndarray
    local_depth_mm: float


def fuse_depth_frames(
    frames: Sequence[np.ndarray],
    *,
    minimum_votes: int = 3,
) -> np.ndarray:
    """Fuse registered depth frames with a zero-aware temporal median."""
    if not frames:
        raise ValueError("at least one depth frame is required")
    arrays = [np.asarray(frame) for frame in frames]
    shape = arrays[0].shape
    if any(array.shape != shape for array in arrays):
        raise ValueError("all depth frames must have matching shapes")
    if minimum_votes <= 0 or minimum_votes > len(arrays):
        raise ValueError("minimum_votes must be within the frame count")

    stack = np.stack(arrays).astype(np.float64)
    valid = np.isfinite(stack) & (stack > 0.0)
    values = np.where(valid, stack, np.nan)
    with np.errstate(invalid="ignore"):
        fused = np.nanmedian(values, axis=0)
    fused[np.sum(valid, axis=0) < int(minimum_votes)] = 0.0
    fused = np.nan_to_num(fused, nan=0.0, posinf=0.0, neginf=0.0)
    return np.rint(fused).astype(arrays[0].dtype)


def select_sharpest_color(frames: Sequence[np.ndarray]) -> np.ndarray:
    """Return the RGB frame with the largest Laplacian variance."""
    if not frames:
        raise ValueError("at least one color frame is required")
    arrays = [np.asarray(frame) for frame in frames]
    shape = arrays[0].shape
    if any(array.shape != shape for array in arrays):
        raise ValueError("all color frames must have matching shapes")
    scores = []
    for frame in arrays:
        gray = (
            cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            if frame.ndim == 3
            else frame
        )
        scores.append(float(cv2.Laplacian(gray, cv2.CV_64F).var()))
    return arrays[int(np.argmax(scores))].copy()


def clean_target_mask(
    mask: np.ndarray,
    depth_mm: np.ndarray,
    *,
    bbox_xyxy: tuple[int, int, int, int],
    depth_tolerance_mm: float = 40.0,
    minimum_component_area_px: int = 20,
) -> np.ndarray:
    """Keep one target body and remove disconnected/depth-outlier pixels."""

    binary = (np.asarray(mask) > 0).astype(np.uint8)
    depth = np.asarray(depth_mm)
    if binary.shape != depth.shape:
        raise ValueError("mask and depth must have matching shapes")
    if depth_tolerance_mm < 0.0:
        raise ValueError("depth_tolerance_mm must be nonnegative")

    kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (3, 3))
    binary = cv2.morphologyEx(binary, cv2.MORPH_CLOSE, kernel)
    count, labels, stats, centroids = cv2.connectedComponentsWithStats(
        binary,
        connectivity=8,
    )
    if count <= 1:
        return np.zeros_like(binary)

    x1, y1, x2, y2 = (float(value) for value in bbox_xyxy)
    center = np.array([(x1 + x2) * 0.5, (y1 + y2) * 0.5])
    diagonal = max(float(np.hypot(x2 - x1, y2 - y1)), 1.0)
    best_label = 0
    best_score = -np.inf
    for label in range(1, count):
        area = int(stats[label, cv2.CC_STAT_AREA])
        if area < int(minimum_component_area_px):
            continue
        distance = float(np.linalg.norm(centroids[label] - center))
        score = float(area) / (1.0 + distance / diagonal)
        if score > best_score:
            best_score = score
            best_label = label
    if best_label == 0:
        return np.zeros_like(binary)

    cleaned = labels == best_label
    component_depth = depth[cleaned]
    valid_depth = component_depth[
        np.isfinite(component_depth) & (component_depth > 0)
    ]
    if len(valid_depth):
        center_x1 = max(0, int(round(0.35 * x1 + 0.65 * center[0])))
        center_x2 = min(
            depth.shape[1],
            int(round(0.35 * x2 + 0.65 * center[0])) + 1,
        )
        center_y1 = max(0, int(round(0.35 * y1 + 0.65 * center[1])))
        center_y2 = min(
            depth.shape[0],
            int(round(0.35 * y2 + 0.65 * center[1])) + 1,
        )
        central_values = depth[center_y1:center_y2, center_x1:center_x2]
        central_component = cleaned[
            center_y1:center_y2,
            center_x1:center_x2,
        ]
        central_values = central_values[
            central_component
            & np.isfinite(central_values)
            & (central_values > 0)
        ]
        reference_mm = float(
            np.median(central_values if len(central_values) else valid_depth)
        )
        depth_ok = (
            (depth <= 0)
            | (
                np.isfinite(depth)
                & (
                    np.abs(depth.astype(np.float64) - reference_mm)
                    <= float(depth_tolerance_mm)
                )
            )
        )
        cleaned &= depth_ok
    return cleaned.astype(np.uint8)


def assess_cube_geometry(
    points_base: np.ndarray,
    *,
    table_z_m: float,
    relaxed_bounds_m: tuple[float, float] = (0.022, 0.038),
    strict_bounds_m: tuple[float, float] = (0.025, 0.035),
    enforce_size_bounds: bool = True,
) -> CubeGeometry:
    """Measure a target and place its physical grasp center at half height."""
    points = np.asarray(points_base, dtype=np.float64)
    points = points[
        np.all(np.isfinite(points), axis=1)
    ] if points.ndim == 2 and points.shape[1] == 3 else np.empty((0, 3))
    if len(points) < 3:
        return CubeGeometry(False, False, 0.0, 0.0, 0.0, len(points), None)

    rectangle = cv2.minAreaRect(points[:, :2].astype(np.float32))
    length_m, width_m = sorted(
        (float(rectangle[1][0]), float(rectangle[1][1])),
        reverse=True,
    )
    # Segmentation includes visible side faces. Estimate the top from the
    # upper surface band so those lower points do not bias cube height down.
    z_values = points[:, 2]
    top_surface_threshold_m = float(np.percentile(z_values, 80.0))
    top_surface_z = z_values[z_values >= top_surface_threshold_m]
    top_z_m = float(np.median(top_surface_z))
    height_m = float(top_z_m - float(table_z_m))
    center_base = np.array(
        [
            float(rectangle[0][0]),
            float(rectangle[0][1]),
            float(table_z_m) + 0.5 * height_m,
        ],
        dtype=np.float64,
    )
    def within(bounds: tuple[float, float]) -> bool:
        low, high = (float(value) for value in bounds)
        return (
            low <= length_m <= high
            and low <= width_m <= high
            and low <= height_m <= high
        )

    measurable = length_m > 0.0 and width_m > 0.0 and height_m > 0.0
    return CubeGeometry(
        valid=measurable and (
            within(relaxed_bounds_m) if enforce_size_bounds else True
        ),
        strict=within(strict_bounds_m),
        length_m=length_m,
        width_m=width_m,
        height_m=height_m,
        point_count=len(points),
        center_base=center_base,
    )


def cube_geometry_center_camera(
    geometry: CubeGeometry,
    T_cam2base: np.ndarray,
) -> np.ndarray:
    """Transform the measured 3D cube center from base to camera coordinates."""

    if geometry.center_base is None:
        raise ValueError("cube geometry has no 3D center")
    center_base = np.asarray(geometry.center_base, dtype=np.float64).reshape(-1)
    transform = np.asarray(T_cam2base, dtype=np.float64)
    if center_base.shape != (3,) or not np.all(np.isfinite(center_base)):
        raise ValueError("cube geometry center must contain three finite values")
    if transform.shape != (4, 4) or not np.all(np.isfinite(transform)):
        raise ValueError("T_cam2base must be a finite 4x4 transform")
    center_base_h = np.append(center_base, 1.0)
    try:
        center_camera_h = np.linalg.solve(transform, center_base_h)
    except np.linalg.LinAlgError as exc:
        raise ValueError("T_cam2base must be invertible") from exc
    if abs(float(center_camera_h[3])) < 1e-12:
        raise ValueError("cube center transformed to an invalid homogeneous point")
    center_camera = center_camera_h[:3] / center_camera_h[3]
    if not np.all(np.isfinite(center_camera)):
        raise ValueError("cube center transformed to nonfinite camera coordinates")
    return center_camera


def sample_grasp_pixels(
    mask: np.ndarray,
    depth_mm: np.ndarray,
    *,
    maximum_points: int = 5,
    erosion_px: int = 3,
) -> tuple[tuple[int, int], ...]:
    """Choose deterministic interior grasp centers from the target mask."""

    if maximum_points <= 0:
        raise ValueError("maximum_points must be positive")
    binary = (np.asarray(mask) > 0).astype(np.uint8)
    depth = np.asarray(depth_mm)
    if binary.shape != depth.shape:
        raise ValueError("mask and depth must have matching shapes")

    erosion = max(0, int(erosion_px))
    if erosion:
        size = 2 * erosion + 1
        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (size, size))
        safe = cv2.erode(binary, kernel, iterations=1)
    else:
        safe = binary
    safe = (safe > 0).astype(np.uint8)
    if not np.any(safe):
        safe = binary
    if not np.any(safe):
        return ()

    distance = cv2.distanceTransform(safe, cv2.DIST_L2, 5)
    ys, xs = np.nonzero(safe)
    centroid = np.array([float(np.mean(xs)), float(np.mean(ys))])
    nearest_index = int(
        np.argmin((xs - centroid[0]) ** 2 + (ys - centroid[1]) ** 2)
    )
    candidates: list[tuple[int, int]] = [
        (int(xs[nearest_index]), int(ys[nearest_index]))
    ]

    maximum_distance = float(np.max(distance))
    core = (safe > 0) & (distance >= 0.55 * maximum_distance)
    core_y, core_x = np.nonzero(core)
    if len(core_x):
        extrema = (
            int(np.argmin(core_x)),
            int(np.argmax(core_x)),
            int(np.argmin(core_y)),
            int(np.argmax(core_y)),
        )
        candidates.extend(
            (int(core_x[index]), int(core_y[index]))
            for index in extrema
        )

    unique: list[tuple[int, int]] = []
    for candidate in candidates:
        if candidate not in unique:
            unique.append(candidate)
        if len(unique) >= int(maximum_points):
            break
    return tuple(unique)


def build_grasp_centers(
    mask: np.ndarray,
    depth_mm: np.ndarray,
    intrinsic: np.ndarray,
    *,
    maximum_points: int = 5,
    erosion_px: int = 3,
    depth_window_radius_px: int = 3,
) -> tuple[GraspCenter, ...]:
    """Backproject safe interior pixels using robust local median depth."""

    binary = (np.asarray(mask) > 0).astype(np.uint8)
    depth = np.asarray(depth_mm)
    K = np.asarray(intrinsic, dtype=np.float64)
    if binary.shape != depth.shape:
        raise ValueError("mask and depth must have matching shapes")
    if K.shape != (3, 3) or not np.all(np.isfinite(K)):
        raise ValueError("intrinsic must be a finite 3x3 matrix")

    pixels = sample_grasp_pixels(
        binary,
        depth,
        maximum_points=maximum_points,
        erosion_px=erosion_px,
    )
    radius = max(0, int(depth_window_radius_px))
    height, width = depth.shape
    centers: list[GraspCenter] = []
    for u, v in pixels:
        x1, x2 = max(0, u - radius), min(width, u + radius + 1)
        y1, y2 = max(0, v - radius), min(height, v + radius + 1)
        roi_depth = depth[y1:y2, x1:x2]
        roi_mask = binary[y1:y2, x1:x2] > 0
        values = roi_depth[
            roi_mask & np.isfinite(roi_depth) & (roi_depth > 0)
        ]
        if not len(values):
            continue
        local_depth_mm = float(np.median(values))
        z_m = local_depth_mm / 1000.0
        x_m = (float(u) - float(K[0, 2])) * z_m / float(K[0, 0])
        y_m = (float(v) - float(K[1, 2])) * z_m / float(K[1, 1])
        centers.append(
            GraspCenter(
                pixel=(int(u), int(v)),
                position_cam=np.array([x_m, y_m, z_m], dtype=np.float64),
                local_depth_mm=local_depth_mm,
            )
        )
    return tuple(centers)
