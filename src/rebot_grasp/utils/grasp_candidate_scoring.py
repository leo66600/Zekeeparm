"""Depth-correction ranking for GraspNet candidates."""

from __future__ import annotations

import numpy as np


DEFAULT_TOP_K = 50
DEFAULT_WIDTH_CLEARANCE_M = 0.008
DEFAULT_WIDTH_SIGMA_M = 0.015
DEFAULT_FINGER_LENGTH_M = 0.060
DEFAULT_FINGER_THICKNESS_M = 0.010
DEFAULT_PALM_CLEARANCE_M = 0.004
DEFAULT_WEIGHTS = {
    "center": 0.20,
    "inner_occupancy": 0.15,
    "lateral_balance": 0.10,
    "width_fit": 0.15,
    "insertion": 0.15,
    "palm_clearance": 0.05,
    "network": 0.10,
    "ready": 0.10,
}


def rank_depth_corrections(
    *,
    translation: np.ndarray,
    rotation: np.ndarray,
    width: float,
    height: float,
    network_depth: float,
    target_points: np.ndarray,
    corrections_m: tuple[float, ...] | list[float],
    finger_length_m: float = DEFAULT_FINGER_LENGTH_M,
    finger_thickness_m: float = DEFAULT_FINGER_THICKNESS_M,
) -> tuple[float, ...]:
    """Rank small TCP corrections using target contact geometry.

    Corrections move the grasp reference along the candidate's local X axis.
    The network depth only defines the GraspNet finger window; it is never
    added to the TCP position.  A small magnitude tie-break keeps nominal
    depth first when contact quality is equivalent.
    """
    center = np.asarray(translation, dtype=np.float64)
    matrix = np.asarray(rotation, dtype=np.float64)
    points = np.asarray(target_points, dtype=np.float64).reshape(-1, 3)
    if (
        center.shape != (3,)
        or matrix.shape != (3, 3)
        or not np.all(np.isfinite(center))
        or not np.all(np.isfinite(matrix))
    ):
        raise ValueError("depth ranking pose must be finite and well-shaped")
    values = tuple(float(value) for value in corrections_m)
    if not values or any(not np.isfinite(value) for value in values):
        raise ValueError("depth corrections must be finite and nonempty")
    unique_values = tuple(dict.fromkeys(values))
    if len(points) == 0:
        return unique_values

    approach = matrix[:, 0]
    width = max(float(width), 1e-4)
    height = max(float(height), 1e-4)
    network_depth = max(float(network_depth), 1e-4)
    finger_length = max(float(finger_length_m), 1e-4)
    scored: list[tuple[float, int, float]] = []
    correction_scale = max(
        max(abs(value) for value in unique_values),
        1e-4,
    )
    for index, correction in enumerate(unique_values):
        local = (points - (center + approach * correction)) @ matrix
        height_window = np.abs(local[:, 2]) <= 0.5 * height
        opening_window = np.abs(local[:, 1]) <= 0.5 * width
        finger_window = (
            local[:, 0] >= network_depth - finger_length
        ) & (local[:, 0] <= network_depth)
        inner = height_window & opening_window & finger_window
        contact_count = int(np.count_nonzero(inner))
        left = int(np.count_nonzero(inner & (local[:, 1] < 0.0)))
        right = int(np.count_nonzero(inner & (local[:, 1] > 0.0)))
        total = left + right
        balance = float(2.0 * min(left, right) / max(total, 1))
        nominal_bias = float(np.exp(-abs(correction) / correction_scale))
        quality = (
            0.65 * min(contact_count / max(len(points), 1), 1.0)
            + 0.20 * balance
            + 0.15 * nominal_bias
        )
        scored.append((-quality, index, correction))
    scored.sort()
    return tuple(item[2] for item in scored)
