"""Fixed-table placement geometry used by the MoveIt grasp route."""

from __future__ import annotations

from dataclasses import dataclass
import math
from typing import Any

import numpy as np

from .transforms import pose6d_to_mat4


@dataclass(frozen=True)
class FixedPlacementSpec:
    """Validated placement target and the two vertical TCP waypoints."""

    x_m: float
    y_m: float
    table_z_m: float
    object_dimensions_m: tuple[float, float, float]
    tcp_rpy: tuple[float, float, float]
    release_clearance_m: float
    preplace_clearance_m: float
    object_padding_m: float

    @property
    def object_height_m(self) -> float:
        return float(self.object_dimensions_m[2])

    @property
    def release_z_m(self) -> float:
        # GraspNet/geometry candidates put the TCP at the object centre.
        return self.table_z_m + self.object_height_m / 2.0 + self.release_clearance_m

    @property
    def above_z_m(self) -> float:
        return self.release_z_m + self.preplace_clearance_m

    @property
    def release_pose(self) -> np.ndarray:
        return np.asarray(
            (self.x_m, self.y_m, self.release_z_m, *self.tcp_rpy),
            dtype=np.float64,
        )

    @property
    def above_pose(self) -> np.ndarray:
        return np.asarray(
            (self.x_m, self.y_m, self.above_z_m, *self.tcp_rpy),
            dtype=np.float64,
        )

    @property
    def release_transform(self) -> np.ndarray:
        return pose6d_to_mat4(*self.release_pose)

    @property
    def above_transform(self) -> np.ndarray:
        return pose6d_to_mat4(*self.above_pose)

    @property
    def padded_dimensions_m(self) -> tuple[float, float, float]:
        pad = self.object_padding_m
        return tuple(float(value + 2.0 * pad) for value in self.object_dimensions_m)


def build_fixed_placement_spec(
    place_cfg: dict[str, Any],
    *,
    table_z_m: float | None,
    object_dimensions_m: tuple[float, float, float] | list[float] | np.ndarray,
) -> FixedPlacementSpec:
    """Validate runtime dimensions and build fixed table placement waypoints."""
    if table_z_m is None or not math.isfinite(float(table_z_m)):
        raise ValueError("fixed placement requires a finite runtime table height")
    dimensions = tuple(float(value) for value in object_dimensions_m)
    if len(dimensions) != 3 or not all(math.isfinite(value) and value > 0.0 for value in dimensions):
        raise ValueError("object dimensions must contain three positive finite values")

    bounds = place_cfg.get("object_height_bounds_m", (0.020, 0.060))
    if len(bounds) != 2:
        raise ValueError("object_height_bounds_m must contain [min, max]")
    height_min, height_max = (float(value) for value in bounds)
    height = dimensions[2]
    if not math.isfinite(height_min) or not math.isfinite(height_max) or height_min <= 0.0 or height_max < height_min:
        raise ValueError("object height bounds are invalid")
    if height < height_min or height > height_max:
        raise ValueError(
            f"object height {height:.4f}m outside placement range "
            f"[{height_min:.4f}, {height_max:.4f}]m"
        )

    x_m = float(place_cfg.get("x_m", 0.30))
    y_m = float(place_cfg.get("y_m", -0.30))
    tcp_rpy = tuple(float(value) for value in place_cfg.get("tcp_rpy", (0.0, 1.5708, 0.0)))
    if len(tcp_rpy) != 3 or not all(math.isfinite(value) for value in tcp_rpy):
        raise ValueError("placement tcp_rpy must contain three finite values")
    clearance = float(place_cfg.get("table_clearance_m", 0.005))
    preplace = float(place_cfg.get("preplace_clearance_m", 0.08))
    padding = float(place_cfg.get("object_padding_m", 0.005))
    if not all(math.isfinite(value) for value in (x_m, y_m, clearance, preplace, padding)):
        raise ValueError("placement values must be finite")
    if clearance < 0.0 or preplace <= 0.0 or padding < 0.0:
        raise ValueError("placement clearances/padding are invalid")
    return FixedPlacementSpec(
        x_m=x_m,
        y_m=y_m,
        table_z_m=float(table_z_m),
        object_dimensions_m=dimensions,
        tcp_rpy=tcp_rpy,
        release_clearance_m=clearance,
        preplace_clearance_m=preplace,
        object_padding_m=padding,
    )
