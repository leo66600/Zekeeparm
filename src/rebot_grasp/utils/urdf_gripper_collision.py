"""Collision checks using the real gripper meshes declared in the robot URDF."""

from __future__ import annotations

from dataclasses import dataclass, replace
import math
from pathlib import Path
from typing import Iterable, Sequence

import cv2
import numpy as np
import open3d as o3d
import pinocchio as pin


_GRIPPER_GEOMETRIES = (
    "gripper_base_0",
    "left_link_0",
    "right_link_0",
)


@dataclass(frozen=True)
class CollisionResult:
    safe: bool
    reason: str
    min_table_clearance_m: float
    min_obstacle_distance_m: float
    sample_index: int = 0
    sample_count: int = 1
    segment_index: int = -1
    segment_fraction: float = 0.0


@dataclass(frozen=True)
class TableHeightEstimate:
    z_m: float
    inlier_count: int
    total_count: int


def transform_points(
    points: np.ndarray,
    transform: np.ndarray,
) -> np.ndarray:
    points = np.asarray(points, dtype=np.float64)
    transform = np.asarray(transform, dtype=np.float64)
    if points.ndim != 2 or points.shape[1] != 3:
        raise ValueError("points must have shape (N, 3)")
    if transform.shape != (4, 4):
        raise ValueError("transform must have shape (4, 4)")
    return points @ transform[:3, :3].T + transform[:3, 3]


def filter_locked_table_points(
    points_base: np.ndarray,
    *,
    table_z_m: float,
    tolerance_m: float,
) -> np.ndarray:
    """Remove the fixed table surface already modeled in MoveIt."""

    points = np.asarray(points_base, dtype=np.float64)
    if points.size == 0:
        return np.empty((0, 3), dtype=np.float64)
    if points.ndim != 2 or points.shape[1] != 3:
        raise ValueError("points_base must have shape (N, 3)")
    table_z = float(table_z_m)
    tolerance = float(tolerance_m)
    if not math.isfinite(table_z):
        raise ValueError("table_z_m must be finite")
    if not math.isfinite(tolerance) or tolerance < 0.0:
        raise ValueError("tolerance_m must be finite and nonnegative")
    finite = np.all(np.isfinite(points), axis=1)
    belongs_to_table = finite & (
        np.abs(points[:, 2] - table_z) <= tolerance
    )
    return points[finite & ~belongs_to_table].copy()


def voxel_downsample_points(
    points: np.ndarray,
    *,
    voxel_size_m: float,
) -> np.ndarray:
    points = np.asarray(points, dtype=np.float64)
    if points.size == 0:
        return np.empty((0, 3), dtype=np.float64)
    if points.ndim != 2 or points.shape[1] != 3:
        raise ValueError("points must have shape (N, 3)")
    cloud = o3d.geometry.PointCloud()
    cloud.points = o3d.utility.Vector3dVector(points)
    reduced = cloud.voxel_down_sample(float(voxel_size_m))
    return np.asarray(reduced.points, dtype=np.float64)


def backproject_obstacle_cloud(
    depth_mm: np.ndarray,
    K: np.ndarray,
    *,
    exclude_mask: np.ndarray,
    min_depth_m: float,
    max_depth_m: float,
) -> np.ndarray:
    depth = np.asarray(depth_mm)
    K = np.asarray(K, dtype=np.float64)
    exclude = np.asarray(exclude_mask)
    if depth.ndim != 2:
        raise ValueError("depth_mm must be a 2D image")
    if exclude.shape != depth.shape:
        raise ValueError("exclude_mask must match depth image shape")
    if K.shape != (3, 3):
        raise ValueError("K must have shape (3, 3)")

    z = depth.astype(np.float64) / 1000.0
    valid = (
        np.isfinite(z)
        & (z > float(min_depth_m))
        & (z < float(max_depth_m))
        & (exclude <= 0)
    )
    v, u = np.nonzero(valid)
    if len(u) == 0:
        return np.empty((0, 3), dtype=np.float64)
    z_valid = z[v, u]
    x = (u.astype(np.float64) - K[0, 2]) * z_valid / K[0, 0]
    y = (v.astype(np.float64) - K[1, 2]) * z_valid / K[1, 1]
    return np.column_stack([x, y, z_valid])


def build_target_depth_exclusion_mask(
    depth_mm: np.ndarray,
    *,
    bbox_xyxy: tuple[int, int, int, int],
    min_depth_m: float,
    max_depth_m: float,
    depth_tolerance_m: float,
    seed_ratio: float = 0.5,
) -> np.ndarray:
    """Select only the target's depth-connected component inside a YOLO box."""

    depth = np.asarray(depth_mm)
    if depth.ndim != 2:
        raise ValueError("depth_mm must be a 2D image")
    h, w = depth.shape
    x1, y1, x2, y2 = [int(value) for value in bbox_xyxy]
    x1 = int(np.clip(x1, 0, max(0, w - 1)))
    x2 = int(np.clip(x2, 0, max(0, w - 1)))
    y1 = int(np.clip(y1, 0, max(0, h - 1)))
    y2 = int(np.clip(y2, 0, max(0, h - 1)))
    output = np.zeros((h, w), dtype=np.uint8)
    if x2 < x1 or y2 < y1:
        return output

    z = depth.astype(np.float64) / 1000.0
    valid = (
        np.isfinite(z)
        & (z > float(min_depth_m))
        & (z < float(max_depth_m))
    )
    roi = np.zeros((h, w), dtype=bool)
    roi[y1 : y2 + 1, x1 : x2 + 1] = True

    ratio = float(np.clip(seed_ratio, 0.1, 1.0))
    cx = 0.5 * (x1 + x2)
    cy = 0.5 * (y1 + y2)
    seed_width = max(1, int(math.ceil((x2 - x1 + 1) * ratio)))
    seed_height = max(1, int(math.ceil((y2 - y1 + 1) * ratio)))
    sx1 = int(np.clip(round(cx - 0.5 * (seed_width - 1)), x1, x2))
    sx2 = int(np.clip(sx1 + seed_width - 1, x1, x2))
    sy1 = int(np.clip(round(cy - 0.5 * (seed_height - 1)), y1, y2))
    sy2 = int(np.clip(sy1 + seed_height - 1, y1, y2))
    seed = np.zeros((h, w), dtype=bool)
    seed[sy1 : sy2 + 1, sx1 : sx2 + 1] = True
    seed_valid = seed & valid
    if not np.any(seed_valid):
        return output

    target_depth = float(np.median(z[seed_valid]))
    candidate = (
        roi
        & valid
        & (np.abs(z - target_depth) <= float(depth_tolerance_m))
    )
    component_count, labels = cv2.connectedComponents(
        candidate.astype(np.uint8),
        connectivity=8,
    )
    if component_count <= 1:
        return output
    seed_labels = labels[seed_valid & candidate]
    seed_labels = seed_labels[seed_labels > 0]
    if seed_labels.size == 0:
        return output
    scores = np.bincount(seed_labels, minlength=component_count)
    target_label = int(np.argmax(scores[1:]) + 1)
    output[labels == target_label] = 1
    return output


def is_top_down_rotation(
    tcp_rotation_base: np.ndarray,
    *,
    max_tilt_deg: float,
) -> bool:
    rotation = np.asarray(tcp_rotation_base, dtype=np.float64)
    if rotation.shape != (3, 3) or not np.all(np.isfinite(rotation)):
        return False
    approach = rotation[:, 0]
    norm = float(np.linalg.norm(approach))
    if norm < 1e-9:
        return False
    downward_alignment = float(np.dot(approach / norm, [0.0, 0.0, -1.0]))
    return downward_alignment >= math.cos(math.radians(float(max_tilt_deg)))


def estimate_horizontal_table_z(
    points_base: np.ndarray,
    *,
    bin_size_m: float,
    inlier_tolerance_m: float,
    min_inliers: int,
) -> TableHeightEstimate:
    points = np.asarray(points_base, dtype=np.float64)
    if points.ndim != 2 or points.shape[1] != 3:
        raise ValueError("points_base must have shape (N, 3)")
    points = points[np.all(np.isfinite(points), axis=1)]
    if len(points) < min_inliers:
        raise RuntimeError(
            f"table plane unavailable: only {len(points)} finite points"
        )
    bin_size_m = float(bin_size_m)
    if bin_size_m <= 0.0:
        raise ValueError("bin_size_m must be positive")

    z = points[:, 2]
    # Anchor bins to the base-frame origin, not the per-frame minimum Z.
    # Otherwise one distant depth outlier shifts every bin boundary and can
    # make the dominant surface jump between neighboring scene planes.
    bins = np.floor(z / bin_size_m).astype(np.int64)
    unique_bins, counts = np.unique(bins, return_counts=True)
    dominant_bin = int(unique_bins[int(np.argmax(counts))])
    center = (dominant_bin + 0.5) * bin_size_m
    inlier_mask = np.abs(z - center) <= float(inlier_tolerance_m)
    inlier_z = z[inlier_mask]
    if len(inlier_z) < int(min_inliers):
        raise RuntimeError(
            "table plane unavailable: "
            f"dominant horizontal band has {len(inlier_z)} points"
        )
    return TableHeightEstimate(
        z_m=float(np.median(inlier_z)),
        inlier_count=int(len(inlier_z)),
        total_count=int(len(points)),
    )


def estimate_horizontal_table_from_depth(
    depth_mm: np.ndarray,
    K: np.ndarray,
    T_cam2base: np.ndarray,
    *,
    exclude_mask: np.ndarray,
    min_depth_m: float,
    max_depth_m: float,
    bin_size_m: float,
    inlier_tolerance_m: float,
    min_inliers: int,
) -> TableHeightEstimate:
    """Estimate the table through the same masked RGB-D projection pipeline."""

    points_cam = backproject_obstacle_cloud(
        depth_mm,
        K,
        exclude_mask=exclude_mask,
        min_depth_m=min_depth_m,
        max_depth_m=max_depth_m,
    )
    return estimate_horizontal_table_z(
        transform_points(points_cam, T_cam2base),
        bin_size_m=bin_size_m,
        inlier_tolerance_m=inlier_tolerance_m,
        min_inliers=min_inliers,
    )


class UrdfGripperCollisionModel:
    """Real gripper collision mesh expressed relative to the configured TCP."""

    def __init__(
        self,
        urdf_path: str | Path,
        *,
        open_width_m: float,
        end_frame: str = "end_link",
    ) -> None:
        self.urdf_path = Path(urdf_path).expanduser().resolve()
        if not self.urdf_path.is_file():
            raise FileNotFoundError(f"URDF not found: {self.urdf_path}")
        package_dirs = [str(self.urdf_path.parent.parent)]
        for parent in self.urdf_path.parents:
            if parent.name == "zekeep_bringup":
                package_dirs.append(str(parent.parent))
                break
        model, collision_model, _ = pin.buildModelsFromUrdf(
            str(self.urdf_path),
            package_dirs=package_dirs,
        )
        q = pin.neutral(model)
        half_opening = float(np.clip(0.5 * open_width_m, 0.0, 0.035))
        for joint_name in ("gripper_joint", "right_joint"):
            joint_id = model.getJointId(joint_name)
            if joint_id <= 0:
                raise ValueError(f"URDF missing {joint_name}")
            q[model.joints[joint_id].idx_q] = half_opening

        data = model.createData()
        geometry_data = pin.GeometryData(collision_model)
        pin.forwardKinematics(model, data, q)
        pin.updateFramePlacements(model, data)
        pin.updateGeometryPlacements(
            model,
            data,
            collision_model,
            geometry_data,
            q,
        )

        end_frame_id = model.getFrameId(end_frame)
        if end_frame_id >= len(model.frames):
            raise ValueError(f"URDF missing end frame {end_frame!r}")
        base_from_tcp = data.oMf[end_frame_id]
        tcp_from_base = base_from_tcp.inverse()

        vertices: list[np.ndarray] = []
        triangles: list[np.ndarray] = []
        geometry_names: list[str] = []
        vertex_offset = 0
        for geometry_name in _GRIPPER_GEOMETRIES:
            geometry_id = collision_model.getGeometryId(geometry_name)
            if geometry_id >= len(collision_model.geometryObjects):
                raise ValueError(f"URDF missing collision geometry {geometry_name}")
            geometry = collision_model.geometryObjects[geometry_id]
            mesh_path = Path(str(geometry.meshPath))
            mesh = o3d.io.read_triangle_mesh(str(mesh_path))
            mesh_vertices = np.asarray(mesh.vertices, dtype=np.float64)
            mesh_triangles = np.asarray(mesh.triangles, dtype=np.int32)
            if len(mesh_vertices) == 0 or len(mesh_triangles) == 0:
                raise RuntimeError(f"Failed to load collision mesh: {mesh_path}")

            scale = np.asarray(geometry.meshScale, dtype=np.float64).reshape(1, 3)
            mesh_vertices = mesh_vertices * scale
            tcp_from_geometry = tcp_from_base * geometry_data.oMg[geometry_id]
            mesh_vertices = (
                mesh_vertices @ tcp_from_geometry.rotation.T
                + tcp_from_geometry.translation
            )
            vertices.append(mesh_vertices)
            triangles.append(mesh_triangles + vertex_offset)
            vertex_offset += len(mesh_vertices)
            geometry_names.append(geometry_name)

        self.geometry_names = tuple(geometry_names)
        self.vertices_tcp = np.vstack(vertices).astype(np.float64)
        self.triangles = np.vstack(triangles).astype(np.int32)
        self._bounds_min = np.min(self.vertices_tcp, axis=0)
        self._bounds_max = np.max(self.vertices_tcp, axis=0)

        legacy_mesh = o3d.geometry.TriangleMesh()
        legacy_mesh.vertices = o3d.utility.Vector3dVector(self.vertices_tcp)
        legacy_mesh.triangles = o3d.utility.Vector3iVector(self.triangles)
        tensor_mesh = o3d.t.geometry.TriangleMesh.from_legacy(legacy_mesh)
        self._scene = o3d.t.geometry.RaycastingScene()
        self._scene.add_triangles(tensor_mesh)

    def filter_self_observations(
        self,
        points_base: np.ndarray,
        T_tcp2base: np.ndarray,
        *,
        distance_m: float,
    ) -> np.ndarray:
        """Remove depth points produced by the gripper at capture time."""

        points = np.asarray(points_base, dtype=np.float64)
        if points.size == 0:
            return np.empty((0, 3), dtype=np.float64)
        if points.ndim != 2 or points.shape[1] != 3:
            raise ValueError("points_base must have shape (N, 3)")
        transform = np.asarray(T_tcp2base, dtype=np.float64)
        if transform.shape != (4, 4):
            raise ValueError("T_tcp2base must have shape (4, 4)")
        distance = float(distance_m)
        if not math.isfinite(distance) or distance < 0.0:
            raise ValueError("distance_m must be finite and nonnegative")

        finite = np.all(np.isfinite(points), axis=1)
        finite_points = points[finite]
        rotation = transform[:3, :3]
        translation = transform[:3, 3]
        points_tcp = (finite_points - translation) @ rotation
        near_bounds = np.all(
            (points_tcp >= self._bounds_min - distance)
            & (points_tcp <= self._bounds_max + distance),
            axis=1,
        )
        remove = np.zeros(len(finite_points), dtype=bool)
        if np.any(near_bounds):
            query = o3d.core.Tensor(
                points_tcp[near_bounds].astype(np.float32)
            )
            mesh_distances = self._scene.compute_distance(query).numpy()
            remove[near_bounds] = mesh_distances <= distance
        return finite_points[~remove].copy()

    def check_pose(
        self,
        T_tcp2base: np.ndarray,
        *,
        obstacle_points_base: np.ndarray,
        table_z_m: float,
        table_clearance_m: float,
        obstacle_clearance_m: float,
    ) -> CollisionResult:
        transform = np.asarray(T_tcp2base, dtype=np.float64)
        if transform.shape != (4, 4):
            raise ValueError("T_tcp2base must have shape (4, 4)")
        rotation = transform[:3, :3]
        translation = transform[:3, 3]
        vertices_base = self.vertices_tcp @ rotation.T + translation
        min_table_clearance = float(np.min(vertices_base[:, 2]) - table_z_m)
        if min_table_clearance < float(table_clearance_m):
            return CollisionResult(
                safe=False,
                reason="table",
                min_table_clearance_m=min_table_clearance,
                min_obstacle_distance_m=math.inf,
            )

        obstacle_points = np.asarray(obstacle_points_base, dtype=np.float64)
        if obstacle_points.size == 0:
            return CollisionResult(
                safe=True,
                reason="",
                min_table_clearance_m=min_table_clearance,
                min_obstacle_distance_m=math.inf,
            )
        if obstacle_points.ndim != 2 or obstacle_points.shape[1] != 3:
            raise ValueError("obstacle_points_base must have shape (N, 3)")
        obstacle_points = obstacle_points[
            np.all(np.isfinite(obstacle_points), axis=1)
        ]
        points_tcp = (obstacle_points - translation) @ rotation
        clearance = float(obstacle_clearance_m)
        broadphase = np.all(
            (points_tcp >= self._bounds_min - clearance)
            & (points_tcp <= self._bounds_max + clearance),
            axis=1,
        )
        points_tcp = points_tcp[broadphase]
        if len(points_tcp) == 0:
            return CollisionResult(
                safe=True,
                reason="",
                min_table_clearance_m=min_table_clearance,
                min_obstacle_distance_m=math.inf,
            )

        query = o3d.core.Tensor(points_tcp.astype(np.float32))
        distances = self._scene.compute_distance(query).numpy()
        min_distance = float(np.min(distances))
        if min_distance <= clearance:
            return CollisionResult(
                safe=False,
                reason="environment",
                min_table_clearance_m=min_table_clearance,
                min_obstacle_distance_m=min_distance,
            )
        return CollisionResult(
            safe=True,
            reason="",
            min_table_clearance_m=min_table_clearance,
            min_obstacle_distance_m=min_distance,
        )

    def check_swept_path(
        self,
        waypoints_tcp2base: Sequence[np.ndarray],
        *,
        obstacle_points_base: np.ndarray,
        table_z_m: float,
        table_clearance_m: float,
        obstacle_clearance_m: float,
        linear_step_m: float,
        angular_step_rad: float,
    ) -> CollisionResult:
        samples = list(
            _interpolate_waypoint_samples(
                waypoints_tcp2base,
                linear_step_m=linear_step_m,
                angular_step_rad=angular_step_rad,
            )
        )
        if not samples:
            raise ValueError("at least one waypoint is required")
        last_safe = CollisionResult(True, "", math.inf, math.inf)
        for index, (transform, segment_index, segment_fraction) in enumerate(
            samples
        ):
            result = self.check_pose(
                transform,
                obstacle_points_base=obstacle_points_base,
                table_z_m=table_z_m,
                table_clearance_m=table_clearance_m,
                obstacle_clearance_m=obstacle_clearance_m,
            )
            if not result.safe:
                return replace(
                    result,
                    sample_index=index,
                    sample_count=len(samples),
                    segment_index=segment_index,
                    segment_fraction=segment_fraction,
                )
            last_safe = result
        _, segment_index, segment_fraction = samples[-1]
        return replace(
            last_safe,
            sample_index=len(samples) - 1,
            sample_count=len(samples),
            segment_index=segment_index,
            segment_fraction=segment_fraction,
        )


def _interpolate_waypoint_samples(
    waypoints: Sequence[np.ndarray],
    *,
    linear_step_m: float,
    angular_step_rad: float,
) -> Iterable[tuple[np.ndarray, int, float]]:
    if not waypoints:
        return
    linear_step_m = max(float(linear_step_m), 1e-6)
    angular_step_rad = max(float(angular_step_rad), 1e-6)
    first = np.asarray(waypoints[0], dtype=np.float64)
    yield first, 0 if len(waypoints) > 1 else -1, 0.0
    for segment_index, (start_array, end_array) in enumerate(
        zip(waypoints, waypoints[1:])
    ):
        start_array = np.asarray(start_array, dtype=np.float64)
        end_array = np.asarray(end_array, dtype=np.float64)
        start = pin.SE3(start_array[:3, :3], start_array[:3, 3])
        end = pin.SE3(end_array[:3, :3], end_array[:3, 3])
        linear_distance = float(
            np.linalg.norm(end.translation - start.translation)
        )
        angular_distance = float(
            np.linalg.norm(pin.log3(start.rotation.T @ end.rotation))
        )
        intervals = max(
            1,
            int(math.ceil(linear_distance / linear_step_m)),
            int(math.ceil(angular_distance / angular_step_rad)),
        )
        for step in range(1, intervals + 1):
            alpha = step / intervals
            yield (
                pin.SE3.Interpolate(start, end, alpha).homogeneous,
                segment_index,
                alpha,
            )
