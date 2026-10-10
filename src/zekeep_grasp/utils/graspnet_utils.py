"""GraspNet helpers that sit above camera frames and YOLO detections."""

from __future__ import annotations

import os
import sys
from pathlib import Path
import time
from dataclasses import dataclass
from typing import Any, Optional

import cv2
import numpy as np
import open3d as o3d
import torch

os.environ.setdefault("MPLCONFIGDIR", "/tmp/matplotlib")
os.environ.setdefault("QT_QPA_FONTDIR", "/usr/share/fonts/truetype")

PROJECT_ROOT = Path(__file__).resolve().parents[1]
WORKSPACE_ROOT = PROJECT_ROOT.parents[1]
GRASPNET_ROOT = WORKSPACE_ROOT / "third_party" / "graspnet-baseline"
DEFAULT_NUM_VIEW = 300
DEFAULT_VOXEL_SIZE = 0.01
DEFAULT_WARMUP_FRAMES = 20
DISPLAY_FLIP_X = np.diag([1.0, -1.0, -1.0, 1.0]).astype(np.float64)


def prepare_graspnet_imports(graspnet_root: Path = GRASPNET_ROOT) -> None:
    for subdir in ("models", "dataset", "utils", "pointnet2", "graspnetAPI"):
        path = str(graspnet_root / subdir)
        if path not in sys.path:
            sys.path.insert(0, path)
    root = str(graspnet_root)
    if root not in sys.path:
        sys.path.insert(0, root)


prepare_graspnet_imports()

try:
    from .yolo_utils import YoloDetection
except ImportError:
    from yolo_utils import YoloDetection

from collision_detector import ModelFreeCollisionDetector  # noqa: E402
from data_utils import CameraInfo, create_point_cloud_from_depth_image  # noqa: E402
from graspnet import GraspNet, pred_decode  # noqa: E402
from graspnetAPI import Grasp, GraspGroup  # noqa: E402

try:
    from .orientation_constraints import OrientationConstraint, constrain_rotation
    from .transforms import (
        graspnet_rotation_to_rebot_tcp_rotation,
        mat4_to_pose6d,
        pose6d_to_mat4,
        transform_grasp_pose_to_base_with_retreat,
    )
except ImportError:
    from orientation_constraints import OrientationConstraint, constrain_rotation
    from transforms import (
        graspnet_rotation_to_rebot_tcp_rotation,
        mat4_to_pose6d,
        pose6d_to_mat4,
        transform_grasp_pose_to_base_with_retreat,
    )


@dataclass
class GraspNetFrameResult:
    grasps: GraspGroup
    pre_bbox_grasps: GraspGroup
    bbox_grasps: GraspGroup
    best: Optional[Grasp]
    status: str
    target_status: str
    detections: list[YoloDetection]
    selected_target: Optional[YoloDetection]
    o3d_cloud: o3d.geometry.PointCloud
    raw_cloud: np.ndarray


class Open3DGraspWindow:
    def __init__(self, title: str, top_k: int) -> None:
        self._top_k = top_k
        self._vis = o3d.visualization.Visualizer()
        if not self._vis.create_window(title, width=1280, height=720):
            self._vis.destroy_window()
            raise RuntimeError("Open3D visualizer window could not be created")
        self._geometries = []
        self._initialized = False

    def update(self, cloud: o3d.geometry.PointCloud, grasps: GraspGroup) -> None:
        for geom in self._geometries:
            self._vis.remove_geometry(geom, reset_bounding_box=False)
        self._geometries = []

        cloud_vis = o3d.geometry.PointCloud(cloud)
        cloud_vis.transform(DISPLAY_FLIP_X)
        geometries = [cloud_vis]

        if len(grasps) > 0:
            grasps_vis = GraspGroup(grasps.grasp_group_array.copy())
            try:
                grasps_vis = grasps_vis.nms()
            except Exception as exc:
                print(f"Grasp NMS skipped: {exc}")
            grasps_vis.sort_by_score()
            grasps_vis = grasps_vis[: self._top_k]
            grasps_vis.transform(DISPLAY_FLIP_X)
            geometries.extend(grasps_vis.to_open3d_geometry_list())

        for geom in geometries:
            self._vis.add_geometry(geom, reset_bounding_box=not self._initialized)
        self._geometries = geometries
        self._initialized = True
        self.poll()

    def poll(self) -> bool:
        alive = self._vis.poll_events()
        self._vis.update_renderer()
        return alive

    def close(self) -> None:
        self._vis.destroy_window()


def copy_grasp_group(grasps: GraspGroup) -> GraspGroup:
    return GraspGroup(grasps.grasp_group_array.copy())


def visualization_grasps(result: GraspNetFrameResult, mode: str) -> GraspGroup:
    """Return the grasp set used for Open3D diagnostics."""
    if mode == "pre-bbox":
        return result.pre_bbox_grasps
    if mode == "bbox":
        return result.bbox_grasps
    return result.grasps


def resolve_checkpoint_path(
    checkpoint: str,
    *,
    project_root: Path = PROJECT_ROOT,
    graspnet_root: Path = GRASPNET_ROOT,
) -> Path:
    checkpoint_path = Path(str(checkpoint)).expanduser()
    if checkpoint_path.is_absolute():
        return checkpoint_path
    cwd_candidate = (Path.cwd() / checkpoint_path).resolve()
    if cwd_candidate.is_file():
        return cwd_candidate
    if len(checkpoint_path.parts) > 1:
        return project_root / checkpoint_path
    return graspnet_root / "checkpoints" / checkpoint_path


def build_net(checkpoint_path: str | Path, num_view: int = DEFAULT_NUM_VIEW) -> GraspNet:
    checkpoint_path = resolve_checkpoint_path(str(checkpoint_path))
    if not torch.cuda.is_available():
        raise RuntimeError("GraspNet pointnet2 operators require CUDA, but torch.cuda is unavailable.")

    net = GraspNet(
        input_feature_dim=0,
        num_view=num_view,
        num_angle=12,
        num_depth=4,
        cylinder_radius=0.05,
        hmin=-0.02,
        hmax_list=[0.01, 0.02, 0.03, 0.04],
        is_training=False,
    )
    device = torch.device("cuda:0")
    net.to(device)

    checkpoint = torch.load(str(checkpoint_path), map_location=device)
    net.load_state_dict(checkpoint["model_state_dict"])
    net.eval()
    print(f"Loaded checkpoint {checkpoint_path} (epoch: {checkpoint['epoch']})")
    return net


def _expanded_bbox(
    bbox_xyxy: tuple[int, int, int, int],
    *,
    margin_px: int,
    expand_ratio: float,
    image_shape: Optional[tuple[int, int]] = None,
) -> tuple[int, int, int, int]:
    x1, y1, x2, y2 = [float(value) for value in bbox_xyxy]
    expand_ratio = max(1.0, float(expand_ratio))
    margin_px = max(0, int(margin_px))
    cx = 0.5 * (x1 + x2)
    cy = 0.5 * (y1 + y2)
    half_w = 0.5 * max(1.0, x2 - x1) * expand_ratio
    half_h = 0.5 * max(1.0, y2 - y1) * expand_ratio
    expanded = (
        int(round(cx - half_w)) - margin_px,
        int(round(cy - half_h)) - margin_px,
        int(round(cx + half_w)) + margin_px,
        int(round(cy + half_h)) + margin_px,
    )
    if image_shape is None:
        return expanded

    h, w = image_shape
    return (
        int(np.clip(expanded[0], 0, max(0, w - 1))),
        int(np.clip(expanded[1], 0, max(0, h - 1))),
        int(np.clip(expanded[2], 0, max(0, w - 1))),
        int(np.clip(expanded[3], 0, max(0, h - 1))),
    )


def build_target_roi_mask(
    image_shape: tuple[int, int],
    bbox_xyxy: tuple[int, int, int, int],
    *,
    margin_px: int = 0,
    expand_ratio: float = 1.0,
) -> np.ndarray:
    """Build the expanded YOLO region used to sample GraspNet points."""

    x1, y1, x2, y2 = _expanded_bbox(
        bbox_xyxy,
        margin_px=margin_px,
        expand_ratio=expand_ratio,
        image_shape=image_shape,
    )
    mask = np.zeros(image_shape, dtype=np.uint8)
    if x2 >= x1 and y2 >= y1:
        mask[y1 : y2 + 1, x1 : x2 + 1] = 1
    return mask


def build_target_sample_mask(
    image_shape: tuple[int, int],
    target: YoloDetection,
    *,
    margin_px: int = 0,
    expand_ratio: float = 1.0,
) -> np.ndarray:
    """Use the detector mask, falling back to its expanded bounding box."""

    if target.mask is None:
        return build_target_roi_mask(
            image_shape,
            target.bbox_xyxy,
            margin_px=margin_px,
            expand_ratio=expand_ratio,
        )
    mask = np.asarray(target.mask, dtype=np.uint8)
    if mask.shape != image_shape:
        mask = cv2.resize(
            mask,
            (image_shape[1], image_shape[0]),
            interpolation=cv2.INTER_NEAREST,
        )
    mask = (mask > 0).astype(np.uint8)
    if not np.any(mask):
        return build_target_roi_mask(
            image_shape,
            target.bbox_xyxy,
            margin_px=margin_px,
            expand_ratio=expand_ratio,
        )
    dilation = max(0, int(margin_px))
    if dilation:
        size = 2 * dilation + 1
        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (size, size))
        mask = cv2.dilate(mask, kernel, iterations=1)
    return mask


def build_end_points(
    color_bgr: np.ndarray,
    depth_mm: np.ndarray,
    K: np.ndarray,
    num_point: int,
    min_depth_m: float,
    max_depth_m: float,
    sample_mask: Optional[np.ndarray] = None,
) -> tuple[dict, o3d.geometry.PointCloud, np.ndarray]:
    if color_bgr.shape[:2] != depth_mm.shape[:2]:
        depth_mm = cv2.resize(depth_mm, (color_bgr.shape[1], color_bgr.shape[0]), interpolation=cv2.INTER_NEAREST)

    min_mm = int(max(0.0, min_depth_m) * 1000.0)
    max_mm = int(max_depth_m * 1000.0)
    depth = depth_mm.astype(np.uint16, copy=False)
    valid_depth_mask = (depth > min_mm) & (depth < max_mm)
    if int(valid_depth_mask.sum()) == 0:
        raise RuntimeError("No valid depth pixels in the configured depth range.")

    color_rgb = cv2.cvtColor(color_bgr, cv2.COLOR_BGR2RGB).astype(np.float32) / 255.0
    h, w = depth.shape
    camera = CameraInfo(w, h, float(K[0, 0]), float(K[1, 1]), float(K[0, 2]), float(K[1, 2]), 1000.0)
    cloud = create_point_cloud_from_depth_image(depth, camera, organized=True)
    full_cloud = cloud[valid_depth_mask]
    full_colors = color_rgb[valid_depth_mask]

    network_mask = valid_depth_mask
    if sample_mask is not None:
        sample_mask = np.asarray(sample_mask)
        if sample_mask.shape != depth.shape:
            sample_mask = cv2.resize(
                sample_mask.astype(np.uint8),
                (w, h),
                interpolation=cv2.INTER_NEAREST,
            )
        network_mask = valid_depth_mask & (sample_mask > 0)
        if int(network_mask.sum()) == 0:
            raise RuntimeError(
                "No valid depth pixels inside the selected target ROI."
            )

    sample_cloud = cloud[network_mask]
    sample_colors = color_rgb[network_mask]

    if len(sample_cloud) >= num_point:
        idxs = np.random.choice(len(sample_cloud), num_point, replace=False)
    else:
        idxs = np.concatenate(
            [
                np.arange(len(sample_cloud)),
                np.random.choice(
                    len(sample_cloud),
                    num_point - len(sample_cloud),
                    replace=True,
                ),
            ],
            axis=0,
        )

    end_points = {
        "point_clouds": torch.from_numpy(sample_cloud[idxs].astype(np.float32)[np.newaxis]).cuda(non_blocking=True),
        "cloud_colors": sample_colors[idxs],
    }

    o3d_cloud = o3d.geometry.PointCloud()
    o3d_cloud.points = o3d.utility.Vector3dVector(full_cloud.astype(np.float32))
    o3d_cloud.colors = o3d.utility.Vector3dVector(full_colors.astype(np.float32))
    return end_points, o3d_cloud, sample_cloud


def infer_grasps(
    net: GraspNet,
    end_points: dict,
    raw_cloud: np.ndarray,
    collision_thresh: float,
    voxel_size: float = DEFAULT_VOXEL_SIZE,
) -> tuple[GraspGroup, dict[str, int]]:
    with torch.no_grad():
        end_points = net(end_points)
        grasp_preds = pred_decode(end_points)

    gg = GraspGroup(grasp_preds[0].detach().cpu().numpy())
    decoded_count = len(gg)
    collision_removed = 0
    if len(gg) > 0 and collision_thresh > 0:
        detector = ModelFreeCollisionDetector(raw_cloud, voxel_size=voxel_size)
        collision_mask = detector.detect(gg, approach_dist=0.05, collision_thresh=collision_thresh)
        collision_removed = int(np.count_nonzero(collision_mask))
        gg = gg[~collision_mask]

    return gg, {
        "decoded": decoded_count,
        "pre_collision": decoded_count,
        "collision_removed": collision_removed,
        "final": len(gg),
    }


def filter_grasps_by_bbox(
    grasps: GraspGroup,
    bbox_xyxy: tuple[int, int, int, int],
    K: np.ndarray,
    *,
    margin_px: int = 0,
    expand_ratio: float = 1.0,
    image_shape: Optional[tuple[int, int]] = None,
) -> GraspGroup:
    if len(grasps) == 0:
        return grasps

    translations = np.asarray(grasps.translations, dtype=np.float64)
    z = translations[:, 2]
    valid_z = z > 1e-6
    u = np.full(len(grasps), np.nan, dtype=np.float64)
    v = np.full(len(grasps), np.nan, dtype=np.float64)
    u[valid_z] = float(K[0, 0]) * translations[valid_z, 0] / z[valid_z] + float(K[0, 2])
    v[valid_z] = float(K[1, 1]) * translations[valid_z, 1] / z[valid_z] + float(K[1, 2])

    x1, y1, x2, y2 = _expanded_bbox(
        bbox_xyxy,
        margin_px=margin_px,
        expand_ratio=expand_ratio,
        image_shape=image_shape,
    )

    keep = valid_z & (u >= x1) & (u <= x2) & (v >= y1) & (v <= y2)
    return grasps[keep]


def _project_grasp_centers_px(
    grasps: GraspGroup,
    K: np.ndarray,
) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    intrinsics = np.asarray(K, dtype=np.float64)
    if intrinsics.shape != (3, 3) or not np.all(np.isfinite(intrinsics)):
        raise ValueError("K must be a finite 3x3 matrix")
    translations = np.asarray(grasps.translations, dtype=np.float64)
    z = translations[:, 2]
    valid = np.isfinite(translations).all(axis=1) & (z > 1e-6)
    u = np.full(len(grasps), np.nan, dtype=np.float64)
    v = np.full(len(grasps), np.nan, dtype=np.float64)
    u[valid] = (
        intrinsics[0, 0] * translations[valid, 0] / z[valid]
        + intrinsics[0, 2]
    )
    v[valid] = (
        intrinsics[1, 1] * translations[valid, 1] / z[valid]
        + intrinsics[1, 2]
    )
    return u, v, valid


def filter_grasps_by_target_mask(
    grasps: GraspGroup,
    target_mask: np.ndarray,
    K: np.ndarray,
    *,
    minimum_edge_distance_px: float,
) -> GraspGroup:
    """Keep grasp centers inside the target's mask safety core."""

    if len(grasps) == 0:
        return grasps
    minimum_distance = float(minimum_edge_distance_px)
    if not np.isfinite(minimum_distance) or minimum_distance < 0.0:
        raise ValueError("minimum_edge_distance_px must be nonnegative and finite")
    mask = (np.asarray(target_mask) > 0).astype(np.uint8)
    if mask.ndim != 2 or not np.any(mask):
        return GraspGroup()
    projected_u, projected_v, valid_z = _project_grasp_centers_px(grasps, K)
    u = np.rint(np.nan_to_num(projected_u, nan=-1.0)).astype(np.int64)
    v = np.rint(np.nan_to_num(projected_v, nan=-1.0)).astype(np.int64)

    height, width = mask.shape
    in_image = (
        valid_z
        & (u >= 0)
        & (u < width)
        & (v >= 0)
        & (v < height)
    )
    distance = cv2.distanceTransform(mask, cv2.DIST_L2, 5)
    keep = np.zeros(len(grasps), dtype=bool)
    indices = np.flatnonzero(in_image)
    if len(indices):
        keep[indices] = (
            distance[v[indices], u[indices]] >= minimum_distance
        ) & (mask[v[indices], u[indices]] > 0)
    return grasps[keep]


def bias_grasp_scores_by_mask_center(
    grasps: GraspGroup,
    target_mask: np.ndarray,
    K: np.ndarray,
    *,
    weight: float,
) -> GraspGroup:
    """Return a copy whose scores softly prefer the mask interior center."""

    result = copy_grasp_group(grasps)
    if len(result) == 0:
        return result
    center_weight = float(weight)
    if not np.isfinite(center_weight) or not 0.0 <= center_weight <= 1.0:
        raise ValueError("mask center score weight must be in [0, 1]")
    mask = (np.asarray(target_mask) > 0).astype(np.uint8)
    if mask.ndim != 2 or not np.any(mask):
        return result
    distance = cv2.distanceTransform(mask, cv2.DIST_L2, 5)
    maximum_distance = float(np.max(distance))
    if maximum_distance <= 0.0 or center_weight == 0.0:
        return result

    projected_u, projected_v, valid = _project_grasp_centers_px(result, K)
    u = np.rint(np.nan_to_num(projected_u, nan=-1.0)).astype(np.int64)
    v = np.rint(np.nan_to_num(projected_v, nan=-1.0)).astype(np.int64)
    height, width = mask.shape
    valid &= (u >= 0) & (u < width) & (v >= 0) & (v < height)
    quality = np.zeros(len(result), dtype=np.float64)
    indices = np.flatnonzero(valid)
    if len(indices):
        quality[indices] = np.clip(
            distance[v[indices], u[indices]] / maximum_distance,
            0.0,
            1.0,
        )
    result.scores = np.asarray(result.scores, dtype=np.float64) * (
        1.0 - center_weight + center_weight * quality
    )
    return result


def anchor_grasps_to_target_center(
    grasps: GraspGroup,
    target_center_cam: np.ndarray,
) -> GraspGroup:
    """Keep GraspNet orientations while using one trusted cube center."""

    result = copy_grasp_group(grasps)
    center = np.asarray(target_center_cam, dtype=np.float64).reshape(-1)
    if center.shape != (3,) or not np.all(np.isfinite(center)):
        raise ValueError("target_center_cam must contain three finite values")
    if len(result):
        result.translations = np.repeat(
            center.reshape(1, 3),
            len(result),
            axis=0,
        )
    return result


def filter_grasps_by_local_depth_support(
    grasps: GraspGroup,
    target_mask: np.ndarray,
    depth_mm: np.ndarray,
    K: np.ndarray,
    *,
    window_radius_px: int,
    minimum_mask_fraction: float,
    minimum_valid_depth_fraction: float,
    maximum_depth_mad_m: float,
    maximum_center_depth_error_m: float,
) -> GraspGroup:
    """Reject centers whose local target depth is sparse or unstable."""

    if len(grasps) == 0:
        return grasps
    radius = int(window_radius_px)
    min_mask = float(minimum_mask_fraction)
    min_valid = float(minimum_valid_depth_fraction)
    max_mad = float(maximum_depth_mad_m)
    max_error = float(maximum_center_depth_error_m)
    if radius < 0:
        raise ValueError("window_radius_px must be nonnegative")
    if not 0.0 < min_mask <= 1.0 or not 0.0 < min_valid <= 1.0:
        raise ValueError("depth support fractions must be in (0, 1]")
    if not np.isfinite(max_mad) or max_mad < 0.0:
        raise ValueError("maximum_depth_mad_m must be nonnegative and finite")
    if not np.isfinite(max_error) or max_error < 0.0:
        raise ValueError("maximum_center_depth_error_m must be nonnegative and finite")

    mask = np.asarray(target_mask) > 0
    depth = np.asarray(depth_mm)
    if mask.ndim != 2 or depth.shape != mask.shape:
        raise ValueError("target mask and depth must have matching 2D shapes")
    projected_u, projected_v, valid = _project_grasp_centers_px(grasps, K)
    u = np.rint(np.nan_to_num(projected_u, nan=-1.0)).astype(np.int64)
    v = np.rint(np.nan_to_num(projected_v, nan=-1.0)).astype(np.int64)
    height, width = mask.shape
    keep = np.zeros(len(grasps), dtype=bool)
    translations = np.asarray(grasps.translations, dtype=np.float64)
    for index in np.flatnonzero(valid):
        if not (0 <= u[index] < width and 0 <= v[index] < height):
            continue
        x1, x2 = max(0, u[index] - radius), min(width, u[index] + radius + 1)
        y1, y2 = max(0, v[index] - radius), min(height, v[index] + radius + 1)
        local_mask = mask[y1:y2, x1:x2]
        mask_count = int(np.count_nonzero(local_mask))
        if mask_count < min_mask * float(local_mask.size):
            continue
        local_depth = depth[y1:y2, x1:x2]
        valid_depth = local_mask & np.isfinite(local_depth) & (local_depth > 0)
        values = np.asarray(local_depth[valid_depth], dtype=np.float64) / 1000.0
        if len(values) < min_valid * float(mask_count):
            continue
        median = float(np.median(values))
        robust_deviation = float(
            np.percentile(np.abs(values - median), 90.0)
        )
        if robust_deviation > max_mad:
            continue
        if abs(float(translations[index, 2]) - median) > max_error:
            continue
        keep[index] = True
    return grasps[keep]


def filter_grasps_by_target_depth(
    grasps: GraspGroup,
    *,
    target_depth_m: float,
    tolerance_m: float,
) -> GraspGroup:
    """Keep grasp centers on the selected target's depth layer."""

    if len(grasps) == 0:
        return grasps
    target_depth = float(target_depth_m)
    tolerance = float(tolerance_m)
    if not np.isfinite(target_depth) or target_depth <= 0.0:
        raise ValueError("target depth must be positive and finite")
    if not np.isfinite(tolerance) or tolerance <= 0.0:
        raise ValueError("target depth tolerance must be positive and finite")
    z = np.asarray(grasps.translations, dtype=np.float64)[:, 2]
    return grasps[
        np.isfinite(z) & (np.abs(z - target_depth) <= tolerance)
    ]


def estimate_target_depth(
    depth_mm: np.ndarray,
    target: YoloDetection,
    *,
    min_depth_m: float,
    max_depth_m: float,
    tolerance_m: float,
    fallback_mask: np.ndarray | None,
) -> float | None:
    """Estimate target depth from its connected bbox component or mask."""

    try:
        from .urdf_gripper_collision import build_target_depth_exclusion_mask
    except ImportError:
        from urdf_gripper_collision import build_target_depth_exclusion_mask

    depth = np.asarray(depth_mm)
    component = build_target_depth_exclusion_mask(
        depth,
        bbox_xyxy=target.bbox_xyxy,
        min_depth_m=min_depth_m,
        max_depth_m=max_depth_m,
        depth_tolerance_m=tolerance_m,
    )
    valid = component > 0
    if not np.any(valid) and fallback_mask is not None:
        mask = np.asarray(fallback_mask)
        if mask.shape != depth.shape:
            mask = cv2.resize(
                mask.astype(np.uint8),
                (depth.shape[1], depth.shape[0]),
                interpolation=cv2.INTER_NEAREST,
            )
        valid = mask > 0
    z = depth.astype(np.float64) / 1000.0
    valid &= (
        np.isfinite(z)
        & (z > float(min_depth_m))
        & (z < float(max_depth_m))
    )
    if not np.any(valid):
        return None
    return float(np.median(z[valid]))


def filter_grasps_by_width(
    grasps: GraspGroup,
    max_width_m: Optional[float],
    *,
    tolerance_m: float = 0.0,
) -> GraspGroup:
    if max_width_m is None or len(grasps) == 0:
        return grasps
    tolerance = float(tolerance_m)
    if not np.isfinite(tolerance) or tolerance < 0.0:
        raise ValueError("grasp width tolerance must be finite and nonnegative")
    return grasps[
        np.asarray(grasps.widths, dtype=np.float64)
        <= float(max_width_m) + tolerance
    ]


def select_best_grasp(grasps: GraspGroup) -> Optional[Grasp]:
    if len(grasps) == 0:
        return None
    ranked = GraspGroup(grasps.grasp_group_array.copy())
    try:
        ranked = ranked.nms()
    except Exception as exc:
        print(f"[WARN] GraspNet NMS skipped: {exc}")
    ranked.sort_by_score()
    return ranked[0] if len(ranked) > 0 else None


def _normalized_class_name(name: str) -> str:
    return "".join(ch for ch in str(name).casefold() if ch.isalnum())


def select_target(detections: list[YoloDetection], target_class: Optional[str]) -> Optional[YoloDetection]:
    if not detections:
        return None
    candidates = detections
    if target_class:
        target_norm = _normalized_class_name(target_class)
        exact = [
            target
            for target in detections
            if _normalized_class_name(target.class_name) == target_norm
        ]
        contains = [
            target
            for target in detections
            if target_norm in _normalized_class_name(target.class_name)
        ]
        candidates = exact or contains
    if not candidates:
        return None
    return max(candidates, key=lambda target: target.conf)


def selected_target_text(selected: Optional[YoloDetection], target_class: Optional[str]) -> str:
    if selected is None:
        return f"target={target_class or 'best'} not found"
    return f"target={selected.class_name} {selected.conf:.2f}"


def target_status_text(selected: Optional[YoloDetection], detections: list[YoloDetection], target_class: Optional[str]) -> str:
    if selected is not None:
        return f"target={selected.class_name} {selected.conf:.2f} detections={len(detections)}"
    if target_class:
        return f"target={target_class} not found detections={len(detections)}"
    return f"target not found detections={len(detections)}"


def draw_detections_overlay(
    frame: np.ndarray,
    detections: list[YoloDetection],
    selected: Optional[YoloDetection],
    target_class: Optional[str],
) -> np.ndarray:
    display = frame.copy()
    selected_key = None
    if selected is not None:
        selected_key = (selected.result_index, selected.detection_index)
    for target in detections:
        is_selected = selected_key == (target.result_index, target.detection_index)
        color = (0, 255, 80) if is_selected else (0, 185, 255)
        thickness = 3 if is_selected else 2
        x1, y1, x2, y2 = target.bbox_xyxy
        cv2.rectangle(display, (x1, y1), (x2, y2), color, thickness)
        label = f"{target.class_name} {target.conf:.2f}"
        if target_class and is_selected:
            label = f"TARGET {label}"
        label_size, _ = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, 0.55, 2)
        bg_y1 = max(0, y1 - label_size[1] - 8)
        cv2.rectangle(display, (x1, bg_y1), (x1 + label_size[0] + 8, y1), (0, 0, 0), -1)
        cv2.putText(display, label, (x1 + 4, y1 - 5), cv2.FONT_HERSHEY_SIMPLEX, 0.55, color, 2, cv2.LINE_AA)
    return display


def draw_best_grasp_projection(display: np.ndarray, grasp: Optional[Grasp], K: np.ndarray) -> None:
    if grasp is None:
        return
    x, y, z = [float(v) for v in grasp.translation]
    if z <= 1e-6:
        return
    u = int(round(float(K[0, 0]) * x / z + float(K[0, 2])))
    v = int(round(float(K[1, 1]) * y / z + float(K[1, 2])))
    if 0 <= u < display.shape[1] and 0 <= v < display.shape[0]:
        cv2.drawMarker(display, (u, v), (0, 0, 255), cv2.MARKER_CROSS, 22, 2, cv2.LINE_AA)
        label = f"best score={grasp.score:.2f} width={grasp.width * 100:.1f}cm"
        cv2.putText(display, label, (u + 10, max(24, v - 10)), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 0, 255), 2)


def grasp_to_base_poses(
    grasp: Grasp,
    T_cam2base: np.ndarray,
    pregrasp_offset_m: float,
    retreat_offset_m: float,
    insertion_depth_m: float = 0.0,
    *,
    orientation_constraint: OrientationConstraint | None = None,
    base_rotation_override: np.ndarray | None = None,
) -> tuple[tuple[float, ...], tuple[float, ...], tuple[float, ...]]:
    tcp_rotation_cam = graspnet_rotation_to_rebot_tcp_rotation(
        grasp.rotation_matrix
    )
    if orientation_constraint is None and base_rotation_override is None:
        return transform_grasp_pose_to_base_with_retreat(
            np.asarray(grasp.translation, dtype=np.float64),
            tcp_rotation_cam,
            T_cam2base,
            pregrasp_offset_m,
            retreat_offset_m,
            insertion_depth_m,
            canonicalize_rotation=True,
        )

    center6d, _, _ = transform_grasp_pose_to_base_with_retreat(
        np.asarray(grasp.translation, dtype=np.float64),
        tcp_rotation_cam,
        T_cam2base,
        0.0,
        0.0,
        0.0,
        canonicalize_rotation=True,
    )
    grasp_transform = pose6d_to_mat4(*center6d)
    if base_rotation_override is None:
        assert orientation_constraint is not None
        grasp_transform[:3, :3] = constrain_rotation(
            grasp_transform[:3, :3],
            orientation_constraint,
        )
    else:
        override = np.asarray(base_rotation_override, dtype=np.float64)
        if (
            override.shape != (3, 3)
            or not np.all(np.isfinite(override))
            or not np.allclose(override.T @ override, np.eye(3), atol=1e-6)
            or not np.isclose(np.linalg.det(override), 1.0, atol=1e-6)
        ):
            raise ValueError(
                "base_rotation_override must be a finite proper rotation"
            )
        if orientation_constraint is not None:
            override = constrain_rotation(override, orientation_constraint)
        grasp_transform[:3, :3] = override
    grasp_transform[:3, 3] += (
        grasp_transform[:3, 0] * float(insertion_depth_m)
    )

    def offset_from_grasp(distance_m: float) -> np.ndarray:
        transform = grasp_transform.copy()
        transform[:3, 3] -= transform[:3, 0] * float(distance_m)
        return transform

    return (
        mat4_to_pose6d(grasp_transform),
        mat4_to_pose6d(offset_from_grasp(pregrasp_offset_m)),
        mat4_to_pose6d(offset_from_grasp(retreat_offset_m)),
    )


def draw_status(
    frame: np.ndarray,
    status: str,
    target_status: str = "",
    frozen: bool = False,
    title: str = "GraspNet Full-Scene Demo",
) -> np.ndarray:
    display = frame.copy()
    lines = [
        title,
        "G/SPACE: infer   R: resume   Q/ESC: quit",
    ]
    if target_status:
        lines.append(target_status)
    lines.append(status)
    y = 28
    for line in lines:
        cv2.putText(display, line, (10, y), cv2.FONT_HERSHEY_SIMPLEX, 0.65, (0, 0, 0), 3, cv2.LINE_AA)
        cv2.putText(display, line, (10, y), cv2.FONT_HERSHEY_SIMPLEX, 0.65, (255, 255, 255), 1, cv2.LINE_AA)
        y += 26
    if frozen:
        cv2.putText(display, "[FROZEN]", (10, y), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 215, 255), 2)
    return display


def infer_frame(
    net: GraspNet,
    color_bgr: np.ndarray,
    depth_mm: np.ndarray,
    K: np.ndarray,
    *,
    num_point: int,
    min_depth: float,
    max_depth: float,
    collision_thresh: float,
    voxel_size: float = DEFAULT_VOXEL_SIZE,
    yolo_model: Optional[Any] = None,
    yolo_opts: Optional[dict[str, Any]] = None,
    target_class: Optional[str] = None,
    target_margin_px: int = 0,
    target_expand_ratio: float = 1.0,
    target_depth_tolerance_m: float = 0.06,
    target_min_edge_distance_px: float = 0.0,
    target_center_score_weight: float = 0.0,
    target_depth_window_radius_px: int = 0,
    target_min_mask_support: float = 0.0,
    target_min_valid_depth_fraction: float = 0.0,
    target_max_local_depth_deviation_m: float = 0.0,
    target_max_center_depth_error_m: float = 0.0,
    max_grasp_width_m: Optional[float] = None,
    grasp_width_tolerance_m: float = 0.0,
    selected_target_override: Optional[YoloDetection] = None,
) -> GraspNetFrameResult:
    try:
        from .yolo_utils import detect_objects
    except ImportError:
        from yolo_utils import detect_objects

    detections: list[YoloDetection] = []
    selected_target: Optional[YoloDetection] = None
    target_label = "full scene"

    if selected_target_override is not None:
        selected_target = selected_target_override
        detections = [selected_target_override]
        target_label = (
            f"{selected_target.class_name} {selected_target.conf:.2f}"
        )
    elif yolo_model is not None:
        _, detections = detect_objects(yolo_model, color_bgr, yolo_opts or {})
        selected_target = select_target(detections, target_class)
        if selected_target is None:
            target_status = target_status_text(selected_target, detections, target_class)
            empty = GraspGroup()
            empty_cloud = o3d.geometry.PointCloud()
            return GraspNetFrameResult(
                grasps=empty,
                pre_bbox_grasps=empty,
                bbox_grasps=empty,
                best=None,
                status=f"inference skipped: {target_status}",
                target_status=target_status,
                detections=detections,
                selected_target=None,
                o3d_cloud=empty_cloud,
                raw_cloud=np.empty((0, 3), dtype=np.float32),
            )
        target_label = f"{selected_target.class_name} {selected_target.conf:.2f}"

    target_filter_mask = None
    if selected_target is not None:
        target_filter_mask = build_target_sample_mask(
            color_bgr.shape[:2],
            selected_target,
            margin_px=target_margin_px,
            expand_ratio=target_expand_ratio,
        )

    tic = time.time()
    end_points, o3d_cloud, raw_cloud = build_end_points(
        color_bgr,
        depth_mm,
        K,
        num_point,
        min_depth,
        max_depth,
        sample_mask=target_filter_mask,
    )
    grasps, counts = infer_grasps(net, end_points, raw_cloud, collision_thresh, voxel_size)
    pre_bbox_grasps = copy_grasp_group(grasps)
    if selected_target is not None:
        grasps = filter_grasps_by_bbox(
            grasps,
            selected_target.bbox_xyxy,
            K,
            margin_px=target_margin_px,
            expand_ratio=target_expand_ratio,
            image_shape=color_bgr.shape[:2],
        )
        target_core_mask = build_target_sample_mask(
            color_bgr.shape[:2],
            selected_target,
            margin_px=0,
            expand_ratio=1.0,
        )
        grasps = filter_grasps_by_target_mask(
            grasps,
            target_core_mask,
            K,
            minimum_edge_distance_px=target_min_edge_distance_px,
        )
        target_depth = estimate_target_depth(
            depth_mm,
            selected_target,
            min_depth_m=min_depth,
            max_depth_m=max_depth,
            tolerance_m=target_depth_tolerance_m,
            fallback_mask=target_filter_mask,
        )
        if target_depth is not None:
            grasps = filter_grasps_by_target_depth(
                grasps,
                target_depth_m=target_depth,
                tolerance_m=target_depth_tolerance_m,
            )
        if (
            target_min_mask_support > 0.0
            and target_min_valid_depth_fraction > 0.0
        ):
            grasps = filter_grasps_by_local_depth_support(
                grasps,
                target_core_mask,
                depth_mm,
                K,
                window_radius_px=target_depth_window_radius_px,
                minimum_mask_fraction=target_min_mask_support,
                minimum_valid_depth_fraction=(
                    target_min_valid_depth_fraction
                ),
                maximum_depth_mad_m=(
                    target_max_local_depth_deviation_m
                ),
                maximum_center_depth_error_m=(
                    target_max_center_depth_error_m
                ),
            )
        grasps = bias_grasp_scores_by_mask_center(
            grasps,
            target_core_mask,
            K,
            weight=target_center_score_weight,
        )
    bbox_grasps = copy_grasp_group(grasps)
    target_widths = np.asarray(
        getattr(bbox_grasps, "widths", ()),
        dtype=np.float64,
    )
    grasps = filter_grasps_by_width(
        grasps,
        max_grasp_width_m,
        tolerance_m=grasp_width_tolerance_m,
    )
    best = select_best_grasp(grasps)
    elapsed = time.time() - tic

    if selected_target is None:
        status = f"grasps={len(grasps)} decoded={counts['decoded']} inference={elapsed:.2f}s"
        target_status = "YOLO disabled: full-scene GraspNet"
    else:
        width_text = (
            f" width=[{np.min(target_widths):.3f},"
            f"{np.max(target_widths):.3f}]m"
            if target_widths.size
            else ""
        )
        status = (
            f"{target_label} grasps={len(grasps)}/{len(bbox_grasps)}/{len(pre_bbox_grasps)} decoded={counts['decoded']} "
            f"collide={counts['collision_removed']}/{counts['pre_collision']}"
            f"{width_text} inference={elapsed:.2f}s"
        )
        target_status = target_status_text(selected_target, detections, target_class)

    if len(bbox_grasps) > 0 and len(grasps) == 0 and max_grasp_width_m is not None:
        status += (
            f"; rejected: width filter removed all candidates; width_limit={float(max_grasp_width_m):.3f}m"
            f" (+{float(grasp_width_tolerance_m):.3f}m tolerance)"
        )

    return GraspNetFrameResult(
        grasps=grasps,
        pre_bbox_grasps=pre_bbox_grasps,
        bbox_grasps=bbox_grasps,
        best=best,
        status=status,
        target_status=target_status,
        detections=detections,
        selected_target=selected_target,
        o3d_cloud=o3d_cloud,
        raw_cloud=raw_cloud,
    )
