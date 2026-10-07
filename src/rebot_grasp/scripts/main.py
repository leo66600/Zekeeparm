"""Minimal direct-SDK visual grasp route.

Pipeline: YOLO target -> single-frame GraspNet -> gripper point-cloud filter
-> TCP floor check -> three-pose SDK IK precheck -> grasp/retreat/place/ready.

This script deliberately does not start ROS or MoveIt. It owns the motor port
through ``OfficialSdkRobot`` and must not run beside a ROS driver or Gateway.
"""

from __future__ import annotations

import argparse
import os
import re
import sys
import time
from pathlib import Path
from typing import Any

os.environ.setdefault("MPLCONFIGDIR", "/tmp/matplotlib")
os.environ.setdefault("QT_QPA_FONTDIR", "/usr/share/fonts/truetype")

PROJECT_ROOT = Path(__file__).resolve().parents[1]
SDK_ROOT = PROJECT_ROOT.parents[1] / "zekeeparm_SDK"
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))
if str(SDK_ROOT) not in sys.path:
    sys.path.insert(1, str(SDK_ROOT))

from utils.native_runtime import preload_environment_libstdcpp  # noqa: E402

preload_environment_libstdcpp()

import cv2  # noqa: E402
import numpy as np  # noqa: E402

from drivers.camera import make_camera  # noqa: E402
from drivers.robot.official_sdk_robot import OfficialSdkRobot  # noqa: E402
from utils.camera_utils import (  # noqa: E402
    camera_to_base_from_hand_eye_reference,
    configure_camera,
    load_config,
    load_hand_eye,
)
from utils.cube_perception import (  # noqa: E402
    assess_cube_geometry,
)
from utils.graspnet_utils import (  # noqa: E402
    build_net,
    draw_detections_overlay,
    grasp_to_base_poses,
    infer_frame,
    resolve_checkpoint_path,
    select_target,
    target_status_text,
)
import utils.graspnet_utils as graspnet_utils  # noqa: E402
from utils.orientation_constraints import OrientationConstraint, constrain_rotation  # noqa: E402
from utils.transforms import pose6d_to_mat4  # noqa: E402
from utils.urdf_gripper_collision import (  # noqa: E402
    UrdfGripperCollisionModel,
    transform_points,
)
from utils.official_grasp_runtime import mask_obb  # noqa: E402
from utils.official_execution import (  # noqa: E402
    MotionGateError,
    validate_execution_targets,
)
from utils.shape_orientation import masked_depth_points_base  # noqa: E402
from utils.yolo_utils import detect_objects, load_yolo  # noqa: E402


def _args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Minimal direct-SDK visual grasp")
    parser.add_argument("--config", default=str(PROJECT_ROOT / "config" / "default.yaml"))
    parser.add_argument("--target-class", default=None)
    parser.add_argument("--camera-type", default=None)
    parser.add_argument("--checkpoint", default=None)
    return parser.parse_args()


def _target_class(cfg: dict[str, Any], args: argparse.Namespace) -> str:
    value = args.target_class
    if value is None:
        value = (cfg.get("graspnet") or {}).get("target_class")
    value = str(value or "").strip()
    if not value:
        raise ValueError("target class required: pass --target-class or set graspnet.target_class")
    return value


def _simple_cfg(cfg: dict[str, Any]) -> tuple[float, float, float, float, float, float]:
    gp = cfg.get("grasp_pipeline") or {}
    grasp = gp.get("grasp") or {}
    safety = cfg.get("safety") or {}
    min_z = float(grasp.get("min_base_z_m", safety.get("min_base_z_m", 0.008)))
    pre = float(grasp.get("pregrasp_offset_m", 0.08))
    retreat = float(grasp.get("retreat_offset_m", pre))
    insertion = float(grasp.get("insertion_depth_m", 0.0))
    collision_thresh = float((cfg.get("graspnet") or {}).get("collision_thresh", 0.01))
    voxel = float((cfg.get("graspnet") or {}).get("voxel_size", 0.01))
    if min_z < 0.0 or pre < 0.0 or retreat < 0.0 or insertion < 0.0:
        raise ValueError("simple grasp distances and min_base_z_m must be nonnegative")
    return min_z, pre, retreat, insertion, collision_thresh, voxel


def _pregrasp_offset_candidates(config: dict[str, Any], primary_offset: float) -> list[float]:
    grasp_cfg = (config.get("grasp_pipeline") or {}).get("grasp") or {}
    if not bool(grasp_cfg.get("allow_shorter_descent_fallback", False)):
        return [float(primary_offset)]
    raw_candidates = grasp_cfg.get("pregrasp_offset_candidates_m", [primary_offset])
    return _offset_candidate_values(raw_candidates, primary_offset)


def _offset_candidate_values(raw_candidates: Any, primary_offset: float) -> list[float]:
    candidates = [float(primary_offset)]
    if raw_candidates is None:
        raw_candidates = []
    for value in raw_candidates:
        candidate = float(value)
        if not np.isfinite(candidate) or candidate < 0.0:
            raise ValueError("pregrasp_offset_candidates_m must be finite and nonnegative")
        if all(abs(candidate - existing) > 1e-9 for existing in candidates):
            candidates.append(candidate)
    return candidates


def _far_pregrasp_offset_candidates(
    config: dict[str, Any], primary_offset: float
) -> list[float]:
    grasp_cfg = (config.get("grasp_pipeline") or {}).get("grasp") or {}
    if not bool(grasp_cfg.get("allow_shorter_descent_fallback", False)):
        return [float(primary_offset)]
    raw_candidates = grasp_cfg.get("far_pregrasp_offset_candidates_m", [0.030, 0.012])
    primary = float(raw_candidates[0]) if raw_candidates else float(primary_offset)
    return _offset_candidate_values(raw_candidates, primary)


def _command_grasp_z(
    measured_center_z: float,
    grasp_z_offset_m: float,
    min_z: float,
    config: dict[str, Any],
) -> float:
    del config  # Kept in the signature for compatibility with existing callers.
    requested_z = float(measured_center_z) + float(grasp_z_offset_m)
    safety_floor = float(min_z)
    if not np.isfinite(requested_z) or not np.isfinite(safety_floor):
        raise ValueError("requested grasp Z and safety floor must be finite")
    if requested_z < safety_floor:
        raise ValueError(
            f"requested grasp Z {requested_z:.4f}m is below safety floor "
            f"{safety_floor:.4f}m"
        )
    return requested_z


def _joint_transition_waypoints(
    start_joints: np.ndarray,
    target_joints: np.ndarray,
    max_segment_delta_rad: float,
) -> list[np.ndarray]:
    start = np.asarray(start_joints, dtype=np.float64).reshape(6)
    target = np.asarray(target_joints, dtype=np.float64).reshape(6)
    limit = float(max_segment_delta_rad)
    if not np.isfinite(limit) or limit <= 0.0:
        raise ValueError("max_segment_delta_rad must be positive")
    delta = float(np.max(np.abs(target - start)))
    if delta <= limit:
        return []
    segment_count = int(np.ceil(delta / limit))
    return [
        start + (target - start) * (index / segment_count)
        for index in range(1, segment_count)
    ]


def _validated_solutions_with_transit(
    poses: tuple[tuple[float, ...], tuple[float, ...], tuple[float, ...]],
    base_solutions: list[np.ndarray],
    *,
    start_joints: np.ndarray,
    max_segment_joint_delta_rad: float,
    jaw_width_m: float,
    joint_limits: np.ndarray,
    workspace: dict,
    min_tcp_z_m: float,
    gripper_max_width_m: float,
) -> list[np.ndarray]:
    pose_arrays = [np.asarray(pose) for pose in poses]
    try:
        validate_execution_targets(
            pose_arrays,
            base_solutions,
            jaw_width_m=jaw_width_m,
            joint_limits=joint_limits,
            workspace=workspace,
            min_tcp_z_m=min_tcp_z_m,
            max_segment_joint_delta_rad=max_segment_joint_delta_rad,
            gripper_max_width_m=gripper_max_width_m,
            start_joints=start_joints,
        )
        return base_solutions
    except MotionGateError as exc:
        if "joint segment delta" not in str(exc):
            raise
    transit = _joint_transition_waypoints(
        start_joints,
        base_solutions[0],
        max_segment_joint_delta_rad,
    )
    if not transit:
        raise MotionGateError("joint segment delta exceeds limit")
    validate_execution_targets(
        pose_arrays,
        base_solutions,
        jaw_width_m=jaw_width_m,
        joint_limits=joint_limits,
        workspace=workspace,
        min_tcp_z_m=min_tcp_z_m,
        max_segment_joint_delta_rad=max_segment_joint_delta_rad,
        gripper_max_width_m=gripper_max_width_m,
        start_joints=None,
    )
    segment_targets = [np.asarray(start_joints, dtype=np.float64), *transit, base_solutions[0]]
    for start, end in zip(segment_targets, segment_targets[1:]):
        delta = float(np.max(np.abs(np.asarray(end) - np.asarray(start))))
        if delta > float(max_segment_joint_delta_rad):
            raise MotionGateError(f"joint segment delta {delta:.4f}rad exceeds limit")
    return [*transit, *base_solutions]


def _sorted_grasps(group: Any) -> Any:
    ranked = graspnet_utils.copy_grasp_group(group)
    try:
        ranked = ranked.nms()
    except Exception as exc:
        print(f"[GraspNet] NMS skipped: {exc}")
    ranked.sort_by_score()
    return ranked


def _anchor_grasps_to_measured_center(
    result: Any,
    depth_mm: np.ndarray,
    camera_K: np.ndarray,
    T_cam2base: np.ndarray,
    cfg: dict[str, Any],
    *,
    min_z: float,
) -> bool:
    """Replace network translations with the measured object center."""
    target = result.selected_target
    if target is None:
        print("[Geometry] target detection unavailable; skip grasp")
        return False
    gp_cfg = cfg.get("graspnet") or {}
    mask = graspnet_utils.build_target_sample_mask(
        depth_mm.shape,
        target,
        margin_px=0,
        expand_ratio=1.0,
    )
    points_base = masked_depth_points_base(
        depth_mm,
        mask,
        camera_K,
        T_cam2base,
        min_depth_m=float(gp_cfg.get("min_depth", 0.05)),
        max_depth_m=float(gp_cfg.get("max_depth", 1.0)),
    )
    table_z = float((cfg.get("safety") or {}).get("table_z_m", 0.0))
    geometry = assess_cube_geometry(
        points_base,
        table_z_m=table_z,
        enforce_size_bounds=False,
    )
    if not geometry.valid or geometry.center_base is None:
        print(
            f"[Geometry] invalid target geometry points={geometry.point_count}; "
            "skip grasp"
        )
        return False
    center_base = np.asarray(geometry.center_base, dtype=np.float64).copy()
    measured_center_z = float(center_base[2])
    # Apply exactly the configured base-Z correction. Safety limits reject an
    # unsafe request instead of silently raising it to a different height.
    grasp_z_offset_m = float(
        (cfg.get("grasp_pipeline") or {}).get("grasp", {}).get(
            "grasp_center_z_offset_m", 0.0
        )
    )
    grasp_x_offset_m = float(
        (cfg.get("grasp_pipeline") or {}).get("grasp", {}).get(
            "grasp_center_x_offset_m", 0.0
        )
    )
    grasp_y_offset_m = float(
        (cfg.get("grasp_pipeline") or {}).get("grasp", {}).get(
            "grasp_center_y_offset_m", 0.0
        )
    )
    try:
        center_base[2] = _command_grasp_z(
            measured_center_z,
            grasp_z_offset_m,
            min_z,
            cfg,
        )
    except ValueError as exc:
        print(f"[Geometry] {exc}; skip grasp")
        return False
    # Use segmentation OBB center for horizontal placement.  The raw
    # GraspNet translation can sit on a visible side/contact patch, which
    # makes the jaws hit the object instead of straddling its center.
    try:
        obb = mask_obb(mask)
    except (TypeError, ValueError, cv2.error) as exc:
        print(f"[Geometry] invalid target OBB: {exc}")
        return False
    u = int(round(obb.center_px[0]))
    v = int(round(obb.center_px[1]))
    radius = 5
    y1, y2 = max(0, v - radius), min(depth_mm.shape[0], v + radius + 1)
    x1, x2 = max(0, u - radius), min(depth_mm.shape[1], u + radius + 1)
    local_depth = depth_mm[y1:y2, x1:x2].astype(np.float64)
    local_mask = mask[y1:y2, x1:x2] > 0
    valid_local = local_mask & np.isfinite(local_depth) & (local_depth > 0.0)
    if not np.any(valid_local):
        print("[Geometry] OBB center has no valid depth; skip grasp")
        return False
    center_depth_camera = float(np.median(local_depth[valid_local])) / 1000.0
    # Intersect camera ray through OBB center with measured object-center
    # plane.  Do not project a 3D point then overwrite base Z: with a tilted
    # camera that moves the image point vertically, causing systematic offset.
    ray_camera = np.array(
        [
            (float(obb.center_px[0]) - float(camera_K[0, 2])) / float(camera_K[0, 0]),
            (float(obb.center_px[1]) - float(camera_K[1, 2])) / float(camera_K[1, 1]),
            1.0,
        ],
        dtype=np.float64,
    )
    camera_origin_base = np.asarray(T_cam2base[:3, 3], dtype=np.float64)
    ray_base = np.asarray(T_cam2base[:3, :3], dtype=np.float64) @ ray_camera
    if abs(float(ray_base[2])) < 1e-9:
        print("[Geometry] OBB center ray parallel to object plane; skip grasp")
        return False
    ray_scale = (float(center_base[2]) - float(camera_origin_base[2])) / float(ray_base[2])
    if not np.isfinite(ray_scale) or ray_scale <= 0.0 or not np.isfinite(center_depth_camera):
        print("[Geometry] invalid OBB center ray intersection; skip grasp")
        return False
    center_base_from_pixel = camera_origin_base + ray_scale * ray_base
    center_base_from_pixel[0] += grasp_x_offset_m
    center_base_from_pixel[1] += grasp_y_offset_m
    center_camera = transform_points(
        center_base_from_pixel.reshape(1, 3), np.linalg.inv(T_cam2base)
    )[0]
    opening_plus = _target_ray_at_base_z(
        (obb.center_px[0] + 20.0 * obb.opening_axis_px[0],
         obb.center_px[1] + 20.0 * obb.opening_axis_px[1]),
        center_base[2],
        T_cam2base,
        camera_K,
    )
    opening_minus = _target_ray_at_base_z(
        (obb.center_px[0] - 20.0 * obb.opening_axis_px[0],
         obb.center_px[1] - 20.0 * obb.opening_axis_px[1]),
        center_base[2],
        T_cam2base,
        camera_K,
    )
    opening_axis_base = opening_plus - opening_minus
    opening_axis_base[2] = 0.0
    opening_norm = float(np.linalg.norm(opening_axis_base))
    if opening_norm < 1e-9:
        print("[Geometry] OBB opening axis is degenerate; skip grasp")
        return False
    result.obb_opening_axis_base = opening_axis_base / opening_norm
    result.grasps.translations = np.repeat(
        center_camera.reshape(1, 3), len(result.grasps), axis=0
    )
    result.best = graspnet_utils.select_best_grasp(result.grasps)
    print(
        "[Geometry] measured "
        f"size=[{geometry.length_m:.4f},{geometry.width_m:.4f},"
        f"{geometry.height_m:.4f}]m center_z={measured_center_z:.4f}m "
        f"z_offset={grasp_z_offset_m:.4f}m "
        f"base_x_offset={grasp_x_offset_m:.4f}m "
        f"base_y_offset={grasp_y_offset_m:.4f}m "
        f"command_z={center_base[2]:.4f}m horizontal=obb_center+base_offset"
    )
    return result.best is not None


def _pose_variants(
    grasp,
    T_cam2base,
    pre_offset,
    retreat_offset,
    insertion_depth,
    opening_axis_base=None,
    max_downward_tilt_deg=45.0,
    oblique_direction_base=None,
):
    """Prefer vertical OBB alignment before raw GraspNet orientations."""
    def vertical_waypoints(center_pose):
        center = tuple(float(value) for value in center_pose[0])
        pre = list(center)
        pre[2] += float(pre_offset)
        pre = tuple(pre)
        return center, pre, pre

    original = grasp_to_base_poses(
        grasp, T_cam2base, pre_offset, retreat_offset, insertion_depth
    )
    original_rotation = pose6d_to_mat4(*original[0])[:3, :3]
    wrist_flip = np.diag([1.0, -1.0, -1.0])
    downward = np.array([0.0, 0.0, -1.0], dtype=np.float64)
    opening = None if opening_axis_base is None else np.asarray(opening_axis_base, dtype=np.float64)

    def make_obb_rotation(tcp_x: np.ndarray) -> np.ndarray | None:
        if opening is None:
            return None
        tcp_x = np.asarray(tcp_x, dtype=np.float64)
        opening_projected = opening - float(np.dot(opening, tcp_x)) * tcp_x
        opening_norm = float(np.linalg.norm(opening_projected))
        if opening_norm <= 1e-9:
            return None
        # URDF gripper_joint axis is TCP +Z. Align OBB short axis to TCP Z.
        tcp_z = opening_projected / opening_norm
        tcp_y = np.cross(tcp_z, tcp_x)
        tcp_y /= max(float(np.linalg.norm(tcp_y)), 1e-9)
        return np.column_stack((tcp_x, tcp_y, tcp_z))

    top_rotation = make_obb_rotation(downward)
    oblique_rotation = None
    if oblique_direction_base is not None:
        direction = np.asarray(oblique_direction_base, dtype=np.float64)
        direction[2] = 0.0
        direction_norm = float(np.linalg.norm(direction))
        if direction_norm > 1e-9:
            direction /= direction_norm
            tilt = np.radians(float(max_downward_tilt_deg))
            oblique_tcp_x = np.cos(tilt) * downward + np.sin(tilt) * direction
            oblique_rotation = make_obb_rotation(oblique_tcp_x)
    top_down = constrain_rotation(
        original_rotation,
        OrientationConstraint(mode="top_down", max_tilt_deg=float(max_downward_tilt_deg)),
    )
    specs = []
    if oblique_rotation is not None:
        specs.extend([
            ("inward_oblique_obb", oblique_rotation),
            ("inward_oblique_obb_flip180", oblique_rotation @ wrist_flip),
        ])
    if top_rotation is not None:
        specs.extend([
            ("top_down_obb", top_rotation),
            ("top_down_obb_flip180", top_rotation @ wrist_flip),
        ])
    specs.extend(
        (
            ("raw", original_rotation),
            ("raw_flip180", original_rotation @ wrist_flip),
            ("top_down_keep_yaw", top_down),
            ("top_down_keep_yaw_flip180", top_down @ wrist_flip),
        )
    )
    variants = []
    for name, rotation in specs:
        pose_set = grasp_to_base_poses(
            grasp,
            T_cam2base,
            pre_offset,
            retreat_offset,
            insertion_depth,
            base_rotation_override=rotation,
        )
        variants.append((name, vertical_waypoints(pose_set)))
    return variants


def _rotation_within_downward_tilt(rotation: np.ndarray, max_tilt_deg: float) -> bool:
    """Accept only approaches within max tilt of base-link downward."""
    matrix = np.asarray(rotation, dtype=np.float64)
    if matrix.shape != (3, 3) or not np.all(np.isfinite(matrix)):
        raise ValueError("grasp rotation must be a finite 3x3 matrix")
    maximum = float(max_tilt_deg)
    if not np.isfinite(maximum) or not 0.0 <= maximum <= 90.0:
        raise ValueError("max downward grasp tilt must be within [0, 90] degrees")
    alignment = float(np.clip(-matrix[2, 0], -1.0, 1.0))
    tilt_deg = float(np.degrees(np.arccos(alignment)))
    return tilt_deg <= maximum + 1e-9


def _snapshot_feedback(frame, detections, selected, grasp, camera_K, status, target_class):
    """Freeze the pre-motion frame with detection, OBB and grasp feedback."""
    display = draw_detections_overlay(frame, detections, selected, target_class)
    if selected is not None and selected.mask is not None:
        try:
            obb = mask_obb(selected.mask)
            cx, cy = obb.center_px
            long_x, long_y = obb.long_axis_px
            half = 0.5 * obb.length_px
            p1 = (int(round(cx - long_x * half)), int(round(cy - long_y * half)))
            p2 = (int(round(cx + long_x * half)), int(round(cy + long_y * half)))
            cv2.line(display, p1, p2, (255, 0, 255), 3, cv2.LINE_AA)
            open_x, open_y = obb.opening_axis_px
            open_half = 0.5 * obb.width_px
            op1 = (int(round(cx - open_x * open_half)), int(round(cy - open_y * open_half)))
            op2 = (int(round(cx + open_x * open_half)), int(round(cy + open_y * open_half)))
            cv2.line(display, op1, op2, (255, 255, 0), 3, cv2.LINE_AA)
            cv2.circle(display, (int(round(cx)), int(round(cy))), 6, (0, 255, 255), -1)
            cv2.putText(display, f"OBB center=({cx:.0f},{cy:.0f}) OPEN axis", (10, 85), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 0), 2)
        except (TypeError, ValueError, cv2.error):
            cv2.putText(display, "OBB unavailable", (10, 85), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 165, 255), 2)
    if grasp is not None:
        x, y, z = np.asarray(grasp.translation, dtype=np.float64)
        if z > 1e-6:
            u = int(round(float(camera_K[0, 0]) * x / z + float(camera_K[0, 2])))
            v = int(round(float(camera_K[1, 1]) * y / z + float(camera_K[1, 2])))
            cv2.drawMarker(display, (u, v), (0, 0, 255), cv2.MARKER_CROSS, 28, 3)
            cv2.putText(display, f"GRASP ({u},{v})", (u + 12, v - 12), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)
    cv2.putText(display, status, (10, 55), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0) if "SUCCESS" in status else (0, 80, 255), 2)
    return display


def _save_snapshot(frame: np.ndarray, cfg: dict[str, Any], status: str) -> None:
    """Persist the latest grasp feedback, including failed attempts."""
    snapshot_dir = Path(str((cfg.get("graspnet") or {}).get(
        "snapshot_dir", "logs/grasp_snapshots"
    )))
    if not snapshot_dir.is_absolute():
        snapshot_dir = PROJECT_ROOT / snapshot_dir
    snapshot_dir.mkdir(parents=True, exist_ok=True)
    label = re.sub(r"[^A-Za-z0-9_-]+", "_", status).strip("_") or "snapshot"
    filename = f"{time.strftime('%Y%m%d_%H%M%S')}_{time.time_ns() % 1_000_000:06d}_{label}.png"
    path = snapshot_dir / filename
    if not cv2.imwrite(str(path), np.asarray(frame)):
        print(f"[Snapshot] save failed: {path}")
        return
    print(f"[Snapshot] saved: {path}")


def _choose_candidate(
    result: Any,
    robot: OfficialSdkRobot,
    T_cam2base: np.ndarray,
    *,
    min_z: float,
    pre_offset: float,
    retreat_offset: float,
    insertion_depth: float,
    config: dict[str, Any],
    collision_model: UrdfGripperCollisionModel | None = None,
) -> tuple[Any, tuple[tuple[float, ...], ...], list[np.ndarray]] | None:
    official_cfg = config.get("official_sdk") or {}
    robot_cfg = config.get("robot") or {}
    safety_cfg = config.get("safety") or {}
    grasp_cfg = (config.get("grasp_pipeline") or {}).get("grasp") or {}
    max_downward_tilt_deg = float(
        safety_cfg.get("max_downward_grasp_tilt_deg", 45.0)
    )
    oblique_first_radius_m = float(grasp_cfg.get("oblique_first_radius_m", 0.43))
    far_candidate_attempt_limit = max(
        1,
        int(grasp_cfg.get("far_candidate_attempt_limit", 8)),
    )
    start_joints = robot.current_joints()
    for rank, grasp in enumerate(_sorted_grasps(result.grasps), start=1):
        grasp_base = transform_points(
            np.asarray(grasp.translation, dtype=np.float64).reshape(1, 3),
            T_cam2base,
        )[0]
        candidate_max_tilt = max_downward_tilt_deg
        print(
            f"[IK] target radius={np.linalg.norm(grasp_base[:2]):.3f}m "
            f"max_tilt={candidate_max_tilt:.1f}deg"
        )
        target_radius = float(np.linalg.norm(grasp_base[:2]))
        oblique_direction = None
        if target_radius >= oblique_first_radius_m:
            oblique_direction = -np.asarray(grasp_base[:3], dtype=np.float64)
            oblique_direction[2] = 0.0
            print(
                f"[IK] far target: try inward oblique before top-down "
                f"threshold={oblique_first_radius_m:.3f}m"
            )
        pre_offset_candidates = (
            _far_pregrasp_offset_candidates(config, pre_offset)
            if oblique_direction is not None
            else _pregrasp_offset_candidates(config, pre_offset)
        )
        variant_index = 0
        for candidate_pre_offset in pre_offset_candidates:
            for variant_name, (grasp_pose, pre_pose, _retreat_pose) in _pose_variants(
                grasp,
                T_cam2base,
                candidate_pre_offset,
                retreat_offset,
                insertion_depth,
                getattr(result, "obb_opening_axis_base", None),
                candidate_max_tilt,
                oblique_direction,
            ):
                variant_index += 1
                if oblique_direction is not None and variant_index > far_candidate_attempt_limit:
                    print(
                        f"[IK] far candidate attempt limit reached "
                        f"({far_candidate_attempt_limit}); skip remaining variants"
                    )
                    break
                # Retrace the verified approach after closing the gripper.
                retreat_pose = pre_pose
                poses = (pre_pose, grasp_pose, retreat_pose)
                z_values = [float(pose[2]) for pose in poses]
                if any(z < min_z for z in z_values):
                    continue
                if not _rotation_within_downward_tilt(
                    pose6d_to_mat4(*grasp_pose)[:3, :3], candidate_max_tilt
                ):
                    print(
                        f"[IK] candidate {rank}.{variant_index} ({variant_name}) "
                        f"rejected: side approach exceeds {candidate_max_tilt:.1f}deg"
                    )
                    continue
                collision_clearance = None
                try:
                    approach_solutions = robot.kinematics.solve_pose_sequence(
                        [np.asarray(pre_pose), np.asarray(grasp_pose)],
                        start_joints,
                    )
                    solutions = [
                        *approach_solutions,
                        np.asarray(approach_solutions[0], dtype=np.float64).copy(),
                    ]
                    solutions = _validated_solutions_with_transit(
                        poses,
                        solutions,
                        start_joints=start_joints,
                        jaw_width_m=float(grasp.width),
                        joint_limits=robot.kinematics.joint_limits,
                        workspace=official_cfg.get("workspace") or {
                            "x": [0.10, 0.55], "y": [-0.35, 0.35], "z": [0.03, 0.60]
                        },
                        min_tcp_z_m=float(official_cfg.get("min_tcp_z_m", 0.03)),
                        max_segment_joint_delta_rad=float(official_cfg.get("max_segment_joint_delta_rad", 1.0)),
                        gripper_max_width_m=float(robot_cfg.get("gripper_max_width_m", 0.070)),
                    )
                    transit_count = len(solutions) - 3
                    if transit_count > 0:
                        print(
                            f"[IK] inserted {transit_count} joint transit waypoint(s) "
                            "before pregrasp"
                        )
                    if collision_model is not None:
                        safety_cfg = config.get("safety") or {}
                        collision = collision_model.check_swept_path(
                            [
                                pose6d_to_mat4(*pre_pose),
                                pose6d_to_mat4(*grasp_pose),
                                pose6d_to_mat4(*retreat_pose),
                            ],
                            obstacle_points_base=np.empty((0, 3), dtype=np.float64),
                            table_z_m=float(safety_cfg.get("table_z_m", -0.007)),
                            # Hard floor stays conservative, while the preferred
                            # margin is reported separately instead of rejecting
                            # every candidate with a 14 mm mesh margin.
                            table_clearance_m=float(safety_cfg.get("table_clearance_m", 0.005)),
                            obstacle_clearance_m=0.0,
                            linear_step_m=float(safety_cfg.get("collision_check_step_m", 0.005)),
                            angular_step_rad=0.05,
                        )
                        if not collision.safe:
                            raise MotionGateError(
                                f"table collision: clearance={collision.min_table_clearance_m:.4f}m"
                            )
                        collision_clearance = float(collision.min_table_clearance_m)
                except (MotionGateError, RuntimeError, ValueError) as exc:
                    print(
                        f"[IK] candidate {rank}.{variant_index} "
                        f"({variant_name}, pre_offset={candidate_pre_offset:.3f}m) rejected: {exc}"
                    )
                    continue
                print(
                    f"[IK] candidate {rank}.{variant_index} accepted "
                    f"({variant_name}) score={float(grasp.score):.4f} "
                    f"pre_offset={candidate_pre_offset:.3f}m z={z_values}"
                )
                accepted_rotation = pose6d_to_mat4(*grasp_pose)[:3, :3]
                accepted_tilt = float(
                    np.degrees(
                        np.arccos(
                            np.clip(-float(accepted_rotation[2, 0]), -1.0, 1.0)
                        )
                    )
                )
                print(f"[IK] accepted approach tilt={accepted_tilt:.1f}deg")
                if collision_clearance is not None:
                    preferred = float(
                        (config.get("safety") or {}).get(
                            "table_preferred_clearance_m", 0.014
                        )
                    )
                    state = "preferred" if collision_clearance >= preferred else "near-table"
                    print(
                        f"[Safety] {state}: min gripper-table clearance="
                        f"{collision_clearance:.4f}m preferred={preferred:.4f}m"
                    )
                return grasp, poses, solutions
            if oblique_direction is not None and variant_index >= far_candidate_attempt_limit:
                break
    return None


def _project_base_point_to_pixel(
    point_base: np.ndarray,
    T_cam2base: np.ndarray,
    camera_K: np.ndarray,
) -> tuple[float, float]:
    point_camera = transform_points(
        np.asarray(point_base, dtype=np.float64).reshape(1, 3),
        np.linalg.inv(T_cam2base),
    )[0]
    if point_camera[2] <= 1e-6:
        raise MotionGateError("planned grasp center is behind camera")
    return (
        float(camera_K[0, 0] * point_camera[0] / point_camera[2] + camera_K[0, 2]),
        float(camera_K[1, 1] * point_camera[1] / point_camera[2] + camera_K[1, 2]),
    )


def _target_ray_at_base_z(
    center_px: tuple[float, float],
    base_z_m: float,
    T_cam2base: np.ndarray,
    camera_K: np.ndarray,
) -> np.ndarray:
    ray_camera = np.array(
        [
            (float(center_px[0]) - float(camera_K[0, 2])) / float(camera_K[0, 0]),
            (float(center_px[1]) - float(camera_K[1, 2])) / float(camera_K[1, 1]),
            1.0,
        ],
        dtype=np.float64,
    )
    origin_base = np.asarray(T_cam2base[:3, 3], dtype=np.float64)
    ray_base = np.asarray(T_cam2base[:3, :3], dtype=np.float64) @ ray_camera
    if abs(float(ray_base[2])) < 1e-9:
        raise MotionGateError("target ray is parallel to object height plane")
    scale = (float(base_z_m) - float(origin_base[2])) / float(ray_base[2])
    if not np.isfinite(scale) or scale <= 0.0:
        raise MotionGateError("target ray does not intersect object height plane")
    return origin_base + scale * ray_base


def _visual_align_at_pregrasp(
    robot: OfficialSdkRobot,
    camera: Any,
    yolo: Any,
    yolo_opts: dict[str, Any],
    target_class: str,
    grasp: Any,
    poses: tuple[tuple[float, ...], ...],
    cfg: dict[str, Any],
    T_hand_eye: np.ndarray,
    hand_eye_reference_frame: str,
    collision_model: UrdfGripperCollisionModel | None,
) -> tuple[tuple[tuple[float, ...], ...], list[np.ndarray]]:
    """Take one pregrasp frame and apply one horizontal correction."""
    settle_s = float(
        ((cfg.get("grasp_pipeline") or {}).get("grasp") or {}).get(
            "visual_alignment_settle_s", 0.5
        )
    )
    if settle_s > 0.0:
        time.sleep(settle_s)
    color, depth = camera.get_frame()
    if color is None or depth is None:
        raise MotionGateError("pregrasp visual alignment frame unavailable")
    _, detections = detect_objects(yolo, color, yolo_opts)
    selected = select_target(detections, target_class)
    grasp_cfg = (cfg.get("grasp_pipeline") or {}).get("grasp") or {}
    minimum_confidence = float(
        grasp_cfg.get("visual_alignment_min_confidence", 0.30)
    )
    if selected is None or float(getattr(selected, "conf", 0.0)) < minimum_confidence:
        raise MotionGateError("pregrasp visual alignment target unavailable")
    mask = graspnet_utils.build_target_sample_mask(
        depth.shape, selected, margin_px=0, expand_ratio=1.0
    )
    try:
        obb = mask_obb(mask)
    except (TypeError, ValueError, cv2.error) as exc:
        raise MotionGateError(f"pregrasp target OBB unavailable: {exc}") from exc
    u = int(round(obb.center_px[0]))
    v = int(round(obb.center_px[1]))
    radius = 5
    y1, y2 = max(0, v - radius), min(depth.shape[0], v + radius + 1)
    x1, x2 = max(0, u - radius), min(depth.shape[1], u + radius + 1)
    local_depth = np.asarray(depth[y1:y2, x1:x2], dtype=np.float64)
    local_mask = mask[y1:y2, x1:x2] > 0
    valid_depth = local_mask & np.isfinite(local_depth) & (local_depth > 0.0)
    if not np.any(valid_depth):
        raise MotionGateError("pregrasp target center depth unavailable")

    camera_K = camera.K.astype(np.float64)
    current_q = robot.current_joints()
    T_cam2base = camera_to_base_from_hand_eye_reference(
        robot.kinematics, current_q, T_hand_eye, cfg, hand_eye_reference_frame
    )
    # Compare the fresh object center against the final grasp center, while
    # the robot camera is physically parked at the high pregrasp pose.
    planned_base = np.asarray(poses[1], dtype=np.float64)[:3]
    planned_px = _project_base_point_to_pixel(planned_base, T_cam2base, camera_K)
    pixel_error = np.asarray(obb.center_px) - np.asarray(planned_px)
    max_pixel_error = float(
        grasp_cfg.get("visual_alignment_max_pixel_error_px", 120.0)
    )
    if float(np.linalg.norm(pixel_error)) > max_pixel_error:
        raise MotionGateError(
            f"pregrasp visual error={np.linalg.norm(pixel_error):.1f}px exceeds "
            f"limit={max_pixel_error:.1f}px"
        )
    object_base = _target_ray_at_base_z(
        obb.center_px, planned_base[2], T_cam2base, camera_K
    )
    delta_xy = object_base[:2] - planned_base[:2]
    max_correction = float(
        grasp_cfg.get("visual_alignment_max_correction_m", 0.040)
    )
    correction_norm = float(np.linalg.norm(delta_xy))
    if correction_norm > max_correction:
        raise MotionGateError(
            f"pregrasp correction={correction_norm * 1000.0:.1f}mm exceeds "
            f"limit={max_correction * 1000.0:.1f}mm"
        )
    corrected_poses = []
    for pose in poses:
        corrected = list(float(value) for value in pose)
        corrected[0] += float(delta_xy[0])
        corrected[1] += float(delta_xy[1])
        corrected_poses.append(tuple(corrected))
    corrected_poses_tuple = tuple(corrected_poses)
    corrected_solutions = robot.kinematics.solve_pose_sequence(
        [np.asarray(corrected_poses_tuple[0]), np.asarray(corrected_poses_tuple[1])],
        current_q,
    )
    corrected_joints = [
        *corrected_solutions,
        np.asarray(corrected_solutions[0], dtype=np.float64).copy(),
    ]
    official_cfg = cfg.get("official_sdk") or {}
    robot_cfg = cfg.get("robot") or {}
    validate_execution_targets(
        [np.asarray(pose) for pose in corrected_poses_tuple], corrected_joints,
        jaw_width_m=float(grasp.width),
        joint_limits=robot.kinematics.joint_limits,
        workspace=official_cfg.get("workspace") or {
            "x": [0.10, 0.55], "y": [-0.35, 0.35], "z": [0.03, 0.60]
        },
        min_tcp_z_m=float(official_cfg.get("min_tcp_z_m", 0.03)),
        max_segment_joint_delta_rad=float(
            official_cfg.get("max_segment_joint_delta_rad", 1.0)
        ),
        gripper_max_width_m=float(robot_cfg.get("gripper_max_width_m", 0.070)),
        start_joints=current_q,
    )
    if collision_model is not None:
        safety_cfg = cfg.get("safety") or {}
        collision = collision_model.check_swept_path(
            [pose6d_to_mat4(*pose) for pose in corrected_poses_tuple],
            obstacle_points_base=np.empty((0, 3), dtype=np.float64),
            table_z_m=float(safety_cfg.get("table_z_m", -0.007)),
            table_clearance_m=float(safety_cfg.get("table_clearance_m", 0.005)),
            obstacle_clearance_m=0.0,
            linear_step_m=float(safety_cfg.get("collision_check_step_m", 0.005)),
            angular_step_rad=0.05,
        )
        if not collision.safe:
            raise MotionGateError(
                f"corrected path table collision: clearance={collision.min_table_clearance_m:.4f}m"
            )
    alignment_display = draw_detections_overlay(
        color, detections, selected, target_class
    )
    object_px = (int(round(obb.center_px[0])), int(round(obb.center_px[1])))
    planned_px_int = (int(round(planned_px[0])), int(round(planned_px[1])))
    cv2.circle(alignment_display, object_px, 7, (0, 255, 255), -1)
    cv2.drawMarker(
        alignment_display, planned_px_int, (0, 0, 255), cv2.MARKER_CROSS, 24, 2
    )
    cv2.arrowedLine(
        alignment_display, planned_px_int, object_px, (255, 0, 255), 2, tipLength=0.2
    )
    cv2.putText(
        alignment_display,
        f"ALIGN dx={delta_xy[0] * 1000.0:.1f} dy={delta_xy[1] * 1000.0:.1f}mm",
        (10, 115),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.6,
        (255, 0, 255),
        2,
    )
    _save_snapshot(alignment_display, cfg, "pregrasp_alignment")
    print(
        f"[Align] object_px=({obb.center_px[0]:.1f},{obb.center_px[1]:.1f}) "
        f"planned_px=({planned_px[0]:.1f},{planned_px[1]:.1f}) "
        f"error_px=({pixel_error[0]:.1f},{pixel_error[1]:.1f}) "
        f"delta_xy=({delta_xy[0] * 1000.0:.1f},{delta_xy[1] * 1000.0:.1f})mm"
    )
    return corrected_poses_tuple, corrected_joints


def _execute_candidate(
    robot: OfficialSdkRobot,
    poses: tuple[tuple[float, ...], ...],
    solutions: list[np.ndarray],
    cfg: dict[str, Any],
) -> bool:
    official_cfg = cfg.get("official_sdk") or {}
    grasp_duration = float(official_cfg.get("segment_duration_s", 2.0))
    if grasp_duration <= 0.0:
        grasp_duration = 2.0
    transit_duration = float(
        official_cfg.get("transit_duration_s", grasp_duration)
    )
    if transit_duration <= 0.0:
        transit_duration = grasp_duration
    robot.open_gripper()
    transit_count = max(0, len(solutions) - 3)
    for index, transit_target in enumerate(solutions[:transit_count], start=1):
        print(f"[Grasp] move transit {index}/{transit_count}")
        robot.move_joints_and_wait(np.asarray(transit_target), transit_duration)
    grasp_solutions = solutions[transit_count:]
    if len(grasp_solutions) != 3:
        raise MotionGateError("candidate must contain pregrasp, grasp, and retreat joints")
    print("[Grasp] move pregrasp")
    robot.move_to_traj_and_wait(
        np.asarray(poses[0]), grasp_solutions[0], transit_duration
    )

    place = cfg.get("place") or {}
    wait_before = max(0.0, float(place.get("pre_grasp_wait_s", 1.0)))
    print(f"[Grasp] wait before closing {wait_before:.1f}s")
    time.sleep(wait_before)
    print("[Grasp] move grasp")
    robot.move_to_traj_and_wait(
        np.asarray(poses[1]), grasp_solutions[1], grasp_duration
    )
    holding = robot.grasp()
    print("[Grasp] holding object" if holding else "[Grasp] empty grasp")
    post_grasp_wait = max(0.0, float(place.get("post_grasp_wait_s", 1.0)))
    print(f"[Grasp] wait for hold confirmation {post_grasp_wait:.1f}s")
    time.sleep(post_grasp_wait)
    if not holding:
        robot.release()
        robot.move_ready()
        return False
    print("[Grasp] move retreat")
    robot.move_to_traj_and_wait(
        np.asarray(poses[2]), grasp_solutions[2], transit_duration
    )

    target = np.asarray(place.get("joint_target_rad", ()), dtype=np.float64)
    if target.size != 6:
        raise ValueError("place.joint_target_rad must contain six values")
    print("[Place] move fixed joint target")
    robot.move_joints_and_wait(target, float(place.get("joint_duration_s", 4.0)))
    time.sleep(float(place.get("pre_release_wait_s", 1.0)))
    robot.release()
    time.sleep(float(place.get("post_release_wait_s", 1.0)))
    print("[Place] return ready")
    robot.move_joints_and_wait(
        robot.ready_joints(), float(place.get("return_duration_s", 4.0))
    )
    print("[Idle] gripper released; robot is at ready pose, waiting for next G command")
    return True


def main() -> int:
    args = _args()
    cfg = load_config(args.config)
    cfg = configure_camera(cfg, args)
    target_class = _target_class(cfg, args)
    min_z, pre_offset, retreat_offset, insertion_depth, collision_thresh, voxel = _simple_cfg(cfg)
    gp_cfg = cfg.get("graspnet") or {}
    checkpoint = args.checkpoint or gp_cfg.get("checkpoint", "checkpoint-rs.tar")
    checkpoint_path = resolve_checkpoint_path(str(checkpoint), project_root=PROJECT_ROOT)
    camera = make_camera(cfg)
    robot = OfficialSdkRobot(cfg)
    collision_model = None
    yolo = None
    faulted = False
    normal_shutdown = False
    try:
        hand_eye_reference_frame = str(
            (cfg.get("robot") or {}).get("end_effector_frame", "")
        ).strip()
        if not hand_eye_reference_frame:
            raise RuntimeError("robot.end_effector_frame is required for hand-eye FK")
        T_hand_eye, mode = load_hand_eye(
            PROJECT_ROOT,
            str(cfg["camera"]["type"]).lower(),
            expected_reference_frame=hand_eye_reference_frame,
        )
        if T_hand_eye is None or mode != "eye_in_hand":
            raise RuntimeError("eye-in-hand calibration required")
        yolo, yolo_opts = load_yolo(
            cfg,
            project_root=PROJECT_ROOT,
            extra_classes=[target_class],
        )
        net = build_net(checkpoint_path, num_view=int(gp_cfg.get("num_view", 300)))
        # Load every fallible perception asset before enabling or moving the
        # arm. A missing model must never leave the robot suddenly disabled at
        # the ready pose.
        from utils.calibration_identity import validate_calibration_identity
        camera.open()
        validate_calibration_identity(PROJECT_ROOT, cfg, camera.serial)
        robot.connect()
        safety_cfg = cfg.get("safety") or {}
        if bool(safety_cfg.get("table_collision_enabled", True)):
            robot_cfg = cfg.get("robot") or {}
            urdf_path = Path(str(robot_cfg.get("urdf_path", "")))
            if not urdf_path.is_absolute():
                urdf_path = (PROJECT_ROOT / urdf_path).resolve()
            collision_model = UrdfGripperCollisionModel(
                urdf_path,
                open_width_m=float(robot_cfg.get("gripper_max_width_m", 0.070)),
                end_frame="grasp_tcp",
            )
            print("[Safety] MoveIt worktable geometry mirrored by local URDF swept-path check")
        robot.move_ready()
        camera.warm_up(20)
        window = "Simple Direct SDK Grasp"
        cv2.namedWindow(window, cv2.WINDOW_NORMAL)
        frozen = False
        last_display = None
        live_detections = []
        live_selected = None
        target_miss_count = 0
        target_miss_hold = 5
        frame_index = 0
        while True:
            color, depth = camera.get_frame()
            if color is None or depth is None:
                continue
            frame_index += 1
            display = color
            if frame_index % max(1, int(yolo_opts.get("infer_every", 3))) == 0 and not frozen:
                _, new_detections = detect_objects(yolo, color, yolo_opts)
                new_selected = select_target(new_detections, target_class)
                if new_selected is not None:
                    live_detections = new_detections
                    live_selected = new_selected
                    target_miss_count = 0
                else:
                    target_miss_count += 1
                    if target_miss_count >= target_miss_hold:
                        live_detections = new_detections
                        live_selected = None
            if not frozen:
                display = draw_detections_overlay(
                    color,
                    live_detections,
                    live_selected,
                    target_class,
                )
                cv2.putText(
                    display,
                    target_status_text(
                        live_selected,
                        live_detections,
                        target_class,
                    ),
                    (10, 28),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.65,
                    (255, 255, 255),
                    2,
                )
            if frozen and last_display is not None:
                display = last_display.copy()
                cv2.putText(display, "[FROZEN] G=grasp R=resume Q=quit", (10, 55), cv2.FONT_HERSHEY_SIMPLEX, 0.65, (0, 215, 255), 2)
            cv2.imshow(window, display)
            key = cv2.waitKey(1) & 0xFF
            if key in (ord("q"), ord("Q"), 27):
                break
            if key in (ord("r"), ord("R")):
                frozen = False
                last_display = None
                continue
            if key not in (ord("g"), ord("G")):
                continue
            frozen = True
            color, depth = camera.get_frame()
            selected_target_override = live_selected
            if selected_target_override is not None:
                print(
                    "[G] reuse live target: "
                    f"{selected_target_override.class_name} "
                    f"{selected_target_override.conf:.2f}"
                )
            result = infer_frame(
                net,
                color,
                depth,
                camera.K.astype(np.float64),
                num_point=int(gp_cfg.get("num_point", 20000)),
                min_depth=float(gp_cfg.get("min_depth", 0.05)),
                max_depth=float(gp_cfg.get("max_depth", 1.0)),
                collision_thresh=collision_thresh,
                voxel_size=voxel,
                yolo_model=yolo,
                yolo_opts=yolo_opts,
                target_class=target_class,
                target_margin_px=int(gp_cfg.get("target_margin_px", 12)),
                target_expand_ratio=float(gp_cfg.get("target_expand_ratio", 1.0)),
                target_depth_tolerance_m=float(gp_cfg.get("target_depth_tolerance_m", 0.040)),
                target_min_edge_distance_px=float(gp_cfg.get("target_min_edge_distance_px", 6.0)),
                target_center_score_weight=float(gp_cfg.get("target_center_score_weight", 0.20)),
                target_depth_window_radius_px=int(gp_cfg.get("target_depth_window_radius_px", 4)),
                target_min_mask_support=float(gp_cfg.get("target_min_mask_support", 0.80)),
                target_min_valid_depth_fraction=float(gp_cfg.get("target_min_valid_depth_fraction", 0.80)),
                target_max_local_depth_deviation_m=float(gp_cfg.get("target_max_local_depth_deviation_m", 0.005)),
                target_max_center_depth_error_m=float(gp_cfg.get("target_max_center_depth_error_m", 0.015)),
                max_grasp_width_m=float((cfg.get("robot") or {}).get("gripper_max_width_m", 0.07)),
                selected_target_override=selected_target_override,
            )
            if result.best is None:
                print(f"[G] no valid target/grasp: {result.status}")
                last_display = _snapshot_feedback(
                    color, live_detections, live_selected, None,
                    camera.K.astype(np.float64),
                    f"GRASP FAILED: {result.status}", target_class
                )
                _save_snapshot(last_display, cfg, "failed_no_grasp")
                frozen = True
                continue
            q_capture = robot.current_joints()
            T_cam2base = camera_to_base_from_hand_eye_reference(
                robot.kinematics,
                q_capture,
                T_hand_eye,
                cfg,
                hand_eye_reference_frame,
            )
            if not _anchor_grasps_to_measured_center(
                result,
                depth,
                camera.K.astype(np.float64),
                T_cam2base,
                cfg,
                min_z=min_z,
            ):
                print("[G] no reliable measured object center")
                last_display = _snapshot_feedback(
                    color, live_detections, live_selected, None,
                    camera.K.astype(np.float64),
                    "GRASP FAILED: no reliable center", target_class
                )
                _save_snapshot(last_display, cfg, "failed_no_center")
                frozen = True
                continue
            candidate = _choose_candidate(
                result,
                robot,
                T_cam2base,
                min_z=min_z,
                pre_offset=pre_offset,
                retreat_offset=retreat_offset,
                insertion_depth=insertion_depth,
                config=cfg,
                collision_model=collision_model,
            )
            if candidate is None:
                print("[G] no candidate passed Z floor and three-pose IK")
                last_display = _snapshot_feedback(
                    color,
                    live_detections,
                    live_selected,
                    None,
                    camera.K.astype(np.float64),
                    "GRASP FAILED: no executable candidate",
                    target_class,
                )
                _save_snapshot(last_display, cfg, "failed_no_ik")
                frozen = True
                continue
            selected_grasp, poses, solutions = candidate
            snapshot = _snapshot_feedback(
                color,
                live_detections,
                live_selected,
                selected_grasp,
                camera.K.astype(np.float64),
                "GRASP PLANNED",
                target_class,
            )
            try:
                executed = _execute_candidate(
                    robot,
                    poses,
                    solutions,
                    cfg,
                )
            except MotionGateError as exc:
                print(f"[Align] failed; aborting grasp: {exc}")
                last_display = _snapshot_feedback(
                    snapshot, live_detections, live_selected, selected_grasp,
                    camera.K.astype(np.float64), f"GRASP FAILED: {exc}", target_class
                )
                _save_snapshot(last_display, cfg, "failed_alignment")
                try:
                    robot.release()
                    robot.move_ready()
                except Exception as recovery_exc:
                    print(f"[ERROR] alignment recovery failed: {recovery_exc}")
                    faulted = True
                    robot.hold_current()
                    return 1
                frozen = True
                continue
            except Exception as exc:
                print(f"[ERROR] grasp route failed: {exc}")
                last_display = _snapshot_feedback(
                    snapshot, live_detections, live_selected, None,
                    camera.K.astype(np.float64), f"GRASP FAILED: {exc}", target_class
                )
                _save_snapshot(last_display, cfg, "failed_execution")
                frozen = True
                faulted = True
                try:
                    robot.hold_current()
                except Exception as recovery_exc:
                    print(f"[ERROR] hold failed; disconnecting without more motion: {recovery_exc}")
                print("[ERROR] fault latched; operator recovery is required")
                return 1
            last_display = _snapshot_feedback(
                color,
                live_detections,
                live_selected,
                selected_grasp,
                camera.K.astype(np.float64),
                "GRASP SUCCESS" if executed else "EMPTY GRASP",
                target_class,
            )
            # Keep the completed snapshot visible.  R resumes live view;
            # G can still start the next grasp cycle while this frame stays.
            frozen = True
            _save_snapshot(last_display, cfg, "success" if executed else "empty_grasp")
            if executed:
                print("[Idle] waiting for next grasp command (G)")
        normal_shutdown = True
        return 0
    finally:
        try:
            camera.close()
        except Exception:
            pass
        cv2.destroyAllWindows()
        if normal_shutdown and not faulted:
            try:
                if robot.control_loop_active:
                    zero_duration = float(
                        (cfg.get("official_sdk") or {}).get(
                            "zero_pose_duration_s", 8.0
                        )
                    )
                    print("[Exit] Q received; return to joint zero pose")
                    robot.move_joints_and_wait(
                        np.zeros(6, dtype=np.float64), zero_duration
                    )
                    print("[Exit] joint zero pose reached; hold 1.0s")
                    time.sleep(1.0)
                    robot.release()
                    print("[Exit] motors disabled")
            except Exception as exc:
                print(f"[Exit] normal shutdown motion failed: {exc}")
        robot.close()


if __name__ == "__main__":
    raise SystemExit(main())
