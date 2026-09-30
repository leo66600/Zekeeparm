"""Lightweight helpers for the official YOLO OBB grasp route."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Optional

import cv2
import numpy as np

from .orientation_constraints import (
    OrientationConstraint,
    constrain_rotation,
)
from .transforms import (
    graspnet_rotation_to_rebot_tcp_rotation,
    mat4_to_pose6d,
    pose6d_to_mat4,
    transform_grasp_pose_to_base_with_retreat,
)
from .yolo_utils import YoloDetection


class Grasp:
    def __init__(
        self,
        score: float,
        width: float,
        height: float,
        depth: float,
        rotation_matrix: np.ndarray,
        translation: np.ndarray,
        object_id: int,
    ) -> None:
        self.score = float(score)
        self.width = float(width)
        self.height = float(height)
        self.depth = float(depth)
        self.rotation_matrix = np.asarray(
            rotation_matrix,
            dtype=np.float64,
        ).reshape(3, 3)
        self.translation = np.asarray(
            translation,
            dtype=np.float64,
        ).reshape(3)
        self.object_id = int(object_id)

    @classmethod
    def from_array(cls, values: np.ndarray) -> "Grasp":
        row = np.asarray(values, dtype=np.float64).reshape(17)
        return cls(
            row[0],
            row[1],
            row[2],
            row[3],
            row[4:13].reshape(3, 3),
            row[13:16],
            int(row[16]),
        )

    def to_array(self) -> np.ndarray:
        return np.concatenate(
            [
                np.asarray(
                    [self.score, self.width, self.height, self.depth],
                    dtype=np.float64,
                ),
                self.rotation_matrix.reshape(-1),
                self.translation,
                np.asarray([self.object_id], dtype=np.float64),
            ]
        )


class GraspGroup:
    def __init__(self, values: Optional[np.ndarray] = None) -> None:
        if values is None:
            self.grasp_group_array = np.empty((0, 17), dtype=np.float64)
        else:
            array = np.asarray(values, dtype=np.float64)
            self.grasp_group_array = array.reshape(-1, 17).copy()

    def __len__(self) -> int:
        return len(self.grasp_group_array)

    def __getitem__(self, index):
        selected = self.grasp_group_array[index]
        if np.asarray(selected).ndim == 1:
            return Grasp.from_array(selected)
        return GraspGroup(selected)

    def add(self, grasp: Grasp) -> None:
        self.grasp_group_array = np.vstack(
            [self.grasp_group_array, grasp.to_array()]
        )

    def sort_by_score(self) -> None:
        if len(self):
            order = np.argsort(-self.grasp_group_array[:, 0])
            self.grasp_group_array = self.grasp_group_array[order]

    def nms(
        self,
        translation_thresh: float = 0.03,
        rotation_thresh: float = np.deg2rad(30.0),
    ) -> "GraspGroup":
        if len(self) <= 1:
            return GraspGroup(self.grasp_group_array)
        order = np.argsort(-self.grasp_group_array[:, 0])
        kept: list[int] = []
        for index in order:
            candidate = Grasp.from_array(self.grasp_group_array[index])
            duplicate = False
            for kept_index in kept:
                existing = Grasp.from_array(
                    self.grasp_group_array[kept_index]
                )
                translation_distance = float(
                    np.linalg.norm(
                        candidate.translation - existing.translation
                    )
                )
                relative = (
                    candidate.rotation_matrix.T
                    @ existing.rotation_matrix
                )
                cosine = float(
                    np.clip((np.trace(relative) - 1.0) * 0.5, -1.0, 1.0)
                )
                rotation_distance = float(np.arccos(cosine))
                if (
                    translation_distance <= float(translation_thresh)
                    and rotation_distance <= float(rotation_thresh)
                ):
                    duplicate = True
                    break
            if not duplicate:
                kept.append(int(index))
        return GraspGroup(self.grasp_group_array[kept])


@dataclass(frozen=True)
class OBBGeometry:
    """Mask OBB geometry used by direct SDK route."""

    center_px: tuple[float, float]
    opening_axis_px: tuple[float, float]
    long_axis_px: tuple[float, float]
    length_px: float
    width_px: float


def mask_obb(mask: np.ndarray) -> OBBGeometry:
    """Return minimum-area rectangle and gripper opening direction.

    Opening direction is OBB short axis. Near-square masks remain valid, but
    caller must treat their yaw as rotationally symmetric.
    """
    binary = np.asarray(mask, dtype=np.uint8)
    if binary.ndim != 2 or not np.any(binary):
        raise ValueError("mask must be a non-empty 2D array")
    points = np.column_stack(np.nonzero(binary > 0))[:, ::-1].astype(np.float32)
    rectangle = cv2.minAreaRect(points.reshape(-1, 1, 2))
    corners = cv2.boxPoints(rectangle).astype(np.float64)
    edges = np.roll(corners, -1, axis=0) - corners
    lengths = np.linalg.norm(edges, axis=1)
    long_index = int(np.argmax(lengths))
    long_axis = edges[long_index] / lengths[long_index]
    short_axis = np.array([-long_axis[1], long_axis[0]], dtype=np.float64)
    short_length = float(min(rectangle[1]))
    long_length = float(max(rectangle[1]))
    center = tuple(float(value) for value in rectangle[0])
    return OBBGeometry(
        center_px=center,
        opening_axis_px=(float(short_axis[0]), float(short_axis[1])),
        long_axis_px=(float(long_axis[0]), float(long_axis[1])),
        length_px=long_length,
        width_px=short_length,
    )


def select_detection_at_point(
    detections: list[YoloDetection],
    point_xy: tuple[int, int],
    target_class: Optional[str] = None,
) -> Optional[YoloDetection]:
    """Select top-confidence mask containing click; overlap resolved by mask."""
    x, y = int(point_xy[0]), int(point_xy[1])
    candidates = []
    target_norm = _normalized_class_name(target_class) if target_class else None
    for detection in detections:
        if target_norm and target_norm not in _normalized_class_name(detection.class_name):
            continue
        x1, y1, x2, y2 = detection.bbox_xyxy
        if not (x1 <= x <= x2 and y1 <= y <= y2):
            continue
        if detection.mask is not None:
            mask = np.asarray(detection.mask)
            if 0 <= y < mask.shape[0] and 0 <= x < mask.shape[1] and not mask[y, x]:
                continue
        candidates.append(detection)
    return max(candidates, key=lambda item: item.conf) if candidates else None


def _normalized_class_name(name: str) -> str:
    return "".join(
        character
        for character in str(name).casefold()
        if character.isalnum()
    )


def select_target(
    detections: list[YoloDetection],
    target_class: Optional[str],
) -> Optional[YoloDetection]:
    if not detections:
        return None
    candidates = detections
    if target_class:
        target_norm = _normalized_class_name(target_class)
        exact = [
            detection
            for detection in detections
            if _normalized_class_name(detection.class_name) == target_norm
        ]
        contains = [
            detection
            for detection in detections
            if target_norm
            in _normalized_class_name(detection.class_name)
        ]
        candidates = exact or contains
    return (
        max(candidates, key=lambda detection: detection.conf)
        if candidates
        else None
    )


def target_status_text(
    selected: Optional[YoloDetection],
    detections: list[YoloDetection],
    target_class: Optional[str],
) -> str:
    if selected is not None:
        return (
            f"target={selected.class_name} {selected.conf:.2f} "
            f"detections={len(detections)}"
        )
    if target_class:
        return (
            f"target={target_class} not found "
            f"detections={len(detections)}"
        )
    return f"target not found detections={len(detections)}"


def draw_detections_overlay(
    frame: np.ndarray,
    detections: list[YoloDetection],
    selected: Optional[YoloDetection],
    target_class: Optional[str],
) -> np.ndarray:
    display = frame.copy()
    selected_key = (
        None
        if selected is None
        else (selected.result_index, selected.detection_index)
    )
    for detection in detections:
        is_selected = selected_key == (
            detection.result_index,
            detection.detection_index,
        )
        color = (0, 255, 80) if is_selected else (0, 185, 255)
        thickness = 3 if is_selected else 2
        x1, y1, x2, y2 = detection.bbox_xyxy
        if detection.mask is not None:
            mask = np.asarray(detection.mask, dtype=np.uint8)
            if mask.shape != display.shape[:2]:
                mask = cv2.resize(
                    mask,
                    (display.shape[1], display.shape[0]),
                    interpolation=cv2.INTER_NEAREST,
                )
            valid = mask > 0
            overlay = np.zeros_like(display)
            overlay[valid] = color
            display = cv2.addWeighted(
                display,
                1.0,
                overlay,
                0.28 if is_selected else 0.18,
                0,
            )
            contours, _ = cv2.findContours(
                valid.astype(np.uint8),
                cv2.RETR_EXTERNAL,
                cv2.CHAIN_APPROX_SIMPLE,
            )
            cv2.drawContours(
                display,
                contours,
                -1,
                color,
                thickness,
                cv2.LINE_AA,
            )
        else:
            cv2.rectangle(
                display,
                (x1, y1),
                (x2, y2),
                color,
                thickness,
            )
        label = f"{detection.class_name} {detection.conf:.2f}"
        if target_class and is_selected:
            label = f"TARGET {label}"
        cv2.putText(
            display,
            label,
            (x1 + 4, max(18, y1 - 5)),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.55,
            color,
            2,
            cv2.LINE_AA,
        )
    return display


def draw_best_grasp_projection(
    display: np.ndarray,
    grasp: Optional[Grasp],
    K: np.ndarray,
) -> None:
    if grasp is None:
        return
    x, y, z = [float(value) for value in grasp.translation]
    if z <= 1e-6:
        return
    u = int(round(float(K[0, 0]) * x / z + float(K[0, 2])))
    v = int(round(float(K[1, 1]) * y / z + float(K[1, 2])))
    if 0 <= u < display.shape[1] and 0 <= v < display.shape[0]:
        cv2.drawMarker(
            display,
            (u, v),
            (0, 0, 255),
            cv2.MARKER_CROSS,
            22,
            2,
            cv2.LINE_AA,
        )


def grasp_to_base_poses(
    grasp: Grasp,
    T_cam2base: np.ndarray,
    pregrasp_offset_m: float,
    retreat_offset_m: float,
    insertion_depth_m: float = 0.0,
    *,
    tcp_to_grasp_center_m: tuple[float, float, float] = (0.0, 0.0, 0.0),
    depth_correction_m: float = 0.0,
    orientation_constraint: OrientationConstraint | None = None,
    base_rotation_override: np.ndarray | None = None,
) -> tuple[tuple[float, ...], tuple[float, ...], tuple[float, ...]]:
    """Convert GraspNet reference pose to end-link waypoints.

    ``grasp.depth`` describes GraspNet's gripper geometry and is not a TCP
    translation. ``tcp_to_grasp_center_m`` is a fixed vector expressed in
    GraspNet local coordinates from the TCP to its grasp reference center.
    ``depth_correction_m`` is a runtime translation along local tool X.
    ``insertion_depth_m`` remains as a compatibility alias for the runtime
    correction and must not be combined with it.
    """
    if not np.isfinite(float(insertion_depth_m)):
        raise ValueError("insertion_depth_m must be finite")
    if not np.isfinite(float(depth_correction_m)):
        raise ValueError("depth_correction_m must be finite")
    if abs(float(insertion_depth_m)) > 1e-12 and abs(float(depth_correction_m)) > 1e-12:
        raise ValueError(
            "insertion_depth_m and depth_correction_m cannot both be nonzero"
        )
    correction_m = (
        float(depth_correction_m)
        if abs(float(depth_correction_m)) > 1e-12
        else float(insertion_depth_m)
    )
    tcp_to_center = np.asarray(tcp_to_grasp_center_m, dtype=np.float64)
    if tcp_to_center.shape != (3,) or not np.all(np.isfinite(tcp_to_center)):
        raise ValueError("tcp_to_grasp_center_m must be three finite values")
    tcp_rotation_cam = graspnet_rotation_to_rebot_tcp_rotation(
        grasp.rotation_matrix
    )
    if (
        orientation_constraint is None
        and base_rotation_override is None
        and not np.any(np.abs(tcp_to_center) > 1e-12)
    ):
        return transform_grasp_pose_to_base_with_retreat(
            grasp.translation,
            tcp_rotation_cam,
            T_cam2base,
            pregrasp_offset_m,
            retreat_offset_m,
            correction_m,
            canonicalize_rotation=True,
        )

    center6d, _, _ = transform_grasp_pose_to_base_with_retreat(
        grasp.translation,
        tcp_rotation_cam,
        T_cam2base,
        0.0,
        0.0,
        0.0,
        canonicalize_rotation=True,
    )
    grasp_transform = pose6d_to_mat4(*center6d)
    if base_rotation_override is None:
        if orientation_constraint is not None:
            grasp_transform[:3, :3] = constrain_rotation(
                grasp_transform[:3, :3],
                orientation_constraint,
            )
    else:
        grasp_transform[:3, :3] = np.asarray(
            base_rotation_override,
            dtype=np.float64,
        )
        if orientation_constraint is not None:
            grasp_transform[:3, :3] = constrain_rotation(
                grasp_transform[:3, :3],
                orientation_constraint,
            )
    grasp_transform[:3, 3] -= grasp_transform[:3, :3] @ tcp_to_center
    grasp_transform[:3, 3] += grasp_transform[:3, 0] * correction_m

    def offset(distance_m: float) -> tuple[float, ...]:
        transform = grasp_transform.copy()
        transform[:3, 3] -= (
            transform[:3, 0] * float(distance_m)
        )
        return mat4_to_pose6d(transform)

    return (
        mat4_to_pose6d(grasp_transform),
        offset(pregrasp_offset_m),
        offset(retreat_offset_m),
    )


def draw_status(
    frame: np.ndarray,
    status: str,
    target_status: str = "",
    frozen: bool = False,
    title: str = "Official OBB Grasp",
) -> np.ndarray:
    display = frame.copy()
    lines = [title, "G/SPACE: grasp   R: resume   Q/ESC: quit"]
    if target_status:
        lines.append(target_status)
    lines.append(status)
    y = 28
    for line in lines:
        cv2.putText(
            display,
            line,
            (10, y),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.65,
            (0, 0, 0),
            3,
            cv2.LINE_AA,
        )
        cv2.putText(
            display,
            line,
            (10, y),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.65,
            (255, 255, 255),
            1,
            cv2.LINE_AA,
        )
        y += 26
    if frozen:
        cv2.putText(
            display,
            "[FROZEN]",
            (10, y),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.55,
            (0, 215, 255),
            2,
        )
    return display
