"""Shared camera/config helpers for scripts."""

from __future__ import annotations

import argparse
from pathlib import Path
from typing import Any, Optional

import numpy as np
import yaml

try:
    from ..drivers.camera import CameraDriver, make_camera
except ImportError:
    from drivers.camera import CameraDriver, make_camera


def load_config(path: str | Path) -> dict[str, Any]:
    config_path = Path(path).expanduser()
    if not config_path.exists():
        raise FileNotFoundError(f"Config not found: {config_path}")
    with open(config_path, encoding="utf-8") as f:
        return yaml.safe_load(f)


def validate_hand_eye_transform(T: np.ndarray) -> np.ndarray:
    transform = np.asarray(T, dtype=np.float64)
    if transform.shape != (4, 4):
        raise ValueError(
            f"hand-eye transform must have shape (4, 4), got {transform.shape}"
        )
    if not np.all(np.isfinite(transform)):
        raise ValueError("hand-eye transform contains non-finite values")
    if not np.allclose(transform[3], [0.0, 0.0, 0.0, 1.0], atol=1e-8):
        raise ValueError("hand-eye transform has an invalid homogeneous bottom row")
    rotation = transform[:3, :3]
    orthogonality_error = float(
        np.linalg.norm(rotation.T @ rotation - np.eye(3))
    )
    determinant = float(np.linalg.det(rotation))
    if orthogonality_error > 1e-3 or abs(determinant - 1.0) > 1e-3:
        raise ValueError(
            "hand-eye transform rotation is not a proper rotation "
            f"(orthogonality_error={orthogonality_error:.3e}, det={determinant:.6f})"
        )
    return transform.copy()


def validate_runtime_intrinsics(
    runtime_K: np.ndarray,
    calibrated_K: np.ndarray,
    *,
    runtime_resolution: tuple[int, int],
    calibrated_resolution: tuple[int, int],
    relative_focal_tolerance: float = 0.02,
    principal_point_tolerance_px: float = 10.0,
) -> None:
    runtime = np.asarray(runtime_K, dtype=np.float64)
    calibrated = np.asarray(calibrated_K, dtype=np.float64)
    if runtime.shape != (3, 3) or calibrated.shape != (3, 3):
        raise ValueError("runtime and calibrated camera matrices must be 3x3")
    if not np.all(np.isfinite(runtime)) or not np.all(np.isfinite(calibrated)):
        raise ValueError("camera intrinsics contain non-finite values")
    if tuple(runtime_resolution) != tuple(calibrated_resolution):
        raise ValueError(
            "camera calibration resolution mismatch: "
            f"runtime={tuple(runtime_resolution)} "
            f"calibrated={tuple(calibrated_resolution)}"
        )

    calibrated_focal = np.array(
        [calibrated[0, 0], calibrated[1, 1]],
        dtype=np.float64,
    )
    if np.any(calibrated_focal <= 0.0):
        raise ValueError("calibrated focal length must be positive")
    relative_focal_error = np.abs(
        np.array([runtime[0, 0], runtime[1, 1]]) / calibrated_focal - 1.0
    )
    if float(np.max(relative_focal_error)) > float(relative_focal_tolerance):
        raise ValueError(
            "camera focal length does not match calibration: "
            f"runtime=({runtime[0, 0]:.2f},{runtime[1, 1]:.2f}) "
            f"calibrated=({calibrated[0, 0]:.2f},{calibrated[1, 1]:.2f}) "
            f"max_error={100.0 * float(np.max(relative_focal_error)):.1f}%"
        )

    principal_error = np.abs(
        np.array([runtime[0, 2], runtime[1, 2]])
        - np.array([calibrated[0, 2], calibrated[1, 2]])
    )
    if float(np.max(principal_error)) > float(principal_point_tolerance_px):
        raise ValueError(
            "camera principal point does not match calibration: "
            f"max_error={float(np.max(principal_error)):.1f}px"
        )


def validate_runtime_distortion(
    runtime_D: np.ndarray,
    calibrated_D: np.ndarray,
    *,
    tolerance: float = 1e-9,
) -> None:
    runtime = np.asarray(runtime_D, dtype=np.float64).reshape(-1)
    calibrated = np.asarray(calibrated_D, dtype=np.float64).reshape(-1)
    if (
        runtime.size == 0
        or calibrated.size == 0
        or runtime.shape != calibrated.shape
        or not np.all(np.isfinite(runtime))
        or not np.all(np.isfinite(calibrated))
    ):
        raise ValueError("camera distortion calibration is invalid")
    max_error = float(np.max(np.abs(runtime - calibrated)))
    if max_error > float(tolerance):
        raise ValueError(
            "runtime camera distortion does not match saved calibration: "
            f"max_error={max_error:.6f}"
        )


def validate_runtime_camera_calibration(
    project_root: str | Path,
    cam_type: str,
    runtime_K: np.ndarray,
    runtime_resolution: tuple[int, int],
    runtime_D: np.ndarray,
) -> None:
    intrinsics_path = (
        Path(project_root)
        / "config"
        / "calibration"
        / str(cam_type).lower()
        / "intrinsics.npz"
    )
    if not intrinsics_path.is_file():
        raise FileNotFoundError(
            f"Camera calibration is required for execution: {intrinsics_path}"
        )
    with np.load(str(intrinsics_path), allow_pickle=False) as data:
        required = {"camera_matrix", "dist_coeffs", "resolution"}
        missing = required.difference(data.files)
        if missing:
            raise ValueError(
                f"Camera calibration is missing keys {sorted(missing)}: "
                f"{intrinsics_path}"
            )
        calibrated_K = np.asarray(data["camera_matrix"]).copy()
        distortion = np.asarray(
            data["dist_coeffs"],
            dtype=np.float64,
        ).reshape(-1)
        resolution_array = np.asarray(data["resolution"]).reshape(-1)
    if resolution_array.size != 2:
        raise ValueError(
            f"Camera calibration resolution must have two values: "
            f"{intrinsics_path}"
        )
    validate_runtime_intrinsics(
        runtime_K,
        calibrated_K,
        runtime_resolution=runtime_resolution,
        calibrated_resolution=(
            int(resolution_array[0]),
            int(resolution_array[1]),
        ),
    )
    if not np.all(np.isfinite(distortion)):
        raise ValueError("camera distortion calibration contains non-finite values")
    if distortion.size == 0 or abs(float(distortion[0])) > 5.0:
        raise ValueError(
            "camera distortion calibration is invalid: "
            f"k1={float(distortion[0]) if distortion.size else float('nan'):.3f}"
        )
    validate_runtime_distortion(runtime_D, distortion)


def load_hand_eye(
    project_root: str | Path,
    cam_type: str,
    *,
    expected_reference_frame: str | None = None,
) -> tuple[Optional[np.ndarray], Optional[str]]:
    hand_eye_path = Path(project_root) / "config" / "calibration" / str(cam_type).lower() / "hand_eye.npz"
    if not hand_eye_path.exists():
        return None, None

    with np.load(str(hand_eye_path), allow_pickle=False) as data:
        if "T_result" not in data or "mode" not in data:
            raise ValueError(f"Invalid hand-eye calibration file: {hand_eye_path}")
        T = validate_hand_eye_transform(data["T_result"])
        mode = str(data["mode"][0])
        reference_frame = (
            str(data["reference_frame"][0])
            if "reference_frame" in data
            else ""
        )
        n_samples = (
            int(data["n_samples"][0])
            if "n_samples" in data
            else None
        )
    if n_samples is not None and n_samples < 5:
        raise ValueError(
            f"Hand-eye calibration has too few samples: {n_samples}"
        )
    expected = str(expected_reference_frame or "").strip()
    if expected and reference_frame != expected:
        raise ValueError(
            "Hand-eye reference frame mismatch: "
            f"saved={reference_frame or '<legacy>'!r} expected={expected!r}"
        )
    return T, mode


def camera_to_base_from_hand_eye_reference(
    kinematics,
    joints: np.ndarray,
    hand_eye: np.ndarray,
    cfg: dict[str, Any],
    reference_frame: str,
) -> np.ndarray:
    """Compose eye-in-hand camera FK using the frame saved by calibration."""
    frame = str(reference_frame).strip()
    if not frame:
        raise ValueError("hand-eye reference frame is empty")
    reference_to_base = kinematics.fk_frame(joints, frame)
    return compose_cam_to_base_transform(reference_to_base, hand_eye, cfg)


def hand_eye_compensation_matrix(cfg: dict[str, Any]) -> np.ndarray:
    calibration = cfg.get("calibration") or {}
    compensation = calibration.get("hand_eye_compensation_m") or {}
    T = np.eye(4, dtype=np.float64)
    T[:3, 3] = [
        float(compensation.get("x", 0.0)),
        float(compensation.get("y", 0.0)),
        float(compensation.get("z", 0.0)),
    ]
    return T


def compose_cam_to_base_transform(T_tcp2base: np.ndarray, T_hand_eye: np.ndarray, cfg: dict[str, Any]) -> np.ndarray:
    T_compensation = hand_eye_compensation_matrix(cfg)
    return T_compensation @ np.asarray(T_tcp2base, dtype=np.float64) @ np.asarray(T_hand_eye, dtype=np.float64)


def configure_camera(
    cfg: dict[str, Any],
    args: argparse.Namespace,
    *,
    realsense_default_fps: int | None = 15,
) -> dict[str, Any]:
    cam_cfg = cfg.setdefault("camera", {})
    camera_type = getattr(args, "camera_type", None)
    width = getattr(args, "width", None)
    height = getattr(args, "height", None)
    fps = getattr(args, "fps", None)

    if camera_type is not None:
        cam_cfg["type"] = camera_type
    cam_type = str(cam_cfg.get("type", "")).lower()
    if not cam_type:
        raise ValueError("camera.type is missing in config; pass --camera-type or set it in YAML")

    if width is not None:
        cam_cfg["color_width"] = int(width)
        cam_cfg["depth_width"] = int(width)
    if height is not None:
        cam_cfg["color_height"] = int(height)
        cam_cfg["depth_height"] = int(height)
    if fps is not None:
        cam_cfg["fps"] = int(fps)
    elif camera_type is not None and realsense_default_fps is not None and "realsense" in cam_type:
        cam_cfg["fps"] = int(realsense_default_fps)
    return cfg


def create_camera_from_args(
    cfg: dict[str, Any],
    args: argparse.Namespace,
    *,
    realsense_default_fps: int | None = 15,
) -> CameraDriver:
    return make_camera(configure_camera(cfg, args, realsense_default_fps=realsense_default_fps))
