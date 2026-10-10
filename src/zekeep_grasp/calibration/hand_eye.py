"""手眼标定 — 基于 OpenCV calibrateHandEye。"""
from __future__ import annotations

from dataclasses import dataclass
from enum import Enum
from pathlib import Path
from typing import List, Union

import cv2
import numpy as np


class CalibMode(Enum):
    EYE_IN_HAND = "eye_in_hand"   # 相机在末端，随末端运动


_METHOD_MAP = {
    "TSAI":       cv2.CALIB_HAND_EYE_TSAI,
    "PARK":       cv2.CALIB_HAND_EYE_PARK,
    "HORAUD":     cv2.CALIB_HAND_EYE_HORAUD,
    "ANDREFF":    cv2.CALIB_HAND_EYE_ANDREFF,
    "DANIILIDIS": cv2.CALIB_HAND_EYE_DANIILIDIS,
}


@dataclass
class CalibResult:
    T_result: np.ndarray    # (4, 4) 手眼变换矩阵
    mode: str               # CalibMode.value
    n_samples: int
    method: str
    T_gripper2base_samples: np.ndarray
    T_marker2cam_samples: np.ndarray
    target2base_samples: np.ndarray
    translation_residuals_m: np.ndarray
    rotation_residuals_deg: np.ndarray
    translation_rmse_m: float
    rotation_rmse_deg: float
    sample_reprojection_rmse_px: np.ndarray
    sample_corner_count: np.ndarray
    reference_frame: str = ""
    urdf_path: str = ""


@dataclass(frozen=True)
class HandEyeClosureQuality:
    target2base_samples: np.ndarray
    translation_residuals_m: np.ndarray
    rotation_residuals_deg: np.ndarray
    translation_rmse_m: float
    rotation_rmse_deg: float


@dataclass
class _Sample:
    T_gripper2base: np.ndarray   # (4, 4)
    T_marker2cam:   np.ndarray   # (4, 4)
    reprojection_rmse_px: float
    corner_count: int


def _mean_rotation(rotations: np.ndarray) -> np.ndarray:
    U, _, Vt = np.linalg.svd(np.sum(rotations, axis=0))
    mean = U @ Vt
    if np.linalg.det(mean) < 0.0:
        U[:, -1] *= -1.0
        mean = U @ Vt
    return mean


def compute_handeye_closure_quality(
    T_cam2gripper: np.ndarray,
    T_gripper2base_samples: np.ndarray,
    T_marker2cam_samples: np.ndarray,
) -> HandEyeClosureQuality:
    """Measure how consistently all samples place the fixed target in base."""
    camera_to_gripper = np.asarray(T_cam2gripper, dtype=np.float64)
    gripper_to_base = np.asarray(
        T_gripper2base_samples,
        dtype=np.float64,
    )
    marker_to_camera = np.asarray(
        T_marker2cam_samples,
        dtype=np.float64,
    )
    if camera_to_gripper.shape != (4, 4):
        raise ValueError("T_cam2gripper must have shape (4, 4)")
    if (
        gripper_to_base.ndim != 3
        or gripper_to_base.shape[1:] != (4, 4)
        or marker_to_camera.shape != gripper_to_base.shape
        or gripper_to_base.shape[0] == 0
    ):
        raise ValueError("hand-eye sample arrays must have shape (N, 4, 4)")

    target_to_base = (
        gripper_to_base
        @ camera_to_gripper[None, :, :]
        @ marker_to_camera
    )
    translations = target_to_base[:, :3, 3]
    translation_center = np.mean(translations, axis=0)
    translation_residuals = np.linalg.norm(
        translations - translation_center,
        axis=1,
    )

    rotations = target_to_base[:, :3, :3]
    rotation_center = _mean_rotation(rotations)
    rotation_residuals = []
    for rotation in rotations:
        relative = rotation_center.T @ rotation
        cosine = np.clip((np.trace(relative) - 1.0) * 0.5, -1.0, 1.0)
        rotation_residuals.append(np.degrees(np.arccos(cosine)))
    rotation_residuals_array = np.asarray(
        rotation_residuals,
        dtype=np.float64,
    )
    return HandEyeClosureQuality(
        target2base_samples=target_to_base,
        translation_residuals_m=translation_residuals,
        rotation_residuals_deg=rotation_residuals_array,
        translation_rmse_m=float(
            np.sqrt(np.mean(translation_residuals**2))
        ),
        rotation_rmse_deg=float(
            np.sqrt(np.mean(rotation_residuals_array**2))
        ),
    )


class HandEyeCalibrator:
    """
    手眼标定器。

    Eye-in-Hand 模式：
        求解 T_cam2gripper，使得
            T_marker2base = T_gripper2base @ T_cam2gripper @ T_marker2cam
        在所有姿态下恒成立。

    使用方法：
        calib = HandEyeCalibrator(CalibMode.EYE_IN_HAND)
        calib.add_sample(T_gripper2base, T_marker2cam)
        ...
        result = calib.calibrate()
        HandEyeCalibrator.save(result, "hand_eye.npz")
    """

    def __init__(
        self,
        mode: CalibMode = CalibMode.EYE_IN_HAND,
        method: str = "TSAI",
        reference_frame: str = "",
        urdf_path: str = "",
    ) -> None:
        if mode != CalibMode.EYE_IN_HAND:
            raise ValueError("Only eye-in-hand calibration is supported")
        self._mode = mode
        self._method = method.upper()
        self._reference_frame = str(reference_frame)
        self._urdf_path = str(urdf_path)
        self._samples: List[_Sample] = []

    @property
    def n_samples(self) -> int:
        return len(self._samples)

    def add_sample(
        self,
        T_gripper2base: np.ndarray,
        T_marker2cam: np.ndarray,
        *,
        reprojection_rmse_px: float = float("nan"),
        corner_count: int = -1,
    ) -> None:
        """
        添加一个标定样本。

        Args:
            T_gripper2base: (4,4) 末端到基座的变换（正运动学 FK 输出）
            T_marker2cam:   (4,4) 标记到相机的变换（ArUco 检测输出）
        """
        self._samples.append(_Sample(
            T_gripper2base=np.asarray(T_gripper2base, dtype=np.float64),
            T_marker2cam=np.asarray(T_marker2cam, dtype=np.float64),
            reprojection_rmse_px=float(reprojection_rmse_px),
            corner_count=int(corner_count),
        ))

    def calibrate(self, min_samples: int = 5) -> CalibResult:
        """
        计算手眼变换。

        Args:
            min_samples: 最少样本数（< 此值会抛出异常）

        Returns:
            CalibResult，T_result 即手眼变换矩阵
        """
        if self.n_samples < min_samples:
            raise ValueError(
                f"样本不足：{self.n_samples} < {min_samples}，请继续采集"
            )

        cv_method = _METHOD_MAP.get(self._method, cv2.CALIB_HAND_EYE_TSAI)

        # OpenCV 接口：R_gripper2base, t_gripper2base, R_target2cam, t_target2cam
        R_g2b = [s.T_gripper2base[:3, :3] for s in self._samples]
        t_g2b = [s.T_gripper2base[:3,  3].reshape(3, 1) for s in self._samples]
        R_t2c = [s.T_marker2cam[:3, :3] for s in self._samples]
        t_t2c = [s.T_marker2cam[:3,  3].reshape(3, 1) for s in self._samples]

        R_c2g, t_c2g = cv2.calibrateHandEye(
            R_g2b, t_g2b, R_t2c, t_t2c, method=cv_method
        )
        T = np.eye(4, dtype=np.float64)
        T[:3, :3] = R_c2g
        T[:3,  3] = t_c2g.flatten()
        gripper_samples = np.stack(
            [sample.T_gripper2base for sample in self._samples]
        )
        marker_samples = np.stack(
            [sample.T_marker2cam for sample in self._samples]
        )
        quality = compute_handeye_closure_quality(
            T,
            gripper_samples,
            marker_samples,
        )

        return CalibResult(
            T_result=T,
            mode=self._mode.value,
            n_samples=self.n_samples,
            method=self._method,
            T_gripper2base_samples=gripper_samples,
            T_marker2cam_samples=marker_samples,
            target2base_samples=quality.target2base_samples,
            translation_residuals_m=quality.translation_residuals_m,
            rotation_residuals_deg=quality.rotation_residuals_deg,
            translation_rmse_m=quality.translation_rmse_m,
            rotation_rmse_deg=quality.rotation_rmse_deg,
            sample_reprojection_rmse_px=np.asarray(
                [
                    sample.reprojection_rmse_px
                    for sample in self._samples
                ],
                dtype=np.float64,
            ),
            sample_corner_count=np.asarray(
                [sample.corner_count for sample in self._samples],
                dtype=np.int32,
            ),
            reference_frame=self._reference_frame,
            urdf_path=self._urdf_path,
        )

    @staticmethod
    def save(result: CalibResult, path: Union[str, Path]) -> None:
        """保存标定结果为 .npz 文件。"""
        path = Path(path)
        path.parent.mkdir(parents=True, exist_ok=True)
        np.savez(
            str(path),
            T_result=result.T_result,
            mode=np.array([result.mode]),
            n_samples=np.array([result.n_samples]),
            method=np.array([result.method]),
            T_gripper2base_samples=result.T_gripper2base_samples,
            T_marker2cam_samples=result.T_marker2cam_samples,
            target2base_samples=result.target2base_samples,
            translation_residuals_m=result.translation_residuals_m,
            rotation_residuals_deg=result.rotation_residuals_deg,
            translation_rmse_m=np.array([result.translation_rmse_m]),
            rotation_rmse_deg=np.array([result.rotation_rmse_deg]),
            sample_reprojection_rmse_px=(
                result.sample_reprojection_rmse_px
            ),
            sample_corner_count=result.sample_corner_count,
            reference_frame=np.array([result.reference_frame]),
            urdf_path=np.array([result.urdf_path]),
        )

    @staticmethod
    def load(path: Union[str, Path]) -> CalibResult:
        """从 .npz 文件加载标定结果。"""
        data = np.load(str(path), allow_pickle=False)
        empty_transforms = np.empty((0, 4, 4), dtype=np.float64)
        empty_values = np.empty(0, dtype=np.float64)
        return CalibResult(
            T_result=data["T_result"],
            mode=str(data["mode"][0]),
            n_samples=int(data["n_samples"][0]),
            method=str(data["method"][0]) if "method" in data else "TSAI",
            T_gripper2base_samples=(
                data["T_gripper2base_samples"]
                if "T_gripper2base_samples" in data
                else empty_transforms
            ),
            T_marker2cam_samples=(
                data["T_marker2cam_samples"]
                if "T_marker2cam_samples" in data
                else empty_transforms
            ),
            target2base_samples=(
                data["target2base_samples"]
                if "target2base_samples" in data
                else empty_transforms
            ),
            translation_residuals_m=(
                data["translation_residuals_m"]
                if "translation_residuals_m" in data
                else empty_values
            ),
            rotation_residuals_deg=(
                data["rotation_residuals_deg"]
                if "rotation_residuals_deg" in data
                else empty_values
            ),
            translation_rmse_m=(
                float(data["translation_rmse_m"][0])
                if "translation_rmse_m" in data
                else float("nan")
            ),
            rotation_rmse_deg=(
                float(data["rotation_rmse_deg"][0])
                if "rotation_rmse_deg" in data
                else float("nan")
            ),
            sample_reprojection_rmse_px=(
                data["sample_reprojection_rmse_px"]
                if "sample_reprojection_rmse_px" in data
                else empty_values
            ),
            sample_corner_count=(
                data["sample_corner_count"]
                if "sample_corner_count" in data
                else np.empty(0, dtype=np.int32)
            ),
            reference_frame=(
                str(data["reference_frame"][0])
                if "reference_frame" in data
                else ""
            ),
            urdf_path=(
                str(data["urdf_path"][0])
                if "urdf_path" in data
                else ""
            ),
        )
