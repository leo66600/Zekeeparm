"""Orbbec Gemini 2 camera driver.

OrbbecGemini2:
  open() / close(): manage the pyorbbecsdk pipeline.
  get_frame(): return aligned BGR and depth-mm frames.
  K / D: camera matrix and distortion coefficients.
"""
from __future__ import annotations

import os
import numpy as np
import cv2
from pathlib import Path
from typing import Optional, Tuple

from .base import CameraDriver, CameraFrameError


class OrbbecGemini2(CameraDriver):
    """Orbbec Gemini 2 RGB-D camera driver."""

    def __init__(
        self,
        width: int = 1280,
        height: int = 720,
        fps: int = 30,
        calib_dir: Optional[str] = None,
        preset_json: Optional[str] = None,
        color_auto_exposure: Optional[bool] = None,
        color_exposure: Optional[int] = None,
        color_gain: Optional[int] = None,
        color_sharpness: Optional[int] = None,
        color_contrast: Optional[int] = None,
        color_distortion_mode: str = "factory",
    ) -> None:
        self._w = width
        self._h = height
        self._fps = fps
        self._calib_dir = Path(calib_dir) if calib_dir else None
        self._preset_json = Path(preset_json) if preset_json else None
        self._color_auto_exposure = color_auto_exposure
        self._color_exposure = color_exposure
        self._color_gain = color_gain
        self._color_sharpness = color_sharpness
        self._color_contrast = color_contrast
        self._color_distortion_mode = str(color_distortion_mode).lower()
        if self._color_distortion_mode not in {"factory", "zero"}:
            raise ValueError(
                "camera.color_distortion_mode must be 'factory' or 'zero'"
            )

        self._pipeline = None
        self._depth_scale_mm: float = 1.0
        self._K: Optional[np.ndarray] = None
        self._D: Optional[np.ndarray] = None
        self._aruco = None
        self._reset_frame_failures()

    # Lifecycle

    def open(self) -> None:
        """Open the camera pipeline."""
        # Import first so native load errors stay visible.
        try:
            import pyorbbecsdk as sdk
            from pyorbbecsdk import (
                Pipeline, Config,
                OBSensorType, OBFormat, OBAlignMode,
                Context,
            )
        except ImportError as e:
            raise RuntimeError(f"pyorbbecsdk is not installed: {e}") from e

        # Silence noisy native logs during SDK initialization.
        devnull = os.open(os.devnull, os.O_WRONLY)
        saved = os.dup(2)
        os.dup2(devnull, 2)
        os.close(devnull)

        try:
            try:
                from pyorbbecsdk import OBLogSeverity
                Context().set_logger_severity(OBLogSeverity.FATAL)
            except Exception:
                pass

            try:
                self._pipeline = Pipeline()
            except Exception as e:
                rules_installer = (
                    Path(sdk.__file__).resolve().parent
                    / "shared"
                    / "install_udev_rules.sh"
                )
                raise RuntimeError(
                    f"Orbbec camera not found: {e}\n"
                    "  Check the USB connection and install the Orbbec udev "
                    "rules:\n"
                    f"  sudo sh {rules_installer}\n"
                    "  Then unplug and reconnect the camera."
                ) from e

            self.serial = str(self._pipeline.get_device().get_device_info().get_serial_number())
            self._configure_device_properties(
                self._pipeline.get_device(),
                sdk,
            )

            cfg = Config()

            # Uncompressed color stream
            plist = self._pipeline.get_stream_profile_list(OBSensorType.COLOR_SENSOR)
            cp = self._get_yuyv_color_profile(plist, OBFormat)
            cfg.enable_stream(cp)

            # Depth stream
            dplist = self._pipeline.get_stream_profile_list(OBSensorType.DEPTH_SENSOR)
            try:
                dp = dplist.get_video_stream_profile(self._w, self._h, OBFormat.Y16, self._fps)
            except Exception:
                dp = dplist.get_default_video_stream_profile()
            cfg.enable_stream(dp)

            cfg.set_align_mode(OBAlignMode.HW_MODE)
            try:
                self._pipeline.start(cfg)
            except Exception as hw_exc:
                if "hardware d2c" not in str(hw_exc).lower():
                    raise
                print(
                    "[OrbbecGemini2] Hardware D2C is not supported by the selected "
                    f"{self._w}x{self._h}@{self._fps} profile; using software D2C."
                )
                cfg.set_align_mode(OBAlignMode.SW_MODE)
                self._pipeline.start(cfg)
            self._reset_frame_failures()

            # Matched RGB factory calibration for the active stream profiles.
            self._K, factory_D, _ = self._factory_rgb_calibration(
                self._pipeline.get_camera_param()
            )
            self._D = self._select_color_distortion(
                factory_D,
                mode=self._color_distortion_mode,
            )

        finally:
            os.dup2(saved, 2)
            os.close(saved)

    def close(self) -> None:
        if self._pipeline is not None:
            try:
                self._pipeline.stop()
            except Exception:
                pass
            self._pipeline = None

    # Frames

    def get_frame(self) -> Tuple[Optional[np.ndarray], Optional[np.ndarray]]:
        if self._pipeline is None:
            return None, None
        try:
            from pyorbbecsdk import OBFormat
            frames = self._pipeline.wait_for_frames(500)
            if frames is None:
                self._record_frame_failure("wait_for_frames timeout")
                return None, None

            color_bgr = None
            cf = frames.get_color_frame()
            if cf is not None:
                w, h = cf.get_width(), cf.get_height()
                raw = np.asarray(cf.get_data(), dtype=np.uint8).copy()
                fmt = cf.get_format()
                try:
                    if fmt not in (OBFormat.YUYV, OBFormat.YUY2):
                        raise RuntimeError(
                            f"Unexpected color format: {fmt}; expected YUYV"
                        )
                    color_bgr = self._decode_yuyv(raw, w, h)
                except Exception as exc:
                    self._record_frame_failure(
                        f"YUYV color decode failed: {exc}"
                    )
                    return None, None

            depth_mm = None
            df = frames.get_depth_frame()
            if df is not None:
                dw, dh = df.get_width(), df.get_height()
                depth_raw = np.frombuffer(df.get_data(), dtype=np.uint16).reshape(dh, dw)
                depth_scale = self._depth_scale_mm
                try:
                    depth_scale = float(df.get_depth_scale())
                    self._depth_scale_mm = depth_scale
                except Exception:
                    pass
                depth_mm = np.clip(
                    np.rint(depth_raw.astype(np.float32) * depth_scale),
                    0,
                    np.iinfo(np.uint16).max,
                ).astype(np.uint16)

            if color_bgr is None or depth_mm is None:
                self._record_frame_failure("missing color or depth frame")
            else:
                self._reset_frame_failures()
            return color_bgr, depth_mm
        except CameraFrameError:
            raise
        except Exception as exc:
            self._record_frame_failure(str(exc))
            return None, None

    # Intrinsics

    @property
    def K(self) -> np.ndarray:
        if self._K is None:
            raise RuntimeError("Camera is not open")
        return self._K

    @property
    def D(self) -> np.ndarray:
        if self._D is None:
            raise RuntimeError("Camera is not open")
        return self._D

    # Internals

    @staticmethod
    def _select_color_distortion(
        factory_distortion: np.ndarray,
        *,
        mode: str,
    ) -> np.ndarray:
        distortion = np.asarray(factory_distortion, dtype=np.float64)
        if mode == "factory":
            return distortion.copy()
        if mode == "zero":
            return np.zeros_like(distortion)
        raise ValueError(
            "camera.color_distortion_mode must be 'factory' or 'zero'"
        )

    @staticmethod
    def _factory_rgb_calibration(
        camera_param,
    ) -> tuple[np.ndarray, np.ndarray, tuple[int, int]]:
        intr = camera_param.rgb_intrinsic
        distortion = camera_param.rgb_distortion
        K = np.array(
            [
                [intr.fx, 0.0, intr.cx],
                [0.0, intr.fy, intr.cy],
                [0.0, 0.0, 1.0],
            ],
            dtype=np.float64,
        )
        D = np.array(
            [
                [
                    distortion.k1,
                    distortion.k2,
                    distortion.p1,
                    distortion.p2,
                    distortion.k3,
                ]
            ],
            dtype=np.float64,
        )
        if not np.all(np.isfinite(K)) or not np.all(np.isfinite(D)):
            raise RuntimeError("Orbbec factory RGB calibration is non-finite")
        if K[0, 0] <= 0.0 or K[1, 1] <= 0.0:
            raise RuntimeError("Orbbec factory RGB focal length is invalid")
        return K, D, (int(intr.width), int(intr.height))

    def _get_yuyv_color_profile(self, profiles, formats):
        try:
            return profiles.get_video_stream_profile(
                self._w, self._h, formats.YUYV, self._fps
            )
        except Exception as exc:
            raise RuntimeError(
                "YUYV color profile is unavailable for "
                f"{self._w}x{self._h}@{self._fps}. "
                "Check that the camera is connected through USB 3."
            ) from exc

    @staticmethod
    def _decode_yuyv(
        raw: np.ndarray, width: int, height: int
    ) -> np.ndarray:
        expected_size = width * height * 2
        if raw.size != expected_size:
            raise ValueError(
                f"YUYV frame has {raw.size} bytes; expected {expected_size}"
            )
        yuyv = raw.reshape(height, width, 2)
        return cv2.cvtColor(yuyv, cv2.COLOR_YUV2BGR_YUY2)

    def _configure_color_properties(self, device, sdk) -> None:
        """Apply explicitly configured RGB controls to the camera device."""
        manual_exposure = (
            self._color_exposure is not None or self._color_gain is not None
        )
        if manual_exposure and self._color_auto_exposure is not False:
            raise ValueError(
                "Manual color exposure/gain requires "
                "camera.color_auto_exposure: false"
            )

        property_ids = sdk.OBPropertyID
        write_permission = sdk.OBPermissionType.PERMISSION_WRITE

        if self._color_auto_exposure is not None:
            property_id = property_ids.OB_PROP_COLOR_AUTO_EXPOSURE_BOOL
            self._require_writable(
                device, property_id, "color_auto_exposure", write_permission
            )
            device.set_bool_property(
                property_id, bool(self._color_auto_exposure)
            )

        int_settings = (
            (
                "color_exposure",
                self._color_exposure,
                property_ids.OB_PROP_COLOR_EXPOSURE_INT,
            ),
            (
                "color_gain",
                self._color_gain,
                property_ids.OB_PROP_COLOR_GAIN_INT,
            ),
            (
                "color_sharpness",
                self._color_sharpness,
                property_ids.OB_PROP_COLOR_SHARPNESS_INT,
            ),
            (
                "color_contrast",
                self._color_contrast,
                property_ids.OB_PROP_COLOR_CONTRAST_INT,
            ),
        )
        applied = []
        for name, value, property_id in int_settings:
            if value is None:
                continue
            self._require_writable(
                device, property_id, name, write_permission
            )
            value = int(value)
            value_range = device.get_int_property_range(property_id)
            step = max(int(value_range.step), 1)
            valid = (
                int(value_range.min) <= value <= int(value_range.max)
                and (value - int(value_range.min)) % step == 0
            )
            if not valid:
                raise ValueError(
                    f"camera.{name}={value} is invalid; supported range is "
                    f"{value_range.min}..{value_range.max}, step={step}"
                )
            device.set_int_property(property_id, value)
            applied.append(f"{name}={value}")

        if self._color_auto_exposure is not None:
            applied.insert(
                0, f"color_auto_exposure={self._color_auto_exposure}"
            )
        if applied:
            print(f"[OrbbecGemini2] Color controls: {', '.join(applied)}")

    def _configure_device_properties(self, device, sdk) -> None:
        """Load a Viewer preset, then apply explicit YAML overrides."""
        if self._preset_json is not None:
            if not self._preset_json.is_file():
                raise FileNotFoundError(
                    f"Camera preset JSON not found: {self._preset_json}"
                )
            device.load_preset_from_json_file(str(self._preset_json))
            print(
                "[OrbbecGemini2] Loaded preset: "
                f"{self._preset_json}"
            )
        self._configure_color_properties(device, sdk)

    @staticmethod
    def _require_writable(
        device, property_id, name: str, write_permission
    ) -> None:
        if not device.is_property_supported(property_id, write_permission):
            raise RuntimeError(
                f"Camera does not support writing camera.{name}"
            )

    def _load_distortion(self) -> np.ndarray:
        """Load distortion; fall back to zeros for invalid calibration."""
        if self._calib_dir is not None:
            npz_path = self._calib_dir / "intrinsics.npz"
            if npz_path.exists():
                try:
                    data = np.load(str(npz_path))
                    D = data["dist_coeffs"].flatten()
                    if abs(D[0]) > 5.0:
                        print(f"[OrbbecGemini2] Invalid k1={D[0]:.2f}; using zero distortion")
                        return np.zeros((1, 5), dtype=np.float64)
                    return D.reshape(1, -1)
                except Exception:
                    pass
        return np.zeros((1, 5), dtype=np.float64)
