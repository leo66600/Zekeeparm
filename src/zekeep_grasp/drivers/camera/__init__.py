from __future__ import annotations

from pathlib import Path

from .base import CameraDriver, CameraFrameError
from .orbbec_gemini2 import OrbbecGemini2
from .realsense import RealsenseCamera

__all__ = ["CameraDriver", "CameraFrameError", "OrbbecGemini2", "RealsenseCamera", "make_camera"]


def make_camera(cfg: dict) -> CameraDriver:
    """Create a camera driver from config/default.yaml."""
    cam_cfg  = cfg.get("camera", {})
    cam_type = cam_cfg.get("type", "").lower()
    w   = cam_cfg.get("color_width",  1280)
    h   = cam_cfg.get("color_height", 720)
    fps = cam_cfg.get("fps", 30)

    _root     = Path(__file__).resolve().parent.parent.parent
    calib_dir = str(_root / "config" / "calibration" / cam_type)
    preset_json = cam_cfg.get("preset_json")
    if preset_json:
        preset_json = Path(str(preset_json)).expanduser()
        if not preset_json.is_absolute():
            preset_json = _root / preset_json

    if "orbbec" in cam_type:
        return OrbbecGemini2(
            w,
            h,
            fps,
            calib_dir=calib_dir,
            preset_json=(
                str(preset_json)
                if preset_json is not None
                else None
            ),
            color_auto_exposure=cam_cfg.get("color_auto_exposure"),
            color_exposure=cam_cfg.get("color_exposure"),
            color_gain=cam_cfg.get("color_gain"),
            color_sharpness=cam_cfg.get("color_sharpness"),
            color_contrast=cam_cfg.get("color_contrast"),
            color_distortion_mode=cam_cfg.get(
                "color_distortion_mode",
                "factory",
            ),
        )
    elif "realsense" in cam_type:
        return RealsenseCamera(w, h, fps, calib_dir=calib_dir)
    else:
        raise ValueError(
            f"Unsupported camera type: {cam_type!r}\n"
            f"Set camera.type in config/default.yaml to:\n"
            f"  orbbec_gemini2 | realsense_d435i | realsense_d405"
        )
