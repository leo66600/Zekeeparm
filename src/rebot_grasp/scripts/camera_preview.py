"""Minimal RGB-D camera preview.

Keys:
  D: toggle depth preview
  Q/Esc: exit

Usage:
    python3 scripts/camera_preview.py
    python3 scripts/camera_preview.py --camera-type realsense_d405
"""

from __future__ import annotations

import argparse
import os
import sys
from pathlib import Path

import cv2
import numpy as np

os.environ.setdefault("QT_QPA_FONTDIR", "/usr/share/fonts/truetype")

PROJECT_ROOT = Path(__file__).resolve().parents[1]
PROJECT_ROOT_STR = str(PROJECT_ROOT)
if PROJECT_ROOT_STR not in sys.path:
    sys.path.insert(0, PROJECT_ROOT_STR)

from drivers.camera import CameraFrameError, make_camera  # noqa: E402
from utils.camera_utils import configure_camera, load_config  # noqa: E402


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Open a live camera preview window.")
    parser.add_argument("--config", default=str(PROJECT_ROOT / "config" / "default.yaml"))
    parser.add_argument(
        "--camera-type",
        choices=("realsense_d435i", "realsense_d405", "orbbec_gemini2"),
        default=None,
    )
    parser.add_argument("--width", type=int, default=None)
    parser.add_argument("--height", type=int, default=None)
    parser.add_argument("--fps", type=int, default=None)
    return parser.parse_args()


def depth_preview(depth_mm: np.ndarray) -> np.ndarray:
    depth = depth_mm.astype(np.float32, copy=False)
    valid = depth[depth > 0]
    if valid.size == 0:
        return np.zeros((*depth_mm.shape, 3), dtype=np.uint8)

    near = float(np.percentile(valid, 2))
    far = float(np.percentile(valid, 98))
    if far <= near:
        far = near + 1.0

    normalized = np.clip((depth - near) * 255.0 / (far - near), 0, 255).astype(np.uint8)
    normalized[depth_mm == 0] = 0
    return cv2.applyColorMap(normalized, cv2.COLORMAP_JET)


def main() -> None:
    args = parse_args()
    cfg = configure_camera(load_config(args.config), args)
    cam_cfg = cfg["camera"]
    cam_type = str(cam_cfg["type"])

    print(
        "Using camera: "
        f"{cam_type} {cam_cfg.get('color_width')}x{cam_cfg.get('color_height')}@{cam_cfg.get('fps')}"
    )

    cam = make_camera(cfg)
    show_depth = False
    window_name = f"Camera Preview ({cam_type})"

    try:
        cam.open()
        cam.warm_up(10)
        print("Camera intrinsics:")
        print(cam.K)
        print("Press D to toggle depth. Press Q or Esc to quit.")

        cv2.namedWindow(window_name, cv2.WINDOW_NORMAL)
        while True:
            color_bgr, depth_mm = cam.get_frame()
            if color_bgr is None:
                key = cv2.waitKey(1) & 0xFF
                if key in (ord("q"), ord("Q"), 27):
                    break
                continue

            display = color_bgr
            if show_depth and depth_mm is not None:
                depth_bgr = depth_preview(depth_mm)
                if depth_bgr.shape[:2] != color_bgr.shape[:2]:
                    depth_bgr = cv2.resize(
                        depth_bgr,
                        (color_bgr.shape[1], color_bgr.shape[0]),
                        interpolation=cv2.INTER_NEAREST,
                    )
                display = np.hstack((color_bgr, depth_bgr))

            cv2.imshow(window_name, display)
            key = cv2.waitKey(1) & 0xFF
            if key in (ord("q"), ord("Q"), 27):
                break
            if key in (ord("d"), ord("D")):
                show_depth = not show_depth
    except (RuntimeError, CameraFrameError) as exc:
        print(f"\n[Fatal] {exc}")
        raise SystemExit(1) from exc
    finally:
        cam.close()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
