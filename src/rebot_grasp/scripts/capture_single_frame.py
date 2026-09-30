"""Capture one RGB-D frame and save color/depth PNG files.

Usage:
    python3 scripts/capture_single_frame.py --out-dir data/rgbd_task2
"""

from __future__ import annotations

import argparse
import sys
from pathlib import Path
from typing import Sequence

import cv2

PROJECT_ROOT = Path(__file__).resolve().parents[1]
PROJECT_ROOT_STR = str(PROJECT_ROOT)
if PROJECT_ROOT_STR not in sys.path:
    sys.path.insert(0, PROJECT_ROOT_STR)

from drivers.camera import CameraFrameError, make_camera  # noqa: E402
from utils.camera_utils import load_config  # noqa: E402


def parse_args(argv: Sequence[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Capture and save one RGB-D color/depth frame."
    )
    parser.add_argument(
        "--config",
        type=Path,
        default=PROJECT_ROOT / "config" / "default.yaml",
        help="Camera configuration YAML path.",
    )
    parser.add_argument(
        "--out-dir",
        type=Path,
        required=True,
        help="Directory for color_bgr.png and depth_mm_16uc1.png.",
    )
    parser.add_argument(
        "--warm-up-frames",
        type=int,
        default=20,
        help="Frames discarded before capture (default: 20).",
    )
    return parser.parse_args(argv)


def capture_single_frame(
    *,
    config_path: Path,
    out_dir: Path,
    warm_up_frames: int = 20,
) -> tuple[Path, Path]:
    """Capture one BGR/depth-mm frame and return saved file paths."""
    if warm_up_frames < 0:
        raise ValueError("warm_up_frames must be non-negative")

    output_dir = Path(out_dir)
    output_dir.mkdir(parents=True, exist_ok=True)
    color_path = output_dir / "color_bgr.png"
    depth_path = output_dir / "depth_mm_16uc1.png"

    camera = make_camera(load_config(config_path))
    try:
        camera.open()
        camera.warm_up(warm_up_frames)
        color_bgr, depth_mm = camera.get_frame()
        if color_bgr is None or depth_mm is None:
            raise CameraFrameError("camera returned an empty color or depth frame")
        if not cv2.imwrite(str(color_path), color_bgr):
            raise OSError(f"failed to write color image: {color_path}")
        if not cv2.imwrite(str(depth_path), depth_mm):
            raise OSError(f"failed to write depth image: {depth_path}")
    finally:
        camera.close()

    return color_path, depth_path


def main(argv: Sequence[str] | None = None) -> None:
    args = parse_args(argv)
    color_path, depth_path = capture_single_frame(
        config_path=args.config,
        out_dir=args.out_dir,
        warm_up_frames=args.warm_up_frames,
    )
    color_bgr = cv2.imread(str(color_path), cv2.IMREAD_COLOR)
    depth_mm = cv2.imread(str(depth_path), cv2.IMREAD_UNCHANGED)
    if color_bgr is None or depth_mm is None:
        raise OSError("saved frame could not be read back")
    print(f"已采集成功，数据存放在：{args.out_dir.resolve()}")
    print(
        f"Color: {color_bgr.shape}, dtype={color_bgr.dtype} | "
        f"Depth: {depth_mm.shape}, dtype={depth_mm.dtype}"
    )
    print(f"Saved: {color_path}")
    print(f"Saved: {depth_path}")


if __name__ == "__main__":
    main()
