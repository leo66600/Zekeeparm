"""Back-project one depth pixel into the camera coordinate system.

Usage:
    python3 scripts/backproject_pixel.py
    python3 scripts/backproject_pixel.py --u 640 --v 400
"""

from __future__ import annotations

import argparse
from pathlib import Path
from typing import Sequence

import cv2
import numpy as np

PROJECT_ROOT = Path(__file__).resolve().parents[1]


def parse_args(argv: Sequence[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Back-project one depth pixel using camera intrinsics."
    )
    parser.add_argument(
        "--intrinsics",
        type=Path,
        default=PROJECT_ROOT / "config" / "calibration" / "orbbec_gemini2" / "intrinsics.npz",
        help="Path to intrinsics.npz.",
    )
    parser.add_argument(
        "--depth",
        type=Path,
        default=PROJECT_ROOT / "data" / "rgbd_task2" / "depth_mm_16uc1.png",
        help="Path to 16UC1 depth image in millimeters.",
    )
    parser.add_argument("--u", type=int, help="Pixel x coordinate. Defaults to image center.")
    parser.add_argument("--v", type=int, help="Pixel y coordinate. Defaults to image center.")
    return parser.parse_args(argv)


def backproject_pixel(
    *,
    u: int,
    v: int,
    depth_mm: np.ndarray,
    camera_matrix: np.ndarray,
) -> tuple[float, float, float, int]:
    """Return camera-frame XYZ in meters and the original depth in millimeters."""
    if depth_mm.ndim != 2:
        raise ValueError("depth image must be a single-channel 16UC1 image")
    height, width = depth_mm.shape
    if not (0 <= u < width and 0 <= v < height):
        raise ValueError(f"pixel ({u}, {v}) is outside depth image size {width}x{height}")

    depth_value_mm = int(depth_mm[v, u])
    z = float(depth_value_mm) / 1000.0
    if z <= 0.0:
        raise ValueError("selected depth is invalid; choose another pixel")

    intrinsic = np.asarray(camera_matrix, dtype=np.float64)
    x = (u - intrinsic[0, 2]) * z / intrinsic[0, 0]
    y = (v - intrinsic[1, 2]) * z / intrinsic[1, 1]
    return float(x), float(y), float(z), depth_value_mm


def main(argv: Sequence[str] | None = None) -> None:
    args = parse_args(argv)
    if (args.u is None) != (args.v is None):
        raise ValueError("--u and --v must be provided together")

    intrinsics = np.load(args.intrinsics)
    camera_matrix = intrinsics["camera_matrix"]
    depth_mm = cv2.imread(str(args.depth), cv2.IMREAD_UNCHANGED)
    if depth_mm is None:
        raise OSError(f"failed to read depth image: {args.depth}")

    height, width = depth_mm.shape
    u = width // 2 if args.u is None else args.u
    v = height // 2 if args.v is None else args.v
    x, y, z, depth_value_mm = backproject_pixel(
        u=u,
        v=v,
        depth_mm=depth_mm,
        camera_matrix=camera_matrix,
    )

    print(f"pixel=({u}, {v}) depth_mm={depth_value_mm}")
    print(f"camera_xyz_m=({x:.6f}, {y:.6f}, {z:.6f})")


if __name__ == "__main__":
    main()
