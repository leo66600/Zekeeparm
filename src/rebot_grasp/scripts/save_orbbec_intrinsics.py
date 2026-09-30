"""Save the active Gemini2 runtime RGB calibration to this project.

This script opens only the camera. It never connects to or enables the robot.
"""

from __future__ import annotations

import argparse
import os
from pathlib import Path
import shutil
import sys
import time
from typing import Optional

import numpy as np


PROJECT_ROOT = Path(__file__).resolve().parents[1]
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

from drivers.camera import make_camera  # noqa: E402
from utils.camera_utils import load_config  # noqa: E402


def save_intrinsics(
    output: Path,
    camera_matrix: np.ndarray,
    dist_coeffs: np.ndarray,
    *,
    resolution: tuple[int, int],
    timestamp: Optional[str] = None,
) -> Optional[Path]:
    output = Path(output)
    K = np.asarray(camera_matrix, dtype=np.float64)
    D = np.asarray(dist_coeffs, dtype=np.float64).reshape(-1)
    width, height = [int(value) for value in resolution]
    if K.shape != (3, 3) or not np.all(np.isfinite(K)):
        raise ValueError("camera_matrix must be a finite 3x3 matrix")
    if D.size < 5 or not np.all(np.isfinite(D)):
        raise ValueError("dist_coeffs must contain at least five finite values")
    if width <= 0 or height <= 0:
        raise ValueError("resolution must be positive")

    output.parent.mkdir(parents=True, exist_ok=True)
    backup = None
    if output.exists():
        stamp = timestamp or time.strftime("%Y%m%d-%H%M%S")
        backup = output.with_name(
            f"{output.stem}.backup-{stamp}{output.suffix}"
        )
        counter = 1
        while backup.exists():
            backup = output.with_name(
                f"{output.stem}.backup-{stamp}-{counter}{output.suffix}"
            )
            counter += 1
        shutil.copy2(output, backup)

    temporary = output.with_name(f".{output.name}.tmp")
    try:
        with temporary.open("wb") as handle:
            np.savez(
                handle,
                camera_matrix=K,
                dist_coeffs=D,
                resolution=np.array([width, height], dtype=np.int32),
                source=np.array(
                    ["orbbec_active_profile_runtime_distortion"]
                ),
            )
        os.replace(temporary, output)
    finally:
        if temporary.exists():
            temporary.unlink()
    return backup


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Save active Gemini2 runtime RGB intrinsics"
    )
    parser.add_argument(
        "--config",
        default=str(PROJECT_ROOT / "config" / "default.yaml"),
    )
    parser.add_argument("--warmup", type=int, default=20)
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    cfg = load_config(args.config)
    cam_type = str(cfg.get("camera", {}).get("type", "")).lower()
    if "orbbec" not in cam_type:
        raise ValueError(
            f"This command is for Orbbec cameras, got {cam_type!r}"
        )

    camera = make_camera(cfg)
    try:
        camera.open()
        camera.warm_up(max(0, int(args.warmup)))
        color_bgr, _ = camera.get_frame()
        if color_bgr is None:
            raise RuntimeError("Could not capture a color frame")
        height, width = color_bgr.shape[:2]
        K = camera.K.copy()
        D = camera.D.copy()
    finally:
        camera.close()

    output = (
        PROJECT_ROOT
        / "config"
        / "calibration"
        / cam_type
        / "intrinsics.npz"
    )
    backup = save_intrinsics(
        output,
        K,
        D,
        resolution=(width, height),
    )
    if backup is not None:
        print(f"Old calibration backup: {backup}")
    print(f"Saved runtime calibration: {output}")
    print(f"Resolution: {width}x{height}")
    print("K:")
    print(K)
    print("D:")
    print(D.reshape(-1))
    print("Robot was not connected or enabled.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
