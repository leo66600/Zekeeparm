"""Confirm existing calibration for unchanged hardware, or bind fresh calibration.

Opens only the camera. Never connects to the robot.
"""
from pathlib import Path
import argparse
import sys

PROJECT_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PROJECT_ROOT))
from drivers.camera import make_camera
from utils.camera_utils import load_config
from utils.calibration_identity import confirm_calibration_identity


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", default=str(PROJECT_ROOT / "config/default.yaml"))
    parser.add_argument("--confirm-same-installation", action="store_true", required=True,
                        help="Confirm these files were calibrated for this exact camera, robot, mounting and scene")
    args = parser.parse_args()
    cfg = load_config(args.config)
    camera = make_camera(cfg)
    try:
        camera.open()
        print(confirm_calibration_identity(PROJECT_ROOT, cfg, camera.serial))
    finally:
        camera.close()


if __name__ == "__main__":
    main()
