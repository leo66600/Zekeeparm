"""Print saved camera intrinsics."""

from pathlib import Path

import numpy as np


INTRINSICS = Path(__file__).resolve().parents[1] / "config/calibration/orbbec_gemini2/intrinsics.npz"


def main() -> int:
    data = np.load(INTRINSICS)
    print("resolution:", data["resolution"])
    print("K:", data["camera_matrix"])
    print("D:", data["dist_coeffs"])
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
