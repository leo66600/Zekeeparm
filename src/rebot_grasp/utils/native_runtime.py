"""Small native-library compatibility helpers for the GraspNet runtime."""

from __future__ import annotations

import ctypes
from pathlib import Path
import sys


def preload_environment_libstdcpp() -> bool:
    """Load the active Python environment's C++ runtime before native modules.

    OpenCV/Open3D can otherwise load Ubuntu's older libstdc++ first.  The
    locally built PointNet2 extension then fails because the already-loaded
    library does not provide the ABI version it was linked against.
    """

    candidate = Path(sys.prefix) / "lib" / "libstdc++.so.6"
    if not candidate.is_file():
        return False
    ctypes.CDLL(str(candidate), mode=ctypes.RTLD_GLOBAL)
    return True
