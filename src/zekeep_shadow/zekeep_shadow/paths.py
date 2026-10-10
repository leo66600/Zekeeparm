"""Locate the perception source tree from source or installed ROS nodes."""

import os
from pathlib import Path


def default_perception_root() -> str:
    workspace = os.environ.get("ZKEEP_WS")
    if workspace:
        return str(Path(workspace).expanduser() / "src" / "zekeep_grasp")
    for parent in Path(__file__).resolve().parents:
        root = parent / "src" / "zekeep_grasp"
        if (root / "utils" / "camera_utils.py").is_file():
            return str(root)
    return str(Path.cwd() / "src" / "zekeep_grasp")
