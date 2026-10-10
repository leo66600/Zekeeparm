from __future__ import annotations

from pathlib import Path
from typing import Any, Mapping


def resolve_robot_model_config(
    project_root: str | Path,
    robot_cfg: Mapping[str, Any],
) -> tuple[Path, str]:
    root = Path(project_root).expanduser().resolve()
    raw_path = str(robot_cfg.get("urdf_path", "")).strip()
    if not raw_path:
        raise ValueError("robot.urdf_path is required")
    path = Path(raw_path).expanduser()
    if not path.is_absolute():
        path = root / path
    path = path.resolve()
    if not path.is_file():
        raise FileNotFoundError(f"Robot URDF not found: {path}")
    frame = str(robot_cfg.get("end_effector_frame", "")).strip()
    if not frame:
        raise ValueError("robot.end_effector_frame is required")
    return path, frame
