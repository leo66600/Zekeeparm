"""Persistent debug artifacts for one visual-grasp attempt."""

from __future__ import annotations

from datetime import datetime
import json
from pathlib import Path
from typing import Any

import cv2
import numpy as np


def _json_value(value: Any) -> Any:
    if isinstance(value, np.ndarray):
        return value.tolist()
    if isinstance(value, (np.integer, np.floating)):
        return value.item()
    if isinstance(value, Path):
        return str(value)
    if isinstance(value, dict):
        return {str(key): _json_value(item) for key, item in value.items()}
    if isinstance(value, (list, tuple)):
        return [_json_value(item) for item in value]
    return value


class GraspDebugBundle:
    def __init__(self, path: Path) -> None:
        self.path = path
        self._metadata: dict[str, Any] = {
            "outcome": "in_progress",
            "stages": {},
        }
        self._write_metadata()

    @classmethod
    def create(
        cls,
        root: Path,
        *,
        attempt_id: str | None = None,
    ) -> "GraspDebugBundle":
        identifier = attempt_id or datetime.now().strftime(
            "%Y%m%d_%H%M%S_%f"
        )
        path = Path(root) / identifier
        path.mkdir(parents=True, exist_ok=False)
        return cls(path)

    def save_frame(
        self,
        color_bgr: np.ndarray,
        depth_mm: np.ndarray,
    ) -> None:
        if not cv2.imwrite(str(self.path / "color.png"), color_bgr):
            raise RuntimeError("failed to save debug color frame")
        np.save(self.path / "depth.npy", np.asarray(depth_mm))
        cv2.imwrite(
            str(self.path / "depth.png"),
            np.asarray(depth_mm, dtype=np.uint16),
        )

    def save_mask(self, mask: np.ndarray) -> None:
        cv2.imwrite(
            str(self.path / "mask.png"),
            (np.asarray(mask) > 0).astype(np.uint8) * 255,
        )

    def record(self, stage: str, **values: Any) -> None:
        self._metadata["stages"][str(stage)] = _json_value(values)
        self._write_metadata()

    def finalize(self, outcome: str, **values: Any) -> None:
        self._metadata["outcome"] = str(outcome)
        if values:
            self._metadata["result"] = _json_value(values)
        self._write_metadata()

    def _write_metadata(self) -> None:
        (self.path / "metadata.json").write_text(
            json.dumps(
                _json_value(self._metadata),
                ensure_ascii=False,
                indent=2,
                sort_keys=True,
            )
            + "\n",
            encoding="utf-8",
        )
