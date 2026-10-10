from __future__ import annotations

from dataclasses import dataclass
from typing import Optional

import numpy as np
import cv2


@dataclass
class Detection:
    name: str
    confidence: float
    bbox: tuple[int, int, int, int]
    mask: np.ndarray
    position_base: Optional[np.ndarray] = None


def detect_red_blocks(image: np.ndarray, min_area: int = 180) -> list[Detection]:
    """Find red connected regions in an aligned BGR frame."""
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(hsv, (0, 140, 65), (6, 255, 255))
    mask |= cv2.inRange(hsv, (174, 140, 65), (179, 255, 255))
    blue, green, red = cv2.split(image)
    mask[(red.astype(np.uint16) < 2 * green) |
         (red.astype(np.uint16) < 2 * blue)] = 0
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, np.ones((3, 3), np.uint8))
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    result = []
    for contour in contours:
        area = cv2.contourArea(contour)
        if area < min_area:
            continue
        x, y, w, h = cv2.boundingRect(contour)
        hull_area = cv2.contourArea(cv2.convexHull(contour))
        if not 0.4 <= w / h <= 2.5 or area / (w * h) < 0.5 or area / hull_area < 0.8:
            continue
        component = np.zeros(mask.shape, np.uint8)
        cv2.drawContours(component, [contour], -1, 1, cv2.FILLED)
        result.append(Detection("red block", 1.0, (x, y, x + w, y + h), component.astype(bool)))
    return result


def estimate_camera_point(detection: Detection, depth_mm: np.ndarray, K: np.ndarray,
                          min_depth_m: float = 0.10, max_depth_m: float = 2.0):
    """Back-project mask center using valid local median depth, in meters."""
    mask = np.asarray(detection.mask, dtype=bool)
    if mask.shape != depth_mm.shape or K.shape != (3, 3):
        return None
    v, u = np.nonzero(mask)
    if not len(u):
        return None
    u0, v0 = float(np.median(u)), float(np.median(v))
    cx, cy = int(round(u0)), int(round(v0))
    y0, y1 = max(0, cy - 5), min(mask.shape[0], cy + 6)
    x0, x1 = max(0, cx - 5), min(mask.shape[1], cx + 6)
    local = depth_mm[y0:y1, x0:x1][mask[y0:y1, x0:x1]]
    values = local.astype(np.float64) / 1000.0
    values = values[np.isfinite(values) & (values >= min_depth_m) & (values <= max_depth_m)]
    if values.size < 20:
        values = depth_mm[mask].astype(np.float64) / 1000.0
        values = values[np.isfinite(values) & (values >= min_depth_m) & (values <= max_depth_m)]
    if values.size < 20:
        return None
    z = float(np.median(values))
    return np.array([(u0 - K[0, 2]) * z / K[0, 0],
                     (v0 - K[1, 2]) * z / K[1, 1], z], dtype=np.float64)


if __name__ == "__main__":
    mask = np.zeros((8, 8), dtype=bool)
    mask[1:7, 1:7] = True
    detection = Detection("red block", .9, (1, 1, 7, 7), mask)
    depth = np.full((8, 8), 500, dtype=np.uint16)
    K = np.array([[100, 0, 3.5], [0, 100, 3.5], [0, 0, 1]])
    point = estimate_camera_point(detection, depth, K)
    assert point is not None and np.allclose(point, [0, 0, .5])
    image = np.zeros((30, 30, 3), np.uint8)
    image[5:25, 5:25] = (0, 0, 255)
    assert len(detect_red_blocks(image)) == 1
    hsv = np.zeros((60, 60, 3), np.uint8)
    hsv[10:30, 10:30] = (2, 210, 180)
    hsv[15:50, 30:55] = (9, 110, 190)  # Skin-like red touching the block.
    assert len(detect_red_blocks(cv2.cvtColor(hsv, cv2.COLOR_HSV2BGR))) == 1
