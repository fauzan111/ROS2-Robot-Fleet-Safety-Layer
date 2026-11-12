"""From a person detection to "is someone in the protective zone?" (numpy only, unit-tested).

Pipeline per camera frame:
1. detector box (pixels) + aligned depth image -> 3D point in the camera optical frame
2. optical frame -> robot base_link (static transform from TF)
3. base_link point -> inside the stop zone? inside the warning zone?

Depth inside a person's box also contains background (floor, walls), so the distance is a
low percentile of the valid depths in the box's central columns: the person is the nearest
large surface in their own box.
"""

from __future__ import annotations

from collections.abc import Sequence
from dataclasses import dataclass

import numpy as np

from fleet_ml.zone import point_in_polygon


@dataclass(frozen=True)
class Detection:
    x: int
    y: int
    w: int
    h: int
    score: float


def person_point_optical(det: Detection, depth: np.ndarray, k: Sequence[float],
                         percentile: float = 20.0, min_valid: int = 20,
                         ) -> tuple[float, float, float] | None:
    """3D point (x right, y down, z forward; metres) of a detected person, or None.

    k: row-major 3x3 intrinsics (sensor_msgs/CameraInfo.k). Returns None if the box has
    too few valid depth pixels to trust.
    """
    h_img, w_img = depth.shape[:2]
    x0 = max(det.x + det.w // 5, 0)
    x1 = min(det.x + det.w - det.w // 5, w_img)
    y0, y1 = max(det.y, 0), min(det.y + det.h, h_img)
    roi = depth[y0:y1, x0:x1]
    valid = roi[np.isfinite(roi) & (roi > 0.05)]
    if valid.size < min_valid:
        return None
    z = float(np.percentile(valid, percentile))
    fx, cx, fy, cy = k[0], k[2], k[4], k[5]
    u = det.x + det.w / 2.0
    v = det.y + det.h / 2.0
    return (u - cx) * z / fx, (v - cy) * z / fy, z


def transform_point(t: np.ndarray, p: Sequence[float]) -> tuple[float, float, float]:
    """Apply a 4x4 homogeneous transform to a 3D point."""
    q = np.asarray(t, dtype=float) @ np.array([p[0], p[1], p[2], 1.0])
    return float(q[0]), float(q[1]), float(q[2])


def transform_from_quaternion(translation: Sequence[float],
                              quaternion_xyzw: Sequence[float]) -> np.ndarray:
    x, y, z, w = quaternion_xyzw
    r = np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])
    t = np.eye(4)
    t[:3, :3] = r
    t[:3, 3] = translation
    return t


@dataclass(frozen=True)
class ZoneDecision:
    in_stop_zone: bool
    in_warning_zone: bool
    nearest_m: float | None  # nearest person (planar distance from base_link), any zone


def zone_decision(points_base: Sequence[Sequence[float]], stop_zone: Sequence[Sequence[float]],
                  warning_zone: Sequence[Sequence[float]]) -> ZoneDecision:
    stop = any(point_in_polygon((p[0], p[1]), stop_zone) for p in points_base)
    warn = stop or any(point_in_polygon((p[0], p[1]), warning_zone) for p in points_base)
    nearest = min((float(np.hypot(p[0], p[1])) for p in points_base), default=None)
    return ZoneDecision(stop, warn, nearest)


def rectangle_zone(length_m: float, half_width_m: float) -> list[tuple[float, float]]:
    """A field in front of the robot: x in [0, length], |y| <= half_width (base_link)."""
    return [(0.0, -half_width_m), (length_m, -half_width_m),
            (length_m, half_width_m), (0.0, half_width_m)]
