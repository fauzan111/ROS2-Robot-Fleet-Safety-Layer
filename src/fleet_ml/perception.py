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


# --- near field: 360 deg lidar, because the camera cannot see most of the stop zone up close


def scan_to_points(ranges: Sequence[float], angle_min: float, angle_increment: float,
                   range_min: float, range_max: float) -> np.ndarray:
    """LaserScan ranges -> (N, 2) points in the scan frame (invalid returns dropped)."""
    r = np.asarray(ranges, dtype=float)
    a = angle_min + angle_increment * np.arange(r.size)
    ok = np.isfinite(r) & (r >= range_min) & (r <= range_max)
    return np.column_stack((r[ok] * np.cos(a[ok]), r[ok] * np.sin(a[ok])))


def static_obstacle_mask(occupancy: np.ndarray, resolution: float,
                         inflate_m: float = 0.15) -> np.ndarray:
    """True where the map says something static is (occupied or unknown), grown by inflate_m
    to absorb localisation error. occupancy: (H, W) int grid from nav_msgs/OccupancyGrid."""
    mask = (occupancy > 50) | (occupancy < 0)
    r = int(np.ceil(inflate_m / resolution))
    if r <= 0:
        return mask
    out = mask.copy()
    h, w = mask.shape
    ys, xs = np.nonzero(mask)
    for dy in range(-r, r + 1):
        for dx in range(-r, r + 1):
            if dx * dx + dy * dy <= r * r:
                out[np.clip(ys + dy, 0, h - 1), np.clip(xs + dx, 0, w - 1)] = True
    return out


def unknown_obstacle_points(points_base: np.ndarray, points_map: np.ndarray, zone,
                            mask: np.ndarray, origin_xy: Sequence[float], resolution: float,
                            exclude_xy: Sequence[Sequence[float]] = (),
                            exclude_radius_m: float = 0.35) -> np.ndarray:
    """Laser points inside the zone (base_link) that are not on the static map (map frame)
    and not on another fleet robot. Points outside the map count as static (no stop)."""
    if len(points_base) == 0:
        return np.empty((0, 2))
    zx = [p[0] for p in zone]
    zy = [p[1] for p in zone]
    box = ((points_base[:, 0] >= min(zx)) & (points_base[:, 0] <= max(zx))
           & (points_base[:, 1] >= min(zy)) & (points_base[:, 1] <= max(zy)))
    keep = []
    h, w = mask.shape
    for i in np.nonzero(box)[0]:
        if not point_in_polygon((points_base[i, 0], points_base[i, 1]), zone):
            continue
        mx, my = points_map[i]
        cx = int((mx - origin_xy[0]) / resolution)
        cy = int((my - origin_xy[1]) / resolution)
        if not (0 <= cx < w and 0 <= cy < h) or mask[cy, cx]:
            continue
        if any(np.hypot(mx - ox, my - oy) <= exclude_radius_m for ox, oy in exclude_xy):
            continue
        keep.append(i)
    return points_base[keep] if keep else np.empty((0, 2))
