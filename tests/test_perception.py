"""Detection -> 3D point -> protective zone (fleet_ml.perception)."""

import math

import numpy as np
import pytest

from fleet_ml.perception import (
    Detection,
    person_point_optical,
    rectangle_zone,
    transform_from_quaternion,
    transform_point,
    zone_decision,
)

# 320x240 camera, hfov 1.047 rad -> fx = 160 / tan(0.5235) = 277.1
FX = 160 / math.tan(1.047 / 2)
K = [FX, 0, 160, 0, FX, 120, 0, 0, 1]


def test_person_point_uses_near_depth_in_box_not_background():
    depth = np.full((240, 320), 6.0, dtype=np.float32)   # far wall
    depth[60:200, 150:190] = 1.5                          # person, slightly right of centre
    det = Detection(x=140, y=50, w=60, h=160, score=0.9)  # box includes some wall
    x, y, z = person_point_optical(det, depth, K)
    assert z == pytest.approx(1.5)
    assert x == pytest.approx((170 - 160) * 1.5 / FX)
    assert y == pytest.approx((130 - 120) * 1.5 / FX)


def test_person_point_none_without_valid_depth():
    depth = np.full((240, 320), np.inf, dtype=np.float32)
    assert person_point_optical(Detection(10, 10, 50, 100, 0.9), depth, K) is None


def test_optical_to_base_link_with_standard_camera_rotation():
    # camera_rgb_optical_frame: rpy(-pi/2, 0, -pi/2) relative to a forward-facing camera at
    # (0.07, -0.05, 0.12) in base_link. Optical z (forward) must map to base_link +x.
    q = (-0.5, 0.5, -0.5, 0.5)  # quaternion (x, y, z, w) of rpy(-pi/2, 0, -pi/2)
    t = transform_from_quaternion((0.07, -0.05, 0.12), q)
    bx, by, bz = transform_point(t, (0.0, 0.0, 2.0))      # 2 m straight ahead
    assert (bx, by, bz) == pytest.approx((2.07, -0.05, 0.12), abs=1e-9)
    bx, by, _ = transform_point(t, (0.5, 0.0, 2.0))       # optical +x = robot's right (-y)
    assert by == pytest.approx(-0.55, abs=1e-9)


def test_zone_decision_stop_warning_and_clear():
    stop, warn = rectangle_zone(1.0, 0.4), rectangle_zone(2.0, 0.6)
    assert zone_decision([(0.8, 0.1, 0)], stop, warn).in_stop_zone
    d = zone_decision([(1.5, 0.0, 0)], stop, warn)
    assert (d.in_stop_zone, d.in_warning_zone, d.nearest_m) == (False, True, 1.5)
    d = zone_decision([(3.0, 0.0, 0), (0.5, 1.5, 0)], stop, warn)  # far, or off to the side
    assert not d.in_warning_zone
    assert zone_decision([], stop, warn).nearest_m is None


def test_scan_to_points_drops_invalid_returns():
    from fleet_ml.perception import scan_to_points

    pts = scan_to_points([1.0, float("inf"), 0.05, 2.0], -math.pi / 2, math.pi / 2, 0.12, 3.5)
    assert pts == pytest.approx(np.array([[0.0, -1.0], [-2.0, 0.0]]), abs=1e-9)  # -90, 180 deg


def test_unknown_obstacles_ignore_map_walls_and_other_robots():
    from fleet_ml.perception import static_obstacle_mask, unknown_obstacle_points

    occ = np.zeros((40, 40), dtype=int)       # 2 m x 2 m map at 5 cm, origin (0, 0)
    occ[:, 30] = 100                          # a wall at x = 1.5 m
    mask = static_obstacle_mask(occ, 0.05, inflate_m=0.1)
    zone = rectangle_zone(1.0, 0.4)
    # robot at map (0.8, 1.0) facing +x: base = map - (0.8, 1.0)
    base = np.array([[0.5, 0.35],   # unknown object ahead      -> kept
                     [0.7, 0.1],    # hits the wall (x=1.5 map) -> static
                     [0.4, -0.2],   # another robot             -> excluded
                     [1.5, 0.0]])   # outside the stop zone
    mp = base + np.array([0.8, 1.0])
    out = unknown_obstacle_points(base, mp, zone, mask, (0.0, 0.0), 0.05,
                                  exclude_xy=[(1.2, 0.8)])
    assert out.tolist() == [[0.5, 0.35]]
