#!/usr/bin/env python3
"""Shared camera/turret geometry (pure NumPy, no ROS).

World frame (ROS REP-103): X forward, Y left, Z up; camera/turret at the origin.
A gimbal pan (yaw about Z) and tilt (pitch) define the camera boresight.
"""
from __future__ import annotations

import math
import numpy as np


def boresight_frame(pan: float, tilt: float):
    """Return (forward, right, down) unit vectors for the given pan/tilt."""
    fwd = np.array([math.cos(tilt) * math.cos(pan),
                    math.cos(tilt) * math.sin(pan),
                    math.sin(tilt)])
    world_up = np.array([0.0, 0.0, 1.0])
    right = np.cross(fwd, world_up)
    n = np.linalg.norm(right)
    right = np.array([0.0, -1.0, 0.0]) if n < 1e-6 else right / n
    down = np.cross(fwd, right)
    down /= (np.linalg.norm(down) + 1e-12)
    return fwd, right, down


def pixel_to_ray(pan: float, tilt: float, u: float, v: float,
                 f_px: float, cx: float, cy: float) -> np.ndarray:
    """Inverse projection: pixel (u, v) → unit ray direction in the world frame."""
    fwd, right, down = boresight_frame(pan, tilt)
    x = (u - cx) / f_px
    y = (v - cy) / f_px
    ray = fwd + x * right + y * down
    return ray / (np.linalg.norm(ray) + 1e-12)


def aim_angles(point: np.ndarray):
    """pan/tilt (rad) that point the boresight straight at a world point."""
    return math.atan2(point[1], point[0]), math.atan2(point[2], math.hypot(point[0], point[1]))
