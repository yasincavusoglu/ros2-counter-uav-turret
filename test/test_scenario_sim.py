#!/usr/bin/env python3
"""Unit tests for the scenario_sim core (pure NumPy/OpenCV, no ROS).

Run:  python3 test/test_scenario_sim.py   (or: pytest)
"""
import math
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from counter_uav_turret.scenario_sim import TargetTrajectory, SyntheticCamera  # noqa: E402


def _aim_at(p):
    """pan/tilt that points the boresight straight at world point p."""
    return math.atan2(p[1], p[0]), math.atan2(p[2], math.hypot(p[0], p[1]))


def test_projection_is_centered_when_aimed():
    """When the gimbal points at the target, it must render at the image center."""
    cam, traj = SyntheticCamera(), TargetTrajectory()
    for t in (0.0, 1.7, 3.3, 5.0):
        p = traj.position(t)
        pan, tilt = _aim_at(p)
        vis, u, v, _ = cam.project(pan, tilt, p)
        assert vis
        assert abs(u - cam.cx) < 1.0 and abs(v - cam.cy) < 1.0, (u, v)


def test_perspective_sizing_grows_when_closer():
    """A closer target must render a larger bounding box."""
    cam = SyntheticCamera()
    far = np.array([120.0, 0.0, 0.0])
    near = np.array([40.0, 0.0, 0.0])
    _, _, _, bb_far = cam.render(0.0, 0.0, far)
    _, _, _, bb_near = cam.render(0.0, 0.0, near)
    assert bb_far is not None and bb_near is not None
    assert bb_near[2] > bb_far[2]                      # width grows


def test_behind_camera_is_invisible():
    """A target behind the boresight must not be visible."""
    cam = SyntheticCamera()
    behind = np.array([-50.0, 0.0, 0.0])               # boresight is +X
    vis, _, _, depth = cam.project(0.0, 0.0, behind)
    assert not vis and depth < 0


def test_acquire_window_exists_for_fixed_gimbal():
    """With a fixed forward gimbal the target must be in-frame for a real acquisition window."""
    cam, traj = SyntheticCamera(), TargetTrajectory()
    seen = sum(cam.project(0.0, 0.0, traj.position(i / 30.0))[0] for i in range(450))
    assert seen > 60, f"acquisition window too small: {seen} frames"


def test_frame_shape_and_type():
    cam = SyntheticCamera()
    img, vis, _, _ = cam.render(0.0, 0.0, TargetTrajectory().position(1.0))
    assert img.shape == (cam.h, cam.w, 3) and img.dtype == np.uint8


if __name__ == '__main__':
    fns = [f for name, f in sorted(globals().items()) if name.startswith('test_')]
    passed = 0
    for f in fns:
        f()
        print(f"  ok  {f.__name__}")
        passed += 1
    print(f"\n=== {passed}/{len(fns)} passed ===")
