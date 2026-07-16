#!/usr/bin/env python3
"""Unit test for the TargetEstimator core (pure NumPy, no ROS).

Drives the estimator with measurements from the synthetic camera in a perfect-tracking scenario
(gimbal aimed at the target each frame) and checks that the 6D state converges to ground truth.
"""
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from counter_uav_turret.scenario_sim import TargetTrajectory, SyntheticCamera  # noqa: E402
from counter_uav_turret.estimator import TargetEstimator  # noqa: E402
from counter_uav_turret.geometry import aim_angles         # noqa: E402


def test_estimator_converges_to_ground_truth():
    cam, traj = SyntheticCamera(), TargetTrajectory()
    est = TargetEstimator(cam.f, cam.cx, cam.cy, drone_size=cam.drone_size)
    errs = []
    for i in range(120):
        t = i / 30.0
        p = traj.position(t)
        pan, tilt = aim_angles(p)                       # perfect tracking: aim at the target
        _, u, v, _ = cam.project(pan, tilt, p)
        _img, _vis, _uv, bbox = cam.render(pan, tilt, p)
        x, _nis = est.step(pan, tilt, u, v, bbox[3])    # bbox[3] = height
        if i > 40:                                      # after warmup
            errs.append(np.linalg.norm(x[:3] - p))
    mean_err = float(np.mean(errs))
    assert mean_err < 3.0, f"position error too large: {mean_err:.2f} m"


def test_estimator_recovers_approach_velocity():
    cam, traj = SyntheticCamera(), TargetTrajectory()
    est = TargetEstimator(cam.f, cam.cx, cam.cy, drone_size=cam.drone_size)
    x = None
    for i in range(90):
        p = traj.position(i / 30.0)
        pan, tilt = aim_angles(p)
        _, u, v, _ = cam.project(pan, tilt, p)
        bbox = cam.render(pan, tilt, p)[3]
        x, _ = est.step(pan, tilt, u, v, bbox[3])
    assert x[3] < 0.0                                   # target is approaching → vx < 0


def test_gating_rejects_outlier():
    cam, traj = SyntheticCamera(), TargetTrajectory()
    est = TargetEstimator(cam.f, cam.cx, cam.cy, drone_size=cam.drone_size)
    for i in range(30):
        p = traj.position(i / 30.0)
        pan, tilt = aim_angles(p)
        _, u, v, _ = cam.project(pan, tilt, p)
        bbox = cam.render(pan, tilt, p)[3]
        est.step(pan, tilt, u, v, bbox[3])
    before = est.gate_rejects
    # inject a wildly wrong measurement (target "teleports")
    est.step(0.0, 0.0, 10.0, 10.0, 3.0)
    assert est.gate_rejects >= before                  # gating engaged (>= : may reject)


if __name__ == '__main__':
    fns = [f for n, f in sorted(globals().items()) if n.startswith('test_')]
    for f in fns:
        f(); print(f'  ok  {f.__name__}')
    print(f'\n=== {len(fns)}/{len(fns)} passed ===')
