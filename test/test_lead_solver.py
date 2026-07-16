#!/usr/bin/env python3
"""Unit tests for the LeadSolver core (pure Python, no ROS)."""
import math
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from counter_uav_turret.lead_solver import LeadSolver  # noqa: E402
from counter_uav_turret.geometry import aim_angles     # noqa: E402


def test_stationary_target_aims_at_it():
    pos = np.array([100.0, 0.0, 20.0])
    aim, pan, tilt, t = LeadSolver(v_effector=250.0).solve(pos, [0, 0, 0])
    p0, t0 = aim_angles(pos)
    assert abs(pan - p0) < 1e-6 and abs(tilt - t0) < 1e-6      # no lead for a still target
    assert abs(t - np.linalg.norm(pos) / 250.0) < 1e-3


def test_lead_is_ahead_of_current_bearing():
    pos = np.array([100.0, 0.0, 20.0])
    vel = np.array([0.0, 30.0, 0.0])                          # moving +Y (left)
    _, pan, _, t = LeadSolver(v_effector=250.0).solve(pos, vel)
    pan0, _ = aim_angles(pos)
    assert pan > pan0                                         # aim leads in the direction of motion
    assert t > 0


def test_infinite_effector_speed_kills_lead():
    pos = np.array([100.0, 0.0, 20.0])
    vel = np.array([0.0, 50.0, 0.0])
    _, pan, _, t = LeadSolver(v_effector=1e12).solve(pos, vel)
    pan0, _ = aim_angles(pos)
    assert abs(pan - pan0) < 1e-4 and t < 1e-3               # instantaneous round → no lead


def test_converges():
    pos = np.array([80.0, 10.0, 15.0])
    vel = np.array([-5.0, 25.0, 1.0])
    aim, _, _, t = LeadSolver(v_effector=250.0).solve(pos, vel)
    # self-consistency: time of flight to the aim point equals the lead time used
    assert abs(np.linalg.norm(aim) / 250.0 - t) < 1e-3


if __name__ == '__main__':
    fns = [f for n, f in sorted(globals().items()) if n.startswith('test_')]
    for f in fns:
        f(); print(f'  ok  {f.__name__}')
    print(f'\n=== {len(fns)}/{len(fns)} passed ===')
