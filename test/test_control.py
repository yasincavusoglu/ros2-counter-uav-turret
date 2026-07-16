#!/usr/bin/env python3
"""Unit tests for gimbal_control + turret_plant (closed loop) and the engagement FSM."""
import math
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from counter_uav_turret.gimbal_control import PIDGimbalController, wrap  # noqa: E402
from counter_uav_turret.turret_plant import GimbalPlant  # noqa: E402
from counter_uav_turret.engagement_fsm import EngagementFSM, SEARCH, ENGAGE  # noqa: E402


# ── control + plant closed loop ──────────────────────────────────────────────
def _run_loop(pan_sp, tilt_sp, steps=400, dt=1 / 60.0):
    ctrl, plant = PIDGimbalController(), GimbalPlant()
    for _ in range(steps):
        pr, tr = ctrl.compute(pan_sp, tilt_sp, plant.pan, plant.tilt, dt)
        plant.step(pr, tr, dt)
    return plant


def test_gimbal_converges_to_setpoint():
    plant = _run_loop(0.30, 0.15)
    assert abs(wrap(plant.pan - 0.30)) < 0.01
    assert abs(plant.tilt - 0.15) < 0.01


def test_slew_rate_is_limited():
    ctrl, plant = PIDGimbalController(max_rate=3.0), GimbalPlant(max_rate=3.0)
    peak = 0.0
    for _ in range(200):                       # big step → controller saturates
        pr, tr = ctrl.compute(1.5, 0.0, plant.pan, plant.tilt, 1 / 60.0)
        plant.step(pr, tr, 1 / 60.0)
        peak = max(peak, abs(plant.pan_rate))
    assert peak <= 3.0 + 1e-6


def test_tilt_limit_enforced():
    plant = _run_loop(0.0, 5.0)                 # command far above the tilt limit
    assert plant.tilt <= 1.4 + 1e-6


# ── engagement FSM ───────────────────────────────────────────────────────────
def test_fsm_search_without_track():
    fsm = EngagementFSM()
    mode, fire = fsm.update(has_track=False, aim_error=0.0, in_zone=True,
                            watchdog_fresh=False, safe_mode=False)
    assert mode == SEARCH and fire is False


def test_fsm_reaches_engage_and_permits_fire():
    fsm = EngagementFSM(lock_frames=5)
    mode = fire = None
    for _ in range(8):
        mode, fire = fsm.update(True, aim_error=0.002, in_zone=True,
                                watchdog_fresh=True, safe_mode=False)
    assert mode == ENGAGE and fire is True


def test_fsm_safe_mode_forbids_fire():
    fsm = EngagementFSM(lock_frames=1)
    for _ in range(5):
        mode, fire = fsm.update(True, 0.001, True, True, safe_mode=True)
    assert fire is False and mode != ENGAGE


def test_fsm_out_of_zone_no_fire():
    fsm = EngagementFSM(lock_frames=1)
    for _ in range(5):
        _, fire = fsm.update(True, 0.001, in_zone=False, watchdog_fresh=True, safe_mode=False)
    assert fire is False


def test_fsm_losing_track_returns_to_search():
    fsm = EngagementFSM(lock_frames=1)
    for _ in range(5):
        fsm.update(True, 0.001, True, True, False)
    mode, fire = fsm.update(False, 0.001, True, True, False)
    assert mode == SEARCH and fire is False


if __name__ == '__main__':
    fns = [f for n, f in sorted(globals().items()) if n.startswith('test_')]
    for f in fns:
        f(); print(f'  ok  {f.__name__}')
    print(f'\n=== {len(fns)}/{len(fns)} passed ===')
